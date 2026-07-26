#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "board_pinmap.h"

namespace
{
constexpr uint32_t kConsoleBaud = 115200UL;
constexpr uint32_t kSensorSpiHz = 1000000UL;
constexpr uint16_t kSpiSensorSamples = 2000U;
constexpr uint16_t kMplSamples = 1000U;
constexpr uint16_t kLoRaSamplesPerFrequency = 1000U;

constexpr uint8_t kWhoAmIRegister = 0x0FU;
constexpr uint8_t kH3lExpectedWhoAmI = 0x32U;
constexpr uint8_t kH3lCtrlReg1 = 0x20U;
constexpr uint8_t kLsmExpectedWhoAmI = 0x6CU;
constexpr uint8_t kLsmCtrl1Xl = 0x10U;

constexpr uint8_t kMplAddress = 0x60U;
constexpr uint8_t kMplWhoAmIRegister = 0x0CU;
constexpr uint8_t kMplExpectedWhoAmI = 0xC4U;
constexpr uint8_t kMplCtrlReg1 = 0x26U;

constexpr uint8_t kLoRaVersionRegister = 0x42U;
constexpr uint8_t kLoRaExpectedVersion = 0x12U;
constexpr uint8_t kLoRaFrfMsbRegister = 0x06U;

constexpr uint8_t kPyroPins[] = {
    BoardPinMap::DroguePyro::gpio1Pin,
    BoardPinMap::DroguePyro::gpio2Pin,
    BoardPinMap::MainPyro::gpio1Pin,
    BoardPinMap::MainPyro::gpio2Pin,
};
constexpr uint32_t kGpsBaudCandidates[] = {9600UL, 38400UL, 115200UL};
constexpr uint32_t kGpsProbeWindowMs = 5000UL;

struct ByteStats
{
    uint32_t total = 0UL;
    uint32_t expected = 0UL;
    uint32_t zero = 0UL;
    uint32_t ff = 0UL;
    uint32_t other = 0UL;
    uint8_t last = 0U;

    void add(uint8_t value, uint8_t expectedValue)
    {
        ++total;
        last = value;
        if (value == expectedValue)
        {
            ++expected;
        }
        else if (value == 0x00U)
        {
            ++zero;
        }
        else if (value == 0xFFU)
        {
            ++ff;
        }
        else
        {
            ++other;
        }
    }

    bool perfect() const
    {
        return total > 0UL && expected == total;
    }
};

struct GpsProbeResult
{
    uint32_t baud = 0UL;
    uint32_t bytes = 0UL;
    uint32_t validNmea = 0UL;
    uint32_t invalidNmea = 0UL;
    uint32_t validUbx = 0UL;
    uint32_t monVerResponses = 0UL;

    bool receivePass() const
    {
        return bytes > 0UL && (validNmea > 0UL || validUbx > 0UL);
    }

    bool transmitPass() const
    {
        return monVerResponses > 0UL;
    }
};

uint32_t runNumber = 0UL;

void printHexByte(uint8_t value)
{
    Serial.print("0x");
    if (value < 0x10U)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void printPassFail(const char *name, bool pass)
{
    Serial.print("RESULT ");
    Serial.print(name);
    Serial.print(' ');
    Serial.println(pass ? "PASS" : "FAIL");
}

void printStats(const char *name, const ByteStats &stats)
{
    Serial.print("STRESS ");
    Serial.print(name);
    Serial.print(" expected=");
    Serial.print(stats.expected);
    Serial.print('/');
    Serial.print(stats.total);
    Serial.print(" zero=");
    Serial.print(stats.zero);
    Serial.print(" ff=");
    Serial.print(stats.ff);
    Serial.print(" other=");
    Serial.print(stats.other);
    Serial.print(" last=");
    printHexByte(stats.last);
    Serial.println();
}

void holdSafetyCriticalPinsInactive()
{
    for (const uint8_t pin : kPyroPins)
    {
        pinMode(pin, INPUT_PULLDOWN);
    }

    pinMode(BoardPinMap::SparkFunSx1276_1W::rxEnablePin, OUTPUT);
    pinMode(BoardPinMap::SparkFunSx1276_1W::txEnablePin, OUTPUT);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::rxEnablePin, LOW);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::txEnablePin, LOW);
}

void deselectSpiDevices()
{
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::SparkFunSx1276_1W::ssPin, OUTPUT);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::ssPin, HIGH);
}

void beginBuses()
{
    deselectSpiDevices();

    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();

    SPI1.setMISO(BoardPinMap::Spi1Bus::misoPin);
    SPI1.setMOSI(BoardPinMap::Spi1Bus::mosiPin);
    SPI1.setSCK(BoardPinMap::Spi1Bus::sckPin);
    SPI1.begin();

    TwoWire &wire = BoardPinMap::MPL3115A2::wire();
    wire.setSDA(BoardPinMap::MPL3115A2::sdaPin);
    wire.setSCL(BoardPinMap::MPL3115A2::sclPin);
    wire.begin();
    wire.setClock(BoardPinMap::I2c0Bus::clockHz);

    BoardPinMap::UbloxM6::serial().setRX(BoardPinMap::UbloxM6::rxPin);
    BoardPinMap::UbloxM6::serial().setTX(BoardPinMap::UbloxM6::txPin);
}

uint8_t readSpi0Register(uint8_t csPin, uint8_t address)
{
    SPI.beginTransaction(SPISettings(kSensorSpiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin, LOW);
    const uint8_t command = static_cast<uint8_t>(address | 0x80U);
    (void)SPI.transfer(command);
    const uint8_t value = SPI.transfer(0x00U);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    return value;
}

void writeSpi0Register(uint8_t csPin, uint8_t address, uint8_t value)
{
    SPI.beginTransaction(SPISettings(kSensorSpiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin, LOW);
    (void)SPI.transfer(static_cast<uint8_t>(address & 0x7FU));
    (void)SPI.transfer(value);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
}

bool testSpiSensor(const char *name,
                   uint8_t csPin,
                   uint8_t expectedWhoAmI,
                   uint8_t writableRegister)
{
    deselectSpiDevices();
    const uint8_t initialWhoAmI = readSpi0Register(csPin, kWhoAmIRegister);
    const uint8_t originalRegister = readSpi0Register(csPin, writableRegister);
    writeSpi0Register(csPin, writableRegister, originalRegister);
    const uint8_t registerAfterWrite = readSpi0Register(csPin, writableRegister);
    const bool writeReadbackPass = registerAfterWrite == originalRegister;

    Serial.print("IDENTITY ");
    Serial.print(name);
    Serial.print(" who=");
    printHexByte(initialWhoAmI);
    Serial.print(" expected=");
    printHexByte(expectedWhoAmI);
    Serial.print(" write_same=");
    Serial.println(writeReadbackPass ? "PASS" : "FAIL");

    ByteStats stats;
    for (uint16_t sample = 0U; sample < kSpiSensorSamples; ++sample)
    {
        stats.add(readSpi0Register(csPin, kWhoAmIRegister), expectedWhoAmI);
        delay(1);
    }
    printStats(name, stats);

    const bool pass = initialWhoAmI == expectedWhoAmI &&
                      writeReadbackPass &&
                      stats.perfect();
    printPassFail(name, pass);
    return pass;
}

bool i2cReadRegister(uint8_t address, uint8_t reg, uint8_t &value)
{
    TwoWire &wire = BoardPinMap::MPL3115A2::wire();
    wire.beginTransmission(address);
    wire.write(reg);
    if (wire.endTransmission(false) != 0U)
    {
        return false;
    }
    if (wire.requestFrom(address, static_cast<uint8_t>(1U)) != 1U)
    {
        return false;
    }
    value = static_cast<uint8_t>(wire.read());
    return true;
}

bool i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    TwoWire &wire = BoardPinMap::MPL3115A2::wire();
    wire.beginTransmission(address);
    wire.write(reg);
    wire.write(value);
    return wire.endTransmission() == 0U;
}

void scanI2c0()
{
    TwoWire &wire = BoardPinMap::MPL3115A2::wire();
    Serial.print("I2C0_FOUND");
    for (uint8_t address = 1U; address < 127U; ++address)
    {
        wire.beginTransmission(address);
        if (wire.endTransmission() == 0U)
        {
            Serial.print(' ');
            printHexByte(address);
        }
    }
    Serial.println();
}

bool testMpl()
{
    scanI2c0();

    uint8_t whoAmI = 0U;
    const bool initialReadPass = i2cReadRegister(kMplAddress, kMplWhoAmIRegister, whoAmI);
    uint8_t originalCtrl = 0U;
    const bool ctrlReadPass = i2cReadRegister(kMplAddress, kMplCtrlReg1, originalCtrl);
    const bool ctrlWritePass = ctrlReadPass &&
                               i2cWriteRegister(kMplAddress, kMplCtrlReg1, originalCtrl);
    uint8_t ctrlAfterWrite = 0U;
    const bool writeReadbackPass = ctrlWritePass &&
                                   i2cReadRegister(kMplAddress, kMplCtrlReg1, ctrlAfterWrite) &&
                                   ctrlAfterWrite == originalCtrl;

    ByteStats stats;
    uint32_t transportErrors = 0UL;
    for (uint16_t sample = 0U; sample < kMplSamples; ++sample)
    {
        uint8_t value = 0U;
        if (i2cReadRegister(kMplAddress, kMplWhoAmIRegister, value))
        {
            stats.add(value, kMplExpectedWhoAmI);
        }
        else
        {
            ++transportErrors;
            stats.add(0U, kMplExpectedWhoAmI);
        }
        delay(1);
    }

    TwoWire &wire = BoardPinMap::MPL3115A2::wire();
    wire.end();
    pinMode(BoardPinMap::MPL3115A2::sdaPin, INPUT);
    pinMode(BoardPinMap::MPL3115A2::sclPin, INPUT);
    delay(2);
    const int sda = digitalRead(BoardPinMap::MPL3115A2::sdaPin);
    const int scl = digitalRead(BoardPinMap::MPL3115A2::sclPin);
    wire.setSDA(BoardPinMap::MPL3115A2::sdaPin);
    wire.setSCL(BoardPinMap::MPL3115A2::sclPin);
    wire.begin();
    wire.setClock(BoardPinMap::I2c0Bus::clockHz);
    Serial.print("IDENTITY MPL3115A2 who=");
    printHexByte(whoAmI);
    Serial.print(" expected=");
    printHexByte(kMplExpectedWhoAmI);
    Serial.print(" write_same=");
    Serial.print(writeReadbackPass ? "PASS" : "FAIL");
    Serial.print(" idle_sda=");
    Serial.print(sda);
    Serial.print(" idle_scl=");
    Serial.println(scl);
    printStats("MPL3115A2", stats);
    Serial.print("I2C_TRANSPORT_ERRORS ");
    Serial.println(transportErrors);

    const bool pass = initialReadPass &&
                      whoAmI == kMplExpectedWhoAmI &&
                      writeReadbackPass &&
                      stats.perfect() &&
                      transportErrors == 0UL &&
                      sda == HIGH &&
                      scl == HIGH;
    printPassFail("MPL3115A2", pass);
    return pass;
}

void resetLoRa()
{
    pinMode(BoardPinMap::SparkFunSx1276_1W::resetPin, OUTPUT);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::ssPin, HIGH);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::rxEnablePin, LOW);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::txEnablePin, LOW);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::resetPin, LOW);
    delay(50);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::resetPin, HIGH);
    delay(500);
}

uint8_t readLoRaRegister(uint8_t address, uint32_t spiHz)
{
    SPI1.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::ssPin, LOW);
    delayMicroseconds(20);
    (void)SPI1.transfer(static_cast<uint8_t>(address & 0x7FU));
    const uint8_t value = SPI1.transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::ssPin, HIGH);
    SPI1.endTransaction();
    return value;
}

void writeLoRaRegister(uint8_t address, uint8_t value, uint32_t spiHz)
{
    SPI1.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE0));
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::ssPin, LOW);
    delayMicroseconds(20);
    (void)SPI1.transfer(static_cast<uint8_t>(address | 0x80U));
    (void)SPI1.transfer(value);
    delayMicroseconds(20);
    digitalWrite(BoardPinMap::SparkFunSx1276_1W::ssPin, HIGH);
    SPI1.endTransaction();
}

struct GpioDriveStats
{
    uint16_t low = 0U;
    uint16_t high = 0U;
    uint16_t total = 0U;

    bool perfect() const
    {
        return total > 0U && low == total && high == total;
    }
};

GpioDriveStats stressGpioOutput(uint8_t pin, uint8_t otherPin)
{
    GpioDriveStats stats;
    digitalWrite(otherPin, LOW);
    for (uint16_t sample = 0U; sample < 1000U; ++sample)
    {
        digitalWrite(pin, LOW);
        delayMicroseconds(100);
        stats.low += digitalRead(pin) == LOW ? 1U : 0U;

        digitalWrite(pin, HIGH);
        delayMicroseconds(100);
        stats.high += digitalRead(pin) == HIGH ? 1U : 0U;
        ++stats.total;
    }
    digitalWrite(pin, LOW);
    return stats;
}

bool testLoRaControlPins()
{
    const uint8_t rxe = BoardPinMap::SparkFunSx1276_1W::rxEnablePin;
    const uint8_t txe = BoardPinMap::SparkFunSx1276_1W::txEnablePin;
    const GpioDriveStats rxStats = stressGpioOutput(rxe, txe);
    const GpioDriveStats txStats = stressGpioOutput(txe, rxe);

    Serial.print("LORA_RXE_DRIVE low=");
    Serial.print(rxStats.low);
    Serial.print('/');
    Serial.print(rxStats.total);
    Serial.print(" high=");
    Serial.print(rxStats.high);
    Serial.print('/');
    Serial.println(rxStats.total);
    Serial.print("LORA_TXE_DRIVE low=");
    Serial.print(txStats.low);
    Serial.print('/');
    Serial.print(txStats.total);
    Serial.print(" high=");
    Serial.print(txStats.high);
    Serial.print('/');
    Serial.println(txStats.total);
    return rxStats.perfect() && txStats.perfect();
}

bool testLoRa()
{
    resetLoRa();

    constexpr uint32_t frequencies[] = {
        25000UL,
        50000UL,
        100000UL,
        250000UL,
        500000UL,
        1000000UL,
    };

    bool stressPass = true;
    uint32_t totalExpected = 0UL;
    uint32_t totalSamples = 0UL;
    for (const uint32_t frequency : frequencies)
    {
        ByteStats stats;
        for (uint16_t sample = 0U; sample < kLoRaSamplesPerFrequency; ++sample)
        {
            stats.add(readLoRaRegister(kLoRaVersionRegister, frequency), kLoRaExpectedVersion);
            delay(1);
        }
        Serial.print("LORA_HZ ");
        Serial.print(frequency);
        Serial.print(' ');
        printStats("SX1276", stats);
        stressPass = stressPass && stats.perfect();
        totalExpected += stats.expected;
        totalSamples += stats.total;
    }

    const uint8_t originalFrfMsb = readLoRaRegister(kLoRaFrfMsbRegister, 250000UL);
    writeLoRaRegister(kLoRaFrfMsbRegister, originalFrfMsb, 250000UL);
    const uint8_t frfAfterWrite = readLoRaRegister(kLoRaFrfMsbRegister, 250000UL);
    const bool writeReadbackPass = frfAfterWrite == originalFrfMsb;
    const bool gpioPass = testLoRaControlPins();

    Serial.print("LORA_TOTAL expected=");
    Serial.print(totalExpected);
    Serial.print('/');
    Serial.print(totalSamples);
    Serial.print(" write_same=");
    Serial.print(writeReadbackPass ? "PASS" : "FAIL");
    Serial.print(" rst=");
    Serial.print(digitalRead(BoardPinMap::SparkFunSx1276_1W::resetPin));
    Serial.print(" nss=");
    Serial.print(digitalRead(BoardPinMap::SparkFunSx1276_1W::ssPin));
    Serial.print(" dio0=");
    Serial.println(digitalRead(BoardPinMap::SparkFunSx1276_1W::dio0Pin));

    const bool pass = stressPass && writeReadbackPass && gpioPass;
    printPassFail("SX1276_SIGNAL_BUS", pass);
    return pass;
}

int hexValue(char value)
{
    if (value >= '0' && value <= '9')
    {
        return value - '0';
    }
    if (value >= 'A' && value <= 'F')
    {
        return value - 'A' + 10;
    }
    if (value >= 'a' && value <= 'f')
    {
        return value - 'a' + 10;
    }
    return -1;
}

class NmeaParser
{
public:
    void feed(uint8_t byte)
    {
        const char value = static_cast<char>(byte);
        if (value == '$')
        {
            active_ = true;
            readingChecksum_ = false;
            checksum_ = 0U;
            claimedChecksum_ = 0U;
            checksumDigits_ = 0U;
            return;
        }
        if (!active_)
        {
            return;
        }
        if (value == '\r' || value == '\n')
        {
            if (readingChecksum_ &&
                checksumDigits_ == 2U &&
                checksum_ == claimedChecksum_)
            {
                ++valid_;
            }
            else
            {
                ++invalid_;
            }
            active_ = false;
            return;
        }
        if (readingChecksum_)
        {
            const int nibble = hexValue(value);
            if (nibble < 0 || checksumDigits_ >= 2U)
            {
                active_ = false;
                ++invalid_;
                return;
            }
            claimedChecksum_ = static_cast<uint8_t>((claimedChecksum_ << 4U) |
                                                    static_cast<uint8_t>(nibble));
            ++checksumDigits_;
            return;
        }
        if (value == '*')
        {
            readingChecksum_ = true;
            return;
        }
        checksum_ ^= byte;
    }

    uint32_t valid() const
    {
        return valid_;
    }

    uint32_t invalid() const
    {
        return invalid_;
    }

private:
    bool active_ = false;
    bool readingChecksum_ = false;
    uint8_t checksum_ = 0U;
    uint8_t claimedChecksum_ = 0U;
    uint8_t checksumDigits_ = 0U;
    uint32_t valid_ = 0UL;
    uint32_t invalid_ = 0UL;
};

class UbxParser
{
public:
    void feed(uint8_t byte)
    {
        switch (state_)
        {
        case 0:
            if (byte == 0xB5U)
            {
                state_ = 1;
            }
            break;
        case 1:
            state_ = byte == 0x62U ? 2 : 0;
            break;
        case 2:
            messageClass_ = byte;
            resetChecksum();
            addChecksum(byte);
            state_ = 3;
            break;
        case 3:
            messageId_ = byte;
            addChecksum(byte);
            state_ = 4;
            break;
        case 4:
            payloadLength_ = byte;
            addChecksum(byte);
            state_ = 5;
            break;
        case 5:
            payloadLength_ |= static_cast<uint16_t>(byte) << 8U;
            addChecksum(byte);
            payloadIndex_ = 0U;
            if (payloadLength_ > 1024U)
            {
                state_ = 0;
            }
            else
            {
                state_ = payloadLength_ == 0U ? 7 : 6;
            }
            break;
        case 6:
            addChecksum(byte);
            ++payloadIndex_;
            if (payloadIndex_ >= payloadLength_)
            {
                state_ = 7;
            }
            break;
        case 7:
            receivedChecksumA_ = byte;
            state_ = 8;
            break;
        case 8:
            if (receivedChecksumA_ == checksumA_ && byte == checksumB_)
            {
                ++valid_;
                if (messageClass_ == 0x0AU && messageId_ == 0x04U)
                {
                    ++monVer_;
                }
            }
            state_ = 0;
            break;
        default:
            state_ = 0;
            break;
        }
    }

    uint32_t valid() const
    {
        return valid_;
    }

    uint32_t monVer() const
    {
        return monVer_;
    }

private:
    void resetChecksum()
    {
        checksumA_ = 0U;
        checksumB_ = 0U;
    }

    void addChecksum(uint8_t byte)
    {
        checksumA_ = static_cast<uint8_t>(checksumA_ + byte);
        checksumB_ = static_cast<uint8_t>(checksumB_ + checksumA_);
    }

    uint8_t state_ = 0U;
    uint8_t messageClass_ = 0U;
    uint8_t messageId_ = 0U;
    uint8_t checksumA_ = 0U;
    uint8_t checksumB_ = 0U;
    uint8_t receivedChecksumA_ = 0U;
    uint16_t payloadLength_ = 0U;
    uint16_t payloadIndex_ = 0U;
    uint32_t valid_ = 0UL;
    uint32_t monVer_ = 0UL;
};

void sendMonVerPoll()
{
    static constexpr uint8_t request[] = {
        0xB5U, 0x62U, 0x0AU, 0x04U, 0x00U, 0x00U, 0x0EU, 0x34U,
    };
    BoardPinMap::UbloxM6::serial().write(request, sizeof(request));
    BoardPinMap::UbloxM6::serial().flush();
}

GpsProbeResult probeGpsBaud(uint32_t baud)
{
    auto &gpsSerial = BoardPinMap::UbloxM6::serial();
    gpsSerial.end();
    delay(20);
    gpsSerial.begin(baud);
    delay(250);
    while (gpsSerial.available() > 0)
    {
        (void)gpsSerial.read();
    }

    NmeaParser nmea;
    UbxParser ubx;
    GpsProbeResult result;
    result.baud = baud;

    sendMonVerPoll();
    const uint32_t startMs = millis();
    bool repeatedPoll = false;
    while ((millis() - startMs) < kGpsProbeWindowMs)
    {
        while (gpsSerial.available() > 0)
        {
            const int value = gpsSerial.read();
            if (value >= 0)
            {
                const uint8_t byte = static_cast<uint8_t>(value);
                ++result.bytes;
                nmea.feed(byte);
                ubx.feed(byte);
            }
        }
        if (!repeatedPoll && (millis() - startMs) >= 1500UL)
        {
            sendMonVerPoll();
            repeatedPoll = true;
        }
        delay(1);
    }

    result.validNmea = nmea.valid();
    result.invalidNmea = nmea.invalid();
    result.validUbx = ubx.valid();
    result.monVerResponses = ubx.monVer();
    return result;
}

bool testGps()
{
    GpsProbeResult best;
    for (const uint32_t baud : kGpsBaudCandidates)
    {
        const GpsProbeResult result = probeGpsBaud(baud);
        Serial.print("GPS_BAUD ");
        Serial.print(result.baud);
        Serial.print(" bytes=");
        Serial.print(result.bytes);
        Serial.print(" nmea_ok=");
        Serial.print(result.validNmea);
        Serial.print(" nmea_bad=");
        Serial.print(result.invalidNmea);
        Serial.print(" ubx_ok=");
        Serial.print(result.validUbx);
        Serial.print(" mon_ver=");
        Serial.println(result.monVerResponses);

        if (result.receivePass())
        {
            best = result;
            break;
        }
        if (result.bytes > best.bytes)
        {
            best = result;
        }
    }

    const bool receivePass = best.receivePass();
    const bool transmitPass = best.transmitPass();
    printPassFail("GPS_TX_TO_TEENSY_RX", receivePass);
    printPassFail("TEENSY_TX_TO_GPS_RX", transmitPass);
    return receivePass && transmitPass;
}

void printPinConfiguration()
{
    Serial.print("PINS SPI0 miso=");
    Serial.print(BoardPinMap::SpiBus::misoPin);
    Serial.print(" mosi=");
    Serial.print(BoardPinMap::SpiBus::mosiPin);
    Serial.print(" sck=");
    Serial.print(BoardPinMap::SpiBus::sckPin);
    Serial.print(" h3l_cs=");
    Serial.print(BoardPinMap::H3LIS331DL::csPin);
    Serial.print(" lsm_cs=");
    Serial.println(BoardPinMap::LSM6DSO32::csPin);

    Serial.print("PINS I2C0 sda=");
    Serial.print(BoardPinMap::MPL3115A2::sdaPin);
    Serial.print(" scl=");
    Serial.println(BoardPinMap::MPL3115A2::sclPin);

    Serial.print("PINS SPI1 miso=");
    Serial.print(BoardPinMap::Spi1Bus::misoPin);
    Serial.print(" mosi=");
    Serial.print(BoardPinMap::Spi1Bus::mosiPin);
    Serial.print(" sck=");
    Serial.print(BoardPinMap::Spi1Bus::sckPin);
    Serial.print(" nss=");
    Serial.print(BoardPinMap::SparkFunSx1276_1W::ssPin);
    Serial.print(" rst=");
    Serial.print(BoardPinMap::SparkFunSx1276_1W::resetPin);
    Serial.print(" rxe=");
    Serial.print(BoardPinMap::SparkFunSx1276_1W::rxEnablePin);
    Serial.print(" txe=");
    Serial.print(BoardPinMap::SparkFunSx1276_1W::txEnablePin);
    Serial.print(" dio0=");
    Serial.println(BoardPinMap::SparkFunSx1276_1W::dio0Pin);

    Serial.print("PINS GPS teensy_rx=");
    Serial.print(BoardPinMap::UbloxM6::rxPin);
    Serial.print(" teensy_tx=");
    Serial.println(BoardPinMap::UbloxM6::txPin);
}

void runAllTests()
{
    ++runNumber;
    Serial.println();
    Serial.print("HARDWARE_SIGNAL_TEST_BEGIN run=");
    Serial.println(runNumber);
    printPinConfiguration();

    const bool h3lPass = testSpiSensor("H3LIS331DL",
                                       BoardPinMap::H3LIS331DL::csPin,
                                       kH3lExpectedWhoAmI,
                                       kH3lCtrlReg1);
    const bool lsmPass = testSpiSensor("LSM6DSO32",
                                       BoardPinMap::LSM6DSO32::csPin,
                                       kLsmExpectedWhoAmI,
                                       kLsmCtrl1Xl);
    const bool mplPass = testMpl();
    const bool loraPass = testLoRa();
    const bool gpsPass = testGps();

    Serial.print("HARDWARE_SIGNAL_TEST_SUMMARY run=");
    Serial.print(runNumber);
    Serial.print(" H3L=");
    Serial.print(h3lPass ? "PASS" : "FAIL");
    Serial.print(" LSM=");
    Serial.print(lsmPass ? "PASS" : "FAIL");
    Serial.print(" MPL=");
    Serial.print(mplPass ? "PASS" : "FAIL");
    Serial.print(" LORA=");
    Serial.print(loraPass ? "PASS" : "FAIL");
    Serial.print(" GPS=");
    Serial.print(gpsPass ? "PASS" : "FAIL");
    Serial.print(" OVERALL=");
    Serial.println((h3lPass && lsmPass && mplPass && loraPass && gpsPass) ? "PASS" : "FAIL");
    Serial.println("HARDWARE_SIGNAL_TEST_END");
    Serial.println("Send 'r' to repeat.");
}
} // namespace

void setup()
{
    Serial.begin(kConsoleBaud);
    while (!Serial && millis() < 5000UL)
    {
    }

    holdSafetyCriticalPinsInactive();
    beginBuses();
    delay(100);
    runAllTests();
}

void loop()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());
        if (command == 'r' || command == 'R')
        {
            runAllTests();
        }
    }
    delay(10);
}
