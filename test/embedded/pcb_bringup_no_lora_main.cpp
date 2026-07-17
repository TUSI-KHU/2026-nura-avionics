#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include "board_pinmap.h"
#include "hal/battery_voltage_hal.h"
#include "hal/buzzer_hal.h"
#include "hal/h3lis331dl_hal.h"
#include "hal/lis3mdl_hal.h"
#include "hal/lsm6dso32_hal.h"
#include "hal/mosfet_pyro_hal.h"
#include "hal/mpl3115a2_hal.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kSensorExerciseMs = 5000UL;
constexpr uint32_t kGpsExerciseMs = 8000UL;
constexpr uint32_t kLivePrintMs = 1000UL;
constexpr uint32_t kGpsRawDumpMs = 5000UL;
constexpr uint32_t kGpsBaudProbeMs = 1500UL;
constexpr uint32_t kPyroArmWindowMs = 10000UL;
constexpr uint32_t kPyroPulseMs = 1000UL;
constexpr uint16_t kBuzzerTestHz = 2400U;

struct TestStats
{
    uint8_t total = 0U;
    uint8_t failed = 0U;
};

struct SensorState
{
    bool lowImuInit = false;
    bool highGInit = false;
    bool magInit = false;
    bool mplInit = false;
    bool gpsInit = false;
    bool powerInit = false;
    bool pyroInit = false;
};

LSM6DSO32HAL lowImu;
H3LIS331DLHAL highG;
LIS3MDLHAL mag;
MPL3115A2HAL mpl;
UbloxM6GNSSHAL gps;
BatteryVoltageHAL battery;
MosfetPyroHAL pyro;
BuzzerHAL buzzer(BoardPinMap::Buzzer::pin);

SensorState sensorState;
Lsm6dso32Reading lowImuReading;
H3LIS331DLReading highGReading;
Lis3mdlReading magReading;
Mpl3115a2Reading mplReading;
UbloxM6GnssReading gpsReading;
BatteryVoltageReading batteryReading;

bool pyroArmed = false;
uint32_t pyroArmExpiresMs = 0UL;
uint32_t pyroPulseEndsMs = 0UL;

bool finite3(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z);
}

void printResult(TestStats &stats, const char *name, bool pass)
{
    ++stats.total;
    if (!pass)
    {
        ++stats.failed;
    }

    Serial.print("TEST ");
    Serial.print(name);
    Serial.print(" ");
    Serial.println(pass ? "PASS" : "FAIL");
}

void printHex2(uint8_t value)
{
    if (value < 0x10U)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void printPinMap()
{
    Serial.println("PINMAP");
    Serial.println("  LED1=34 LED2=33 BUZZER=2 POWER_SENSE=22");
    Serial.println("  I2C0 MPL3115A2 SDA=18 SCL=19");
    Serial.println("  I2C1 LIS3MDL SDA=17 SCL=16");
    Serial.println("  SPI0 MISO=12 MOSI=11 SCK=13 H3LIS_CS=0 LSM_CS=10");
    Serial.println("  GPS Serial3 TX=14 RX=15 baud=38400");
    Serial.println("  PYRO1 gpio1=28 gpio2=29 sense=25");
    Serial.println("  PYRO2 gpio1=38 gpio2=35 sense=41");
    Serial.println("  LoRa intentionally excluded in this branch.");
}

uint8_t readSpiRegisterRaw(uint8_t csPin, uint32_t spiFrequency, uint8_t spiMode, uint8_t address)
{
    SPISettings settings(spiFrequency, MSBFIRST, spiMode);
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    SPI.beginTransaction(settings);
    digitalWrite(csPin, LOW);
    delayMicroseconds(20);
    SPI.transfer(address | 0x80U);
    const uint8_t value = SPI.transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();
    return value;
}

bool readI2cRegister(TwoWire &wire, uint8_t address, uint8_t reg, uint8_t &value)
{
    wire.beginTransmission(address);
    wire.write(reg);
    if (wire.endTransmission(false) != 0)
    {
        return false;
    }
    if (wire.requestFrom(static_cast<int>(address), 1) != 1)
    {
        return false;
    }
    value = static_cast<uint8_t>(wire.read());
    return true;
}

void scanI2cBus(TwoWire &wire, const char *name)
{
    Serial.print("I2C_SCAN ");
    Serial.print(name);
    Serial.print(" found=");
    bool any = false;
    for (uint8_t address = 1U; address < 127U; ++address)
    {
        wire.beginTransmission(address);
        if (wire.endTransmission() == 0)
        {
            if (any)
            {
                Serial.print(",");
            }
            Serial.print("0x");
            printHex2(address);
            any = true;
        }
    }
    if (!any)
    {
        Serial.print("none");
    }
    Serial.println();
}

void initializeBoardPins()
{
    pinMode(BoardPinMap::StatusIndicator::led1Pin, OUTPUT);
    pinMode(BoardPinMap::StatusIndicator::led2Pin, OUTPUT);
    digitalWrite(BoardPinMap::StatusIndicator::led1Pin, LOW);
    digitalWrite(BoardPinMap::StatusIndicator::led2Pin, LOW);

    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);

    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();

    TwoWire &i2c0 = BoardPinMap::I2c0Bus::wire();
    i2c0.setSDA(BoardPinMap::I2c0Bus::sdaPin);
    i2c0.setSCL(BoardPinMap::I2c0Bus::sclPin);
    i2c0.begin();
    i2c0.setClock(BoardPinMap::I2c0Bus::clockHz);

    TwoWire &i2c1 = BoardPinMap::I2c1Bus::wire();
    i2c1.setSDA(BoardPinMap::I2c1Bus::sdaPin);
    i2c1.setSCL(BoardPinMap::I2c1Bus::sclPin);
    i2c1.begin();
    i2c1.setClock(BoardPinMap::I2c1Bus::clockHz);
}

bool testTeensyAndIndicators()
{
    for (uint8_t i = 0U; i < 2U; ++i)
    {
        digitalWrite(BoardPinMap::StatusIndicator::led1Pin, HIGH);
        digitalWrite(BoardPinMap::StatusIndicator::led2Pin, LOW);
        delay(120);
        digitalWrite(BoardPinMap::StatusIndicator::led1Pin, LOW);
        digitalWrite(BoardPinMap::StatusIndicator::led2Pin, HIGH);
        delay(120);
    }
    digitalWrite(BoardPinMap::StatusIndicator::led2Pin, LOW);

    if (buzzer.begin())
    {
        buzzer.playTone(kBuzzerTestHz);
        delay(150);
        buzzer.silence();
    }

    return true;
}

bool testLowImu()
{
    const uint8_t whoAmI = readSpiRegisterRaw(BoardPinMap::LSM6DSO32::csPin,
                                              NuraConstants::LSM6DSO32::kProbeSpiHz,
                                              SPI_MODE0,
                                              NuraConstants::LSM6DSO32::kWhoAmIRegister);
    Serial.print("LSM6DSO32_WHOAMI=0x");
    printHex2(whoAmI);
    Serial.println();

    sensorState.lowImuInit = lowImu.begin(BoardPinMap::LSM6DSO32::csPin, SPI);
    if (!sensorState.lowImuInit)
    {
        return false;
    }

    for (uint8_t i = 0U; i < 10U; ++i)
    {
        if (lowImu.read(lowImuReading, millis()) &&
            finite3(lowImuReading.accelXMps2, lowImuReading.accelYMps2, lowImuReading.accelZMps2) &&
            finite3(lowImuReading.gyroXDps, lowImuReading.gyroYDps, lowImuReading.gyroZDps))
        {
            return true;
        }
        delay(20);
    }
    return false;
}

bool testHighG()
{
    Serial.print("H3LIS331DL_WHOAMI_RAW");
    for (uint8_t mode = 0U; mode < 4U; ++mode)
    {
        const uint8_t whoAmI = readSpiRegisterRaw(BoardPinMap::H3LIS331DL::csPin,
                                                  1000000UL,
                                                  mode,
                                                  0x0FU);
        Serial.print(" m");
        Serial.print(mode);
        Serial.print("=0x");
        printHex2(whoAmI);
    }
    Serial.println();

    sensorState.highGInit = highG.begin(BoardPinMap::H3LIS331DL::csPin,
                                        SPI,
                                        H3LIS331DLRange::RANGE_200G);
    if (!sensorState.highGInit)
    {
        return false;
    }

    for (uint8_t i = 0U; i < 10U; ++i)
    {
        if (highG.read(highGReading, millis()) &&
            highGReading.whoAmI == NuraConstants::H3LIS331DL::kExpectedWhoAmI &&
            finite3(highGReading.accelXG, highGReading.accelYG, highGReading.accelZG))
        {
            return true;
        }
        delay(20);
    }
    return false;
}

bool testMag()
{
    uint8_t whoAmI = 0U;
    const bool whoOk = readI2cRegister(BoardPinMap::LIS3MDL::wire(),
                                       BoardPinMap::LIS3MDL::i2cAddress,
                                       0x0FU,
                                       whoAmI);
    Serial.print("LIS3MDL_WHOAMI=");
    Serial.print(whoOk ? "0x" : "read_fail");
    if (whoOk)
    {
        printHex2(whoAmI);
    }
    Serial.println();

    sensorState.magInit = mag.begin(BoardPinMap::LIS3MDL::i2cAddress,
                                    BoardPinMap::LIS3MDL::wire());
    if (!sensorState.magInit)
    {
        return false;
    }

    for (uint8_t i = 0U; i < 10U; ++i)
    {
        if (mag.read(magReading, millis()) &&
            finite3(magReading.magXuT, magReading.magYuT, magReading.magZuT))
        {
            return true;
        }
        delay(20);
    }
    return false;
}

bool testMpl()
{
    uint8_t whoAmI = 0U;
    const bool whoOk = readI2cRegister(BoardPinMap::MPL3115A2::wire(),
                                       BoardPinMap::MPL3115A2::i2cAddress,
                                       0x0CU,
                                       whoAmI);
    Serial.print("MPL3115A2_WHOAMI=");
    Serial.print(whoOk ? "0x" : "read_fail");
    if (whoOk)
    {
        printHex2(whoAmI);
    }
    Serial.println();

    sensorState.mplInit = mpl.begin(BoardPinMap::MPL3115A2::wire(), 700U);
    if (!sensorState.mplInit)
    {
        return false;
    }

    for (uint8_t i = 0U; i < 5U; ++i)
    {
        if (mpl.read(mplReading, millis()) &&
            isfinite(mplReading.pressurePa) &&
            mplReading.pressurePa >= NuraConstants::MPL3115A2::kMinDatasheetPressurePa &&
            mplReading.pressurePa <= NuraConstants::MPL3115A2::kMaxDatasheetPressurePa)
        {
            return true;
        }
        delay(100);
    }
    return false;
}

bool testPowerSense()
{
    sensorState.powerInit = battery.begin(BoardPinMap::PowerSense::voltagePin,
                                          NuraConstants::Sensors::kPowerSenseAdcReferenceMv,
                                          NuraConstants::Sensors::kPowerSenseAdcResolutionBits,
                                          NuraConstants::Sensors::kPowerSenseDividerRatioNumerator,
                                          NuraConstants::Sensors::kPowerSenseDividerRatioDenominator,
                                          NuraConstants::Sensors::kPowerSenseMinValidBatteryMv,
                                          NuraConstants::Sensors::kPowerSenseMaxValidBatteryMv);
    if (!sensorState.powerInit)
    {
        return false;
    }
    return battery.read(batteryReading, millis()) && batteryReading.valid;
}

bool testGpsElectrical()
{
    auto &serial = BoardPinMap::UbloxM6::serial();
    serial.setRX(BoardPinMap::UbloxM6::rxPin);
    serial.setTX(BoardPinMap::UbloxM6::txPin);
    sensorState.gpsInit = gps.begin(serial,
                                    BoardPinMap::UbloxM6::baud,
                                    NuraConstants::Sensors::kGnssMaxFixAgeMs);
    if (!sensorState.gpsInit)
    {
        return false;
    }

    const uint32_t startMs = millis();
    while ((millis() - startMs) < kGpsExerciseMs)
    {
        (void)gps.poll(gpsReading, millis(), 128U);
        delay(20);
    }

    Serial.print("GPS chars=");
    Serial.print(gpsReading.charsProcessed);
    Serial.print(" checksum_ok=");
    Serial.print(gpsReading.passedChecksum);
    Serial.print(" checksum_bad=");
    Serial.print(gpsReading.failedChecksum);
    Serial.print(" fix=");
    Serial.print(gpsReading.hasFix ? "yes" : "no");
    Serial.print(" sats=");
    Serial.println(gpsReading.satellites);

    return gpsReading.charsProcessed > 0UL && gpsReading.passedChecksum > 0UL;
}

bool testPyroBenchPath()
{
    sensorState.pyroInit = pyro.begin();
    pyro.allOff();
    pinMode(BoardPinMap::Pyro1::sensePin, INPUT);
    pinMode(BoardPinMap::Pyro2::sensePin, INPUT);

    Serial.print("PYRO_SENSE p1=");
    Serial.print(digitalRead(BoardPinMap::Pyro1::sensePin));
    Serial.print(" p2=");
    Serial.println(digitalRead(BoardPinMap::Pyro2::sensePin));
    Serial.println("PYRO manual only: type ARM, then P1 or P2 within 10 seconds.");
    return sensorState.pyroInit;
}

void updateLiveReadings()
{
    const uint32_t nowMs = millis();
    if (sensorState.lowImuInit)
    {
        (void)lowImu.read(lowImuReading, nowMs);
    }
    if (sensorState.highGInit)
    {
        (void)highG.read(highGReading, nowMs);
    }
    if (sensorState.magInit)
    {
        (void)mag.read(magReading, nowMs);
    }
    if (sensorState.mplInit)
    {
        (void)mpl.read(mplReading, nowMs);
    }
    if (sensorState.gpsInit)
    {
        (void)gps.poll(gpsReading, nowMs, 128U);
    }
    if (sensorState.powerInit)
    {
        (void)battery.read(batteryReading, nowMs);
    }
}

void printLiveSnapshot()
{
    Serial.print("LIVE low_acc=");
    Serial.print(lowImuReading.accelXMps2, 2);
    Serial.print(",");
    Serial.print(lowImuReading.accelYMps2, 2);
    Serial.print(",");
    Serial.print(lowImuReading.accelZMps2, 2);
    Serial.print(" high_g=");
    Serial.print(highGReading.accelXG, 2);
    Serial.print(",");
    Serial.print(highGReading.accelYG, 2);
    Serial.print(",");
    Serial.print(highGReading.accelZG, 2);
    Serial.print(" mag_uT=");
    Serial.print(magReading.magXuT, 1);
    Serial.print(",");
    Serial.print(magReading.magYuT, 1);
    Serial.print(",");
    Serial.print(magReading.magZuT, 1);
    Serial.print(" mpl_pa=");
    Serial.print(mplReading.pressurePa, 1);
    Serial.print(" batt_mv=");
    Serial.print(batteryReading.batteryMv);
    Serial.print(" gps_chars=");
    Serial.print(gpsReading.charsProcessed);
    Serial.print(" gps_fix=");
    Serial.print(gpsReading.hasFix ? "yes" : "no");
    Serial.print(" pyro_sense=");
    Serial.print(digitalRead(BoardPinMap::Pyro1::sensePin));
    Serial.print(",");
    Serial.println(digitalRead(BoardPinMap::Pyro2::sensePin));
}

void allPyroOff()
{
    pyro.allOff();
    pyroPulseEndsMs = 0UL;
}

void startPyroPulse(bool pyro1, bool pyro2)
{
    const uint32_t nowMs = millis();
    if (!sensorState.pyroInit)
    {
        Serial.println("PYRO_REJECT not_initialized");
        return;
    }
    if (!pyroArmed || nowMs >= pyroArmExpiresMs)
    {
        pyroArmed = false;
        Serial.println("PYRO_REJECT not_armed");
        return;
    }
    if (pyroPulseEndsMs != 0UL)
    {
        Serial.println("PYRO_REJECT pulse_active");
        return;
    }

    pyro.setDrogue(pyro1);
    pyro.setMain(pyro2);
    pyroPulseEndsMs = nowMs + kPyroPulseMs;
    pyroArmed = false;
    Serial.print("PYRO_PULSE_START channel=");
    Serial.println(pyro1 ? "P1" : "P2");
}

void servicePyro()
{
    const uint32_t nowMs = millis();
    if (pyroArmed && nowMs >= pyroArmExpiresMs)
    {
        pyroArmed = false;
        Serial.println("PYRO_ARM_EXPIRED");
    }
    if (pyroPulseEndsMs != 0UL && nowMs >= pyroPulseEndsMs)
    {
        allPyroOff();
        Serial.println("PYRO_PULSE_DONE all_off");
    }
}

void printHelp()
{
    Serial.println("COMMANDS: HELP, STATUS, H3_RAW, SPI_SCAN, GPS_RAW, GPS_BAUD_SCAN, ARM, DISARM, P1, P2, ALL_OFF");
    Serial.println("  ARM expires after 10 seconds. P1/P2 pulse for 1000 ms.");
}

void printH3Raw()
{
    Serial.print("H3LIS331DL_RAW cs=");
    Serial.print(BoardPinMap::H3LIS331DL::csPin);
    for (uint8_t mode = 0U; mode < 4U; ++mode)
    {
        const uint8_t whoAmI = readSpiRegisterRaw(BoardPinMap::H3LIS331DL::csPin,
                                                  1000000UL,
                                                  mode,
                                                  0x0FU);
        const uint8_t ctrl1 = readSpiRegisterRaw(BoardPinMap::H3LIS331DL::csPin,
                                                 1000000UL,
                                                 mode,
                                                 0x20U);
        Serial.print(" m");
        Serial.print(mode);
        Serial.print("_who=0x");
        printHex2(whoAmI);
        Serial.print("_ctrl1=0x");
        printHex2(ctrl1);
    }
    Serial.println();
}

void deselectSpiScanPins(const uint8_t *pins, uint8_t count)
{
    for (uint8_t i = 0U; i < count; ++i)
    {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], HIGH);
    }
}

void printSpiScan()
{
    static constexpr uint8_t kCandidateCsPins[] = {
        BoardPinMap::H3LIS331DL::csPin,
        BoardPinMap::LSM6DSO32::csPin,
        9U,
        24U,
        30U,
        31U,
        32U,
    };

    deselectSpiScanPins(kCandidateCsPins, sizeof(kCandidateCsPins));

    Serial.println("SPI_SCAN_BEGIN bus=SPI0 miso=12 mosi=11 sck=13 expected_lsm=0x6C expected_h3=0x32");
    for (uint8_t i = 0U; i < sizeof(kCandidateCsPins); ++i)
    {
        const uint8_t csPin = kCandidateCsPins[i];
        Serial.print("SPI_SCAN cs=");
        Serial.print(csPin);
        for (uint8_t mode = 0U; mode < 4U; ++mode)
        {
            deselectSpiScanPins(kCandidateCsPins, sizeof(kCandidateCsPins));
            const uint8_t who = readSpiRegisterRaw(csPin, 1000000UL, mode, 0x0FU);
            const uint8_t ctrl1 = readSpiRegisterRaw(csPin, 1000000UL, mode, 0x20U);
            Serial.print(" m");
            Serial.print(mode);
            Serial.print("_who=0x");
            printHex2(who);
            Serial.print("_ctrl1=0x");
            printHex2(ctrl1);
        }
        Serial.println();
    }
    deselectSpiScanPins(kCandidateCsPins, sizeof(kCandidateCsPins));
    Serial.println("SPI_SCAN_END");
}

void dumpGpsRaw()
{
    auto &serial = BoardPinMap::UbloxM6::serial();
    Serial.println("GPS_RAW_BEGIN");
    const uint32_t startMs = millis();
    while ((millis() - startMs) < kGpsRawDumpMs)
    {
        while (serial.available() > 0)
        {
            const int value = serial.read();
            if (value >= 0)
            {
                Serial.write(static_cast<uint8_t>(value));
            }
        }
        servicePyro();
        delay(1);
    }
    Serial.println();
    Serial.println("GPS_RAW_END");
}

void scanGpsBaud()
{
    auto &serial = BoardPinMap::UbloxM6::serial();
    const uint32_t baudRates[] = {4800UL, 9600UL, 38400UL, 57600UL, 115200UL};

    Serial.println("GPS_BAUD_SCAN_BEGIN");
    for (uint8_t i = 0U; i < sizeof(baudRates) / sizeof(baudRates[0]); ++i)
    {
        serial.end();
        delay(50);
        serial.setRX(BoardPinMap::UbloxM6::rxPin);
        serial.setTX(BoardPinMap::UbloxM6::txPin);
        serial.begin(baudRates[i]);
        delay(100);

        uint16_t total = 0U;
        uint16_t printable = 0U;
        uint16_t dollar = 0U;
        uint16_t newline = 0U;
        char sample[49] = {};
        uint8_t sampleLen = 0U;
        const uint32_t startMs = millis();

        while ((millis() - startMs) < kGpsBaudProbeMs)
        {
            while (serial.available() > 0)
            {
                const int value = serial.read();
                if (value < 0)
                {
                    continue;
                }
                const char c = static_cast<char>(value);
                ++total;
                if ((c >= 0x20 && c <= 0x7E) || c == '\r' || c == '\n')
                {
                    ++printable;
                    if (sampleLen < sizeof(sample) - 1U)
                    {
                        sample[sampleLen++] = (c == '\r' || c == '\n') ? ' ' : c;
                    }
                }
                if (c == '$')
                {
                    ++dollar;
                }
                if (c == '\n')
                {
                    ++newline;
                }
            }
            servicePyro();
            delay(1);
        }

        Serial.print("GPS_BAUD baud=");
        Serial.print(baudRates[i]);
        Serial.print(" total=");
        Serial.print(total);
        Serial.print(" printable=");
        Serial.print(printable);
        Serial.print(" dollar=");
        Serial.print(dollar);
        Serial.print(" newline=");
        Serial.print(newline);
        Serial.print(" sample=\"");
        Serial.print(sample);
        Serial.println("\"");
    }

    serial.end();
    delay(50);
    serial.setRX(BoardPinMap::UbloxM6::rxPin);
    serial.setTX(BoardPinMap::UbloxM6::txPin);
    serial.begin(BoardPinMap::UbloxM6::baud);
    Serial.println("GPS_BAUD_SCAN_END");
}

void handleCommand(char *command)
{
    for (char *p = command; *p != '\0'; ++p)
    {
        if (*p >= 'a' && *p <= 'z')
        {
            *p = static_cast<char>(*p - 'a' + 'A');
        }
    }

    if (strcmp(command, "HELP") == 0)
    {
        printHelp();
    }
    else if (strcmp(command, "STATUS") == 0)
    {
        printLiveSnapshot();
    }
    else if (strcmp(command, "H3_RAW") == 0)
    {
        printH3Raw();
    }
    else if (strcmp(command, "SPI_SCAN") == 0)
    {
        printSpiScan();
    }
    else if (strcmp(command, "GPS_RAW") == 0)
    {
        dumpGpsRaw();
    }
    else if (strcmp(command, "GPS_BAUD_SCAN") == 0)
    {
        scanGpsBaud();
    }
    else if (strcmp(command, "ARM") == 0)
    {
        pyroArmed = true;
        pyroArmExpiresMs = millis() + kPyroArmWindowMs;
        Serial.println("PYRO_ARMED");
    }
    else if (strcmp(command, "DISARM") == 0)
    {
        pyroArmed = false;
        allPyroOff();
        Serial.println("PYRO_DISARMED all_off");
    }
    else if (strcmp(command, "P1") == 0)
    {
        startPyroPulse(true, false);
    }
    else if (strcmp(command, "P2") == 0)
    {
        startPyroPulse(false, true);
    }
    else if (strcmp(command, "ALL_OFF") == 0)
    {
        pyroArmed = false;
        allPyroOff();
        Serial.println("PYRO_ALL_OFF");
    }
    else if (command[0] != '\0')
    {
        Serial.print("UNKNOWN_COMMAND ");
        Serial.println(command);
        printHelp();
    }
}

void pollSerialCommands()
{
    static char command[32];
    static uint8_t length = 0U;

    while (Serial.available() > 0)
    {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            command[length] = '\0';
            handleCommand(command);
            length = 0U;
            continue;
        }
        if (length < sizeof(command) - 1U)
        {
            command[length++] = c;
        }
    }
}

void runBringupTests()
{
    TestStats stats;
    Serial.println("NURA PCB BRINGUP NO-LORA TEST");
    printPinMap();
    initializeBoardPins();
    delay(250);

    printResult(stats, "teensy_led_buzzer", testTeensyAndIndicators());
    scanI2cBus(BoardPinMap::I2c0Bus::wire(), "I2C0");
    scanI2cBus(BoardPinMap::I2c1Bus::wire(), "I2C1");

    printResult(stats, "mpl3115a2_i2c0", testMpl());
    printResult(stats, "lis3mdl_i2c1", testMag());
    printResult(stats, "lsm6dso32_spi", testLowImu());
    printResult(stats, "h3lis331dl_spi", testHighG());
    printResult(stats, "power_sense", testPowerSense());
    printResult(stats, "gps_nmea_serial3", testGpsElectrical());
    printResult(stats, "pyro_manual_path_ready", testPyroBenchPath());

    const uint32_t startMs = millis();
    while ((millis() - startMs) < kSensorExerciseMs)
    {
        updateLiveReadings();
        delay(20);
    }
    printLiveSnapshot();

    Serial.print("SUMMARY ");
    Serial.print(stats.failed == 0U ? "PASS" : "FAIL");
    Serial.print(" total=");
    Serial.print(stats.total);
    Serial.print(" failed=");
    Serial.println(stats.failed);
    printHelp();
}
} // namespace

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    while (!Serial && millis() < 4000UL)
    {
    }
    runBringupTests();
}

void loop()
{
    static uint32_t lastPrintMs = 0UL;
    updateLiveReadings();
    pollSerialCommands();
    servicePyro();

    const uint32_t nowMs = millis();
    if ((nowMs - lastPrintMs) >= kLivePrintMs)
    {
        printLiveSnapshot();
        lastPrintMs = nowMs;
    }
    delay(10);
}
