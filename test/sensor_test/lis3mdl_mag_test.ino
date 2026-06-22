#include <Arduino.h>
#include <Wire.h>
#include "board_pinmap.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN BoardPinMap::LIS3MDL::sdaPin
#define I2C_SCL_PIN BoardPinMap::LIS3MDL::sclPin
#define I2C1_SDA_PIN BoardPinMap::LIS3MDL::sdaPin
#define I2C1_SCL_PIN BoardPinMap::LIS3MDL::sclPin
#define LIS3MDL_I2C_ADDR BoardPinMap::LIS3MDL::i2cAddress
#define LIS3MDL_SAMPLE_COUNT 80
#define LIS3MDL_SAMPLE_DELAY_MS 20
#define LIS3MDL_WHOAMI_REG 0x0F
#define LIS3MDL_WHOAMI_VALUE 0x3D
#define BITBANG_I2C_DELAY_US 50
// ================================================================

Adafruit_LIS3MDL mag;
static TwoWire *lisWire = &Wire1;
static const char *detectedBusName = "Wire1 17/16";
static uint8_t detectedLisAddress = 0U;
static bool lisReady = false;

struct I2cPinPair
{
    const char *name;
    uint8_t sda;
    uint8_t scl;
};

static void printHexByte(uint8_t value)
{
    Serial.print("0x");
    if (value < 16U)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

static bool readI2cRegister(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t &value)
{
    bus.beginTransmission(address);
    bus.write(reg);
    if (bus.endTransmission(false) != 0)
    {
        return false;
    }

    if (bus.requestFrom(address, static_cast<uint8_t>(1U)) != 1)
    {
        return false;
    }

    value = bus.read();
    return true;
}

static void printLineLevels(const I2cPinPair &pair)
{
    pinMode(pair.sda, INPUT);
    pinMode(pair.scl, INPUT);
    delayMicroseconds(20);
    Serial.print("LINES raw ");
    Serial.print(pair.name);
    Serial.print(" SDA=");
    Serial.print(digitalRead(pair.sda));
    Serial.print(" SCL=");
    Serial.println(digitalRead(pair.scl));

    pinMode(pair.sda, INPUT_PULLUP);
    pinMode(pair.scl, INPUT_PULLUP);
    delayMicroseconds(100);
    Serial.print("LINES pullup ");
    Serial.print(pair.name);
    Serial.print(" SDA=");
    Serial.print(digitalRead(pair.sda));
    Serial.print(" SCL=");
    Serial.println(digitalRead(pair.scl));
}

static void bitbangRelease(uint8_t pin)
{
    pinMode(pin, INPUT_PULLUP);
}

static void bitbangLow(uint8_t pin)
{
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
}

static void bitbangDelay()
{
    delayMicroseconds(BITBANG_I2C_DELAY_US);
}

static bool bitbangWaitHigh(uint8_t pin)
{
    bitbangRelease(pin);
    for (uint8_t i = 0U; i < 100U; ++i)
    {
        if (digitalRead(pin) == HIGH)
        {
            return true;
        }
        delayMicroseconds(10);
    }
    return false;
}

static bool bitbangStart(const I2cPinPair &pair)
{
    bitbangRelease(pair.sda);
    if (!bitbangWaitHigh(pair.scl))
    {
        return false;
    }
    bitbangDelay();
    if (digitalRead(pair.sda) == LOW)
    {
        return false;
    }
    bitbangLow(pair.sda);
    bitbangDelay();
    bitbangLow(pair.scl);
    return true;
}

static void bitbangStop(const I2cPinPair &pair)
{
    bitbangLow(pair.sda);
    bitbangDelay();
    bitbangRelease(pair.scl);
    bitbangDelay();
    bitbangRelease(pair.sda);
    bitbangDelay();
}

static bool bitbangWriteByte(const I2cPinPair &pair, uint8_t value)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        if ((value & mask) != 0U)
        {
            bitbangRelease(pair.sda);
        }
        else
        {
            bitbangLow(pair.sda);
        }
        bitbangDelay();
        if (!bitbangWaitHigh(pair.scl))
        {
            return false;
        }
        bitbangDelay();
        bitbangLow(pair.scl);
    }

    bitbangRelease(pair.sda);
    bitbangDelay();
    if (!bitbangWaitHigh(pair.scl))
    {
        return false;
    }
    const bool ack = digitalRead(pair.sda) == LOW;
    bitbangDelay();
    bitbangLow(pair.scl);
    return ack;
}

static uint8_t bitbangReadByte(const I2cPinPair &pair, bool ack)
{
    uint8_t value = 0U;
    bitbangRelease(pair.sda);
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        value <<= 1U;
        bitbangDelay();
        if (bitbangWaitHigh(pair.scl) && digitalRead(pair.sda) == HIGH)
        {
            value |= 1U;
        }
        bitbangDelay();
        bitbangLow(pair.scl);
    }

    if (ack)
    {
        bitbangLow(pair.sda);
    }
    else
    {
        bitbangRelease(pair.sda);
    }
    bitbangDelay();
    bitbangWaitHigh(pair.scl);
    bitbangDelay();
    bitbangLow(pair.scl);
    bitbangRelease(pair.sda);
    return value;
}

static bool bitbangReadI2cRegister(const I2cPinPair &pair, uint8_t address, uint8_t reg, uint8_t &value)
{
    if (!bitbangStart(pair))
    {
        bitbangStop(pair);
        return false;
    }
    if (!bitbangWriteByte(pair, static_cast<uint8_t>(address << 1U)) ||
        !bitbangWriteByte(pair, reg))
    {
        bitbangStop(pair);
        return false;
    }
    if (!bitbangStart(pair))
    {
        bitbangStop(pair);
        return false;
    }
    if (!bitbangWriteByte(pair, static_cast<uint8_t>((address << 1U) | 1U)))
    {
        bitbangStop(pair);
        return false;
    }
    value = bitbangReadByte(pair, false);
    bitbangStop(pair);
    return true;
}

static bool bitbangDetectLis3mdl()
{
    const I2cPinPair pairs[] = {
        {"Wire1 17/16 normal", I2C1_SDA_PIN, I2C1_SCL_PIN},
        {"Wire1 17/16 swapped", I2C1_SCL_PIN, I2C1_SDA_PIN},
    };
    const uint8_t addresses[] = {0x1CU, 0x1EU};

    Serial.println("BITBANG_SCAN_BEGIN");
    for (const I2cPinPair &pair : pairs)
    {
        printLineLevels(pair);
        for (uint8_t address : addresses)
        {
            uint8_t whoami = 0U;
            Serial.print("BITBANG_CHECK ");
            Serial.print(pair.name);
            Serial.print(" addr=");
            printHexByte(address);
            if (!bitbangReadI2cRegister(pair, address, LIS3MDL_WHOAMI_REG, whoami))
            {
                Serial.println(" no_response");
                continue;
            }

            Serial.print(" value=");
            printHexByte(whoami);
            Serial.println();
            if (whoami == LIS3MDL_WHOAMI_VALUE)
            {
                Serial.print("PASS: LIS3MDL CONNECTION COMPLETE bitbang=");
                Serial.print(pair.name);
                Serial.print(" address=");
                printHexByte(address);
                Serial.println();
                return true;
            }
        }
    }
    Serial.println("BITBANG_SCAN_END");
    return false;
}

static void scanI2c(TwoWire &bus, const char *name, uint8_t sdaPin, uint8_t sclPin)
{
    Serial.print("I2C_SCAN_BEGIN bus=");
    Serial.print(name);
    Serial.print(" SDA=");
    Serial.print(sdaPin);
    Serial.print(" SCL=");
    Serial.println(sclPin);
    for (uint8_t address = 1U; address < 127U; ++address)
    {
        bus.beginTransmission(address);
        if (bus.endTransmission() == 0)
        {
            Serial.print("I2C_FOUND ");
            printHexByte(address);
            Serial.println();
        }
    }
    Serial.print("I2C_SCAN_END bus=");
    Serial.println(name);
}

static bool detectLis3mdlOnBus(TwoWire &bus, const char *name)
{
    const uint8_t candidates[] = {LIS3MDL_I2C_ADDR, 0x1CU, 0x1EU};
    for (uint8_t i = 0U; i < sizeof(candidates); ++i)
    {
        const uint8_t address = candidates[i];
        bool alreadyTried = false;
        for (uint8_t j = 0U; j < i; ++j)
        {
            if (candidates[j] == address)
            {
                alreadyTried = true;
            }
        }
        if (alreadyTried)
        {
            continue;
        }

        uint8_t whoami = 0U;
        Serial.print("CHECK: LIS3MDL WHOAMI bus=");
        Serial.print(name);
        Serial.print(" addr=");
        printHexByte(address);
        if (!readI2cRegister(bus, address, LIS3MDL_WHOAMI_REG, whoami))
        {
            Serial.println(" no_response");
            continue;
        }

        Serial.print(" value=");
        printHexByte(whoami);
        Serial.println();
        if (whoami == LIS3MDL_WHOAMI_VALUE)
        {
            lisWire = &bus;
            detectedBusName = name;
            detectedLisAddress = address;
            return true;
        }
    }

    return false;
}

static void beginI2cBuses()
{
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C_SCL_PIN, INPUT_PULLUP);
    pinMode(I2C1_SDA_PIN, INPUT_PULLUP);
    pinMode(I2C1_SCL_PIN, INPUT_PULLUP);
    delay(10);

    Wire1.setSDA(I2C1_SDA_PIN);
    Wire1.setSCL(I2C1_SCL_PIN);
    Wire1.begin();
    Wire1.setClock(10000UL);
}

static void scanAllI2c()
{
    scanI2c(Wire1, "Wire1 17/16", I2C1_SDA_PIN, I2C1_SCL_PIN);
}

static bool detectLis3mdl()
{
    return detectLis3mdlOnBus(Wire1, "Wire1 17/16");
}

static bool validMag(const sensors_event_t &event)
{
    return isfinite(event.magnetic.x) &&
           isfinite(event.magnetic.y) &&
           isfinite(event.magnetic.z);
}

static void printSample(uint16_t index, const sensors_event_t &event)
{
    Serial.print("sample=");
    Serial.print(index);
    Serial.print(" mag_uT=");
    Serial.print(event.magnetic.x, 4);
    Serial.print(",");
    Serial.print(event.magnetic.y, 4);
    Serial.print(",");
    Serial.print(event.magnetic.z, 4);
    Serial.print(" raw=");
    Serial.print(mag.x);
    Serial.print(",");
    Serial.print(mag.y);
    Serial.print(",");
    Serial.println(mag.z);
}

static bool runDefectTest()
{
    bool anyInvalid = false;
    bool anyNonZeroRaw = false;
    float minMag = 1000000.0f;
    float maxMag = 0.0f;

    for (uint16_t i = 0; i < LIS3MDL_SAMPLE_COUNT; ++i)
    {
        sensors_event_t event;
        if (!mag.getEvent(&event) || !validMag(event))
        {
            anyInvalid = true;
            continue;
        }

        if (mag.x != 0 || mag.y != 0 || mag.z != 0)
        {
            anyNonZeroRaw = true;
        }

        const float field = sqrtf(event.magnetic.x * event.magnetic.x +
                                  event.magnetic.y * event.magnetic.y +
                                  event.magnetic.z * event.magnetic.z);
        minMag = min(minMag, field);
        maxMag = max(maxMag, field);

        if ((i % 10U) == 0U)
        {
            printSample(i, event);
        }

        delay(LIS3MDL_SAMPLE_DELAY_MS);
    }

    Serial.print("field_mag_range_uT=");
    Serial.print(minMag, 4);
    Serial.print("..");
    Serial.println(maxMag, 4);

    if (anyInvalid)
    {
        Serial.println("FAIL: invalid or non-finite sample detected");
        return false;
    }

    if (!anyNonZeroRaw)
    {
        Serial.println("FAIL: all raw axes stayed zero");
        return false;
    }

    if (minMag < 5.0f || maxMag > 200.0f)
    {
        Serial.println("WARN: magnetic field magnitude is unusual; check nearby magnets/current loops");
    }

    Serial.println("PASS: LIS3MDL responded and produced usable magnetic samples");
    return true;
}

static bool attemptLis3mdlConnection()
{
    scanAllI2c();

    if (!detectLis3mdl())
    {
        if (bitbangDetectLis3mdl())
        {
            digitalWrite(LED_BUILTIN, HIGH);
            lisReady = true;
            return true;
        }
        Serial.println("FAIL: LIS3MDL WHOAMI not found at 0x1C or 0x1E on hardware or bitbang I2C");
        return false;
    }

    if (!mag.begin_I2C(detectedLisAddress, lisWire))
    {
        Serial.println("FAIL: LIS3MDL driver init failed after WHOAMI response");
        return false;
    }

    mag.setRange(LIS3MDL_RANGE_16_GAUSS);
    mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    Serial.print("PASS: LIS3MDL CONNECTION COMPLETE bus=");
    Serial.print(detectedBusName);
    Serial.print(" address=");
    printHexByte(detectedLisAddress);
    Serial.println();
    digitalWrite(LED_BUILTIN, HIGH);
    lisReady = true;
    runDefectTest();
    return true;
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("LIS3MDL magnetometer defect test");

    beginI2cBuses();
    attemptLis3mdlConnection();
}

void loop()
{
    if (!lisReady)
    {
        Serial.println("RETRY: LIS3MDL not connected yet");
        attemptLis3mdlConnection();
        delay(3000);
        return;
    }

    sensors_event_t event;
    if (mag.getEvent(&event))
    {
        printSample(0, event);
    }
    delay(1000);
}
