#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "board_pinmap.h"

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <math.h>

#define SERIAL_BAUD 115200
#define GPS_TEST_WINDOW_MS 8000UL
#define LORA_SPI_FREQUENCY_HZ 125000UL
#define LORA_REG_VERSION 0x42
#define LORA_EXPECTED_VERSION 0x12
#define LIS3MDL_WHOAMI_REG 0x0F
#define LIS3MDL_WHOAMI_VALUE 0x3D
#define CURRENT_I2C_BUS Wire1
#define CURRENT_I2C_BUS_NAME "Wire1"

Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DSO32 lsm6dso32;
TinyGPSPlus gps;

static uint32_t runCounter = 0UL;

static void printHexByte(uint8_t value)
{
    Serial.print("0x");
    if (value < 16U)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

static void printBoolResult(const char *name, bool ok)
{
    Serial.print(ok ? "PASS: " : "FAIL: ");
    Serial.println(name);
}

static bool finite3(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z);
}

static void deselectSpiDevices()
{
    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::ssPin, OUTPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
}

static void printI2cLineLevels()
{
    pinMode(BoardPinMap::LIS3MDL::sdaPin, INPUT_PULLUP);
    pinMode(BoardPinMap::LIS3MDL::sclPin, INPUT_PULLUP);
    delayMicroseconds(100);
    Serial.print("I2C_LINES sda=");
    Serial.print(BoardPinMap::LIS3MDL::sdaPin);
    Serial.print(":");
    Serial.print(digitalRead(BoardPinMap::LIS3MDL::sdaPin));
    Serial.print(" scl=");
    Serial.print(BoardPinMap::LIS3MDL::sclPin);
    Serial.print(":");
    Serial.println(digitalRead(BoardPinMap::LIS3MDL::sclPin));
}

static void beginBuses()
{
    CURRENT_I2C_BUS.setSDA(BoardPinMap::LIS3MDL::sdaPin);
    CURRENT_I2C_BUS.setSCL(BoardPinMap::LIS3MDL::sclPin);
    CURRENT_I2C_BUS.begin();
    CURRENT_I2C_BUS.setClock(100000UL);

    deselectSpiDevices();
    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();

    BoardPinMap::UbloxM6::serial().setRX(BoardPinMap::UbloxM6::rxPin);
    BoardPinMap::UbloxM6::serial().setTX(BoardPinMap::UbloxM6::txPin);
    BoardPinMap::UbloxM6::serial().begin(BoardPinMap::UbloxM6::baud);
}

static void scanI2c()
{
    Serial.print("I2C_SCAN_BEGIN bus=");
    Serial.println(CURRENT_I2C_BUS_NAME);
    for (uint8_t address = 1U; address < 127U; ++address)
    {
        CURRENT_I2C_BUS.beginTransmission(address);
        if (CURRENT_I2C_BUS.endTransmission() == 0)
        {
            Serial.print("I2C_FOUND ");
            printHexByte(address);
            Serial.println();
        }
    }
    Serial.print("I2C_SCAN_END bus=");
    Serial.println(CURRENT_I2C_BUS_NAME);
}

static bool readLisWhoami(uint8_t address, uint8_t &whoami)
{
    CURRENT_I2C_BUS.beginTransmission(address);
    CURRENT_I2C_BUS.write(LIS3MDL_WHOAMI_REG);
    if (CURRENT_I2C_BUS.endTransmission(false) != 0)
    {
        return false;
    }
    if (CURRENT_I2C_BUS.requestFrom(address, static_cast<uint8_t>(1U)) != 1)
    {
        return false;
    }
    whoami = CURRENT_I2C_BUS.read();
    return true;
}

static bool testLis3mdl()
{
    const uint8_t candidates[] = {BoardPinMap::LIS3MDL::i2cAddress, 0x1EU};
    uint8_t detectedAddress = 0U;
    for (uint8_t i = 0U; i < sizeof(candidates); ++i)
    {
        uint8_t whoami = 0U;
        Serial.print("LIS3MDL_WHOAMI addr=");
        printHexByte(candidates[i]);
        if (!readLisWhoami(candidates[i], whoami))
        {
            Serial.println(" no_response");
            continue;
        }

        Serial.print(" value=");
        printHexByte(whoami);
        Serial.println();
        if (whoami == LIS3MDL_WHOAMI_VALUE)
        {
            detectedAddress = candidates[i];
            break;
        }
    }

    if (detectedAddress == 0U || !lis3mdl.begin_I2C(detectedAddress, &CURRENT_I2C_BUS))
    {
        return false;
    }

    lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    sensors_event_t event;
    if (!lis3mdl.getEvent(&event) || !finite3(event.magnetic.x, event.magnetic.y, event.magnetic.z))
    {
        return false;
    }

    Serial.print("LIS3MDL_uT=");
    Serial.print(event.magnetic.x, 3);
    Serial.print(",");
    Serial.print(event.magnetic.y, 3);
    Serial.print(",");
    Serial.println(event.magnetic.z, 3);
    return true;
}

static bool testLsm6dso32()
{
    deselectSpiDevices();
    const bool initOk = lsm6dso32.begin_SPI(BoardPinMap::LSM6DSO32::csPin, &SPI);
    deselectSpiDevices();
    if (!initOk)
    {
        return false;
    }

    lsm6dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
    lsm6dso32.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    lsm6dso32.setAccelDataRate(LSM6DS_RATE_416_HZ);
    lsm6dso32.setGyroDataRate(LSM6DS_RATE_416_HZ);

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    if (!lsm6dso32.getEvent(&accel, &gyro, &temp) ||
        !finite3(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z) ||
        !finite3(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z) ||
        !isfinite(temp.temperature))
    {
        return false;
    }

    Serial.print("LSM6DSO32_accel=");
    Serial.print(accel.acceleration.x, 3);
    Serial.print(",");
    Serial.print(accel.acceleration.y, 3);
    Serial.print(",");
    Serial.print(accel.acceleration.z, 3);
    Serial.print(" tempC=");
    Serial.println(temp.temperature, 2);
    return true;
}

static uint8_t readLoraVersion(uint8_t spiMode)
{
    SPISettings settings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, spiMode);
    deselectSpiDevices();
    SPI.beginTransaction(settings);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, LOW);
    delayMicroseconds(20);
    SPI.transfer(LORA_REG_VERSION & 0x7F);
    const uint8_t value = SPI.transfer(0x00);
    delayMicroseconds(20);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
    SPI.endTransaction();
    return value;
}

static bool testLora()
{
    deselectSpiDevices();
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::resetPin, OUTPUT);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::resetPin, LOW);
    delay(50);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::resetPin, HIGH);
    delay(500);

    const uint8_t mode0 = readLoraVersion(SPI_MODE0);
    const uint8_t mode1 = readLoraVersion(SPI_MODE1);
    const uint8_t mode2 = readLoraVersion(SPI_MODE2);
    const uint8_t mode3 = readLoraVersion(SPI_MODE3);

    Serial.print("LORA_VERSION m0=");
    printHexByte(mode0);
    Serial.print(" m1=");
    printHexByte(mode1);
    Serial.print(" m2=");
    printHexByte(mode2);
    Serial.print(" m3=");
    printHexByte(mode3);
    Serial.println();

    return mode0 == LORA_EXPECTED_VERSION ||
           mode1 == LORA_EXPECTED_VERSION ||
           mode2 == LORA_EXPECTED_VERSION ||
           mode3 == LORA_EXPECTED_VERSION;
}

static bool testGps()
{
    const uint32_t startMs = millis();
    uint32_t bytes = 0UL;
    while ((millis() - startMs) < GPS_TEST_WINDOW_MS)
    {
        while (BoardPinMap::UbloxM6::serial().available() > 0)
        {
            const int c = BoardPinMap::UbloxM6::serial().read();
            if (c >= 0)
            {
                gps.encode(static_cast<char>(c));
                ++bytes;
            }
        }
        if (gps.passedChecksum() > 0UL)
        {
            break;
        }
    }

    Serial.print("GPS_UART bytes=");
    Serial.print(bytes);
    Serial.print(" parsed=");
    Serial.print(gps.passedChecksum());
    Serial.print(" checksum_fail=");
    Serial.print(gps.failedChecksum());
    Serial.print(" fix=");
    Serial.println(gps.location.isValid() ? "yes" : "no");
    return gps.passedChecksum() > 0UL;
}

static void runTests()
{
    Serial.println();
    Serial.println("NURA current stack smoke test");
    Serial.println("Included: LIS3MDL, LSM6DSO32, SX127x LoRa, GPS. Excluded: MPL3115A2, H3LIS331DL.");
    Serial.print("RUN ");
    Serial.println(runCounter++);

    beginBuses();
    printI2cLineLevels();
    scanI2c();

    const bool lisOk = testLis3mdl();
    printBoolResult("LIS3MDL I2C detected", lisOk);

    const bool lsmOk = testLsm6dso32();
    printBoolResult("LSM6DSO32 SPI detected", lsmOk);

    const bool loraOk = testLora();
    printBoolResult("SX127x LoRa SPI detected", loraOk);

    const bool gpsOk = testGps();
    printBoolResult("GY-GPS6MV2 UART NMEA detected", gpsOk);

    Serial.print("SUMMARY ");
    Serial.println((lisOk && lsmOk && loraOk && gpsOk) ? "PASS" : "FAIL");
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 10000)
    {
    }
    runTests();
}

void loop()
{
    delay(5000);
    runTests();
}
