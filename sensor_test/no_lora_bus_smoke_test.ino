#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "board_pinmap.h"

#include <Adafruit_H3LIS331.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <math.h>

#define SERIAL_BAUD 115200
#define GPS_TEST_WINDOW_MS 12000UL

Adafruit_LIS3MDL lis3mdl;
Adafruit_MPL3115A2 mpl3115a2;
Adafruit_LSM6DSO32 lsm6dso32;
Adafruit_H3LIS331 h3lis331dl;
TinyGPSPlus gps;

static uint32_t runCounter = 0UL;

static void printBoolResult(const char *name, bool ok)
{
    Serial.print(ok ? "PASS: " : "FAIL: ");
    Serial.println(name);
}

static void printHexByte(uint8_t value)
{
    Serial.print("0x");
    if (value < 16U)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

static bool finite3(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z);
}

static void deselectSpiSensors()
{
    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);
}

static void beginBuses()
{
    Wire.setSDA(BoardPinMap::LIS3MDL::sdaPin);
    Wire.setSCL(BoardPinMap::LIS3MDL::sclPin);
    Wire.begin();

    deselectSpiSensors();
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
    Serial.println("I2C_SCAN_BEGIN");
    for (uint8_t address = 1U; address < 127U; ++address)
    {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("I2C_FOUND ");
            printHexByte(address);
            Serial.println();
        }
    }
    Serial.println("I2C_SCAN_END");
}

static bool testLis3mdl()
{
    if (!lis3mdl.begin_I2C(BoardPinMap::LIS3MDL::i2cAddress, &Wire))
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

static bool testMpl3115a2()
{
    if (!mpl3115a2.begin(&Wire))
    {
        return false;
    }

    mpl3115a2.setMode(MPL3115A2_BAROMETER);
    mpl3115a2.startOneShot();
    const uint32_t startMs = millis();
    while (!mpl3115a2.conversionComplete() && (millis() - startMs) < 200UL)
    {
        delay(1);
    }

    if (!mpl3115a2.conversionComplete())
    {
        return false;
    }

    const float pressureHpa = mpl3115a2.getLastConversionResults(MPL3115A2_PRESSURE);
    const float temperatureC = mpl3115a2.getLastConversionResults(MPL3115A2_TEMPERATURE);
    if (!isfinite(pressureHpa) || !isfinite(temperatureC))
    {
        return false;
    }

    Serial.print("MPL3115A2_hPa=");
    Serial.print(pressureHpa, 3);
    Serial.print(" tempC=");
    Serial.println(temperatureC, 2);
    return true;
}

static bool testLsm6dso32()
{
    deselectSpiSensors();
    const bool initOk = lsm6dso32.begin_SPI(BoardPinMap::LSM6DSO32::csPin, &SPI);
    deselectSpiSensors();
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

static bool testH3lis331dl()
{
    deselectSpiSensors();
    const bool initOk = h3lis331dl.begin_SPI(BoardPinMap::H3LIS331DL::csPin, &SPI);
    deselectSpiSensors();
    if (!initOk)
    {
        return false;
    }

    h3lis331dl.setRange(H3LIS331_RANGE_200_G);
    h3lis331dl.setDataRate(LIS331_DATARATE_1000_HZ);

    sensors_event_t event;
    if (!h3lis331dl.getEvent(&event) ||
        !finite3(event.acceleration.x, event.acceleration.y, event.acceleration.z))
    {
        return false;
    }

    Serial.print("H3LIS331DL_accel=");
    Serial.print(event.acceleration.x, 3);
    Serial.print(",");
    Serial.print(event.acceleration.y, 3);
    Serial.print(",");
    Serial.println(event.acceleration.z, 3);
    return true;
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

static void runAllTests()
{
    Serial.println();
    Serial.println("NURA no-LoRa bus smoke test");
    Serial.println("I2C: LIS3MDL/MPL3115A2, SPI: LSM6DSO32/H3LIS331DL, UART: GPS");
    Serial.print("RUN ");
    Serial.println(runCounter++);

    beginBuses();
    scanI2c();

    const bool lisOk = testLis3mdl();
    printBoolResult("LIS3MDL I2C detected", lisOk);

    const bool mplOk = testMpl3115a2();
    printBoolResult("MPL3115A2 I2C detected", mplOk);

    const bool lsmOk = testLsm6dso32();
    printBoolResult("LSM6DSO32 SPI detected", lsmOk);

    const bool h3lOk = testH3lis331dl();
    printBoolResult("H3LIS331DL SPI detected", h3lOk);

    const bool gpsOk = testGps();
    printBoolResult("GY-GPS6MV2 UART NMEA detected", gpsOk);

    Serial.print("SUMMARY ");
    Serial.println((lisOk && mplOk && lsmOk && h3lOk && gpsOk) ? "PASS" : "FAIL");
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 10000)
    {
    }

    runAllTests();
}

void loop()
{
    delay(5000);
    runAllTests();
}
