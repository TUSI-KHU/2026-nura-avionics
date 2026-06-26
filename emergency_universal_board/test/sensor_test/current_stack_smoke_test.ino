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
#define GPS_TEST_WINDOW_MS 8000UL
#define MPL3115A2_CONVERSION_TIMEOUT_MS 700UL
#define LORA_SPI_FREQUENCY_HZ 125000UL
#define LORA_REG_VERSION 0x42
#define LORA_EXPECTED_VERSION 0x12
#define LIS3MDL_WHOAMI_REG 0x0F
#define LIS3MDL_WHOAMI_VALUE 0x3D
#define MPL3115A2_WHOAMI_REG 0x0C
#define MPL3115A2_WHOAMI_VALUE 0xC4
#define LIS_I2C_BUS BoardPinMap::LIS3MDL::wire()
#define LIS_I2C_BUS_NAME BoardPinMap::I2c1Bus::name()
#define LIS_I2C_SDA_PIN BoardPinMap::LIS3MDL::sdaPin
#define LIS_I2C_SCL_PIN BoardPinMap::LIS3MDL::sclPin
#define MPL_I2C_BUS BoardPinMap::MPL3115A2::wire()
#define MPL_I2C_BUS_NAME BoardPinMap::I2c0Bus::name()
#define MPL_I2C_SDA_PIN BoardPinMap::MPL3115A2::sdaPin
#define MPL_I2C_SCL_PIN BoardPinMap::MPL3115A2::sclPin

Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DSO32 lsm6dso32;
Adafruit_H3LIS331 h3lis331dl;
Adafruit_MPL3115A2 mpl3115a2;
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
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::ssPin, OUTPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
}

static void printI2cLineLevels(const char *name, uint8_t sdaPin, uint8_t sclPin)
{
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
    delayMicroseconds(100);
    Serial.print("I2C_LINES bus=");
    Serial.print(name);
    Serial.print(" sda=");
    Serial.print(sdaPin);
    Serial.print(":");
    Serial.print(digitalRead(sdaPin));
    Serial.print(" scl=");
    Serial.print(sclPin);
    Serial.print(":");
    Serial.println(digitalRead(sclPin));
}

static void beginBuses()
{
    MPL_I2C_BUS.setSDA(MPL_I2C_SDA_PIN);
    MPL_I2C_BUS.setSCL(MPL_I2C_SCL_PIN);
    MPL_I2C_BUS.begin();
    MPL_I2C_BUS.setClock(BoardPinMap::I2c0Bus::clockHz);

    LIS_I2C_BUS.setSDA(LIS_I2C_SDA_PIN);
    LIS_I2C_BUS.setSCL(LIS_I2C_SCL_PIN);
    LIS_I2C_BUS.begin();
    LIS_I2C_BUS.setClock(BoardPinMap::I2c1Bus::clockHz);

    deselectSpiDevices();
    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();

    BoardPinMap::UbloxM6::serial().setRX(BoardPinMap::UbloxM6::rxPin);
    BoardPinMap::UbloxM6::serial().setTX(BoardPinMap::UbloxM6::txPin);
    BoardPinMap::UbloxM6::serial().begin(BoardPinMap::UbloxM6::baud);
}

static void scanI2c(TwoWire &bus, const char *name)
{
    Serial.print("I2C_SCAN_BEGIN bus=");
    Serial.println(name);
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

static bool testLis3mdl()
{
    const uint8_t candidates[] = {BoardPinMap::LIS3MDL::i2cAddress, 0x1EU};
    uint8_t detectedAddress = 0U;
    for (uint8_t i = 0U; i < sizeof(candidates); ++i)
    {
        uint8_t whoami = 0U;
        Serial.print("LIS3MDL_WHOAMI addr=");
        printHexByte(candidates[i]);
        if (!readI2cRegister(LIS_I2C_BUS, candidates[i], LIS3MDL_WHOAMI_REG, whoami))
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

    if (detectedAddress == 0U || !lis3mdl.begin_I2C(detectedAddress, &LIS_I2C_BUS))
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
    uint8_t whoami = 0U;
    Serial.print("MPL3115A2_WHOAMI addr=");
    printHexByte(BoardPinMap::MPL3115A2::i2cAddress);
    if (!readI2cRegister(MPL_I2C_BUS, BoardPinMap::MPL3115A2::i2cAddress, MPL3115A2_WHOAMI_REG, whoami))
    {
        Serial.println(" no_response");
        return false;
    }
    Serial.print(" value=");
    printHexByte(whoami);
    Serial.println();
    if (whoami != MPL3115A2_WHOAMI_VALUE)
    {
        return false;
    }

    if (!mpl3115a2.begin(&MPL_I2C_BUS))
    {
        Serial.println("MPL3115A2_INIT failed");
        return false;
    }
    Serial.println("MPL3115A2_INIT ok");

    mpl3115a2.setMode(MPL3115A2_BAROMETER);
    mpl3115a2.startOneShot();
    const uint32_t startMs = millis();
    while (!mpl3115a2.conversionComplete() && (millis() - startMs) < MPL3115A2_CONVERSION_TIMEOUT_MS)
    {
        delay(1);
    }

    if (!mpl3115a2.conversionComplete())
    {
        Serial.println("MPL3115A2_CONVERSION timeout");
        return false;
    }

    const float pressureHpa = mpl3115a2.getLastConversionResults(MPL3115A2_PRESSURE);
    const float temperatureC = mpl3115a2.getLastConversionResults(MPL3115A2_TEMPERATURE);
    if (!isfinite(pressureHpa) || !isfinite(temperatureC))
    {
        Serial.println("MPL3115A2_SAMPLE invalid");
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

static bool testH3lis331dl()
{
    deselectSpiDevices();
    const bool initOk = h3lis331dl.begin_SPI(BoardPinMap::H3LIS331DL::csPin, &SPI);
    deselectSpiDevices();
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
    Serial.println("BEGIN: SX127x LoRa SPI");
    Serial.flush();
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
    Serial.println("BEGIN: GY-GPS6MV2 UART");
    Serial.flush();
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
    Serial.println("Included: LIS3MDL, MPL3115A2, LSM6DSO32, H3LIS331DL, SX127x LoRa, GPS.");
    Serial.print("RUN ");
    Serial.println(runCounter++);

    printI2cLineLevels(MPL_I2C_BUS_NAME, MPL_I2C_SDA_PIN, MPL_I2C_SCL_PIN);
    printI2cLineLevels(LIS_I2C_BUS_NAME, LIS_I2C_SDA_PIN, LIS_I2C_SCL_PIN);
    beginBuses();
    scanI2c(MPL_I2C_BUS, MPL_I2C_BUS_NAME);
    scanI2c(LIS_I2C_BUS, LIS_I2C_BUS_NAME);

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

    const bool loraOk = testLora();
    printBoolResult("SX127x LoRa SPI detected", loraOk);

    Serial.print("SUMMARY ");
    Serial.println((lisOk && mplOk && lsmOk && h3lOk && loraOk && gpsOk) ? "PASS" : "FAIL");
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
