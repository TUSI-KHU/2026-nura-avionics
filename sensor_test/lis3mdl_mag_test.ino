#include <Arduino.h>
#include <Wire.h>
#include "../include/board_pinmap.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN BoardPinMap::LIS3MDL::sdaPin
#define I2C_SCL_PIN BoardPinMap::LIS3MDL::sclPin
#define LIS3MDL_I2C_ADDR BoardPinMap::LIS3MDL::i2cAddress
#define LIS3MDL_SAMPLE_COUNT 80
#define LIS3MDL_SAMPLE_DELAY_MS 20
// ================================================================

Adafruit_LIS3MDL mag;

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

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("LIS3MDL magnetometer defect test");

    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

    if (!mag.begin_I2C(LIS3MDL_I2C_ADDR, &Wire))
    {
        Serial.println("FAIL: LIS3MDL not found on configured I2C address");
        return;
    }

    mag.setRange(LIS3MDL_RANGE_16_GAUSS);
    mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    Serial.println("PASS: WHOAMI/init OK");
    runDefectTest();
}

void loop()
{
    sensors_event_t event;
    if (mag.getEvent(&event))
    {
        printSample(0, event);
    }
    delay(1000);
}
