#include <Arduino.h>
#include <Wire.h>
#include "../include/board_pinmap.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN BoardPinMap::LSM6DSOX::sdaPin
#define I2C_SCL_PIN BoardPinMap::LSM6DSOX::sclPin
#define LSM6DSOX_I2C_ADDR BoardPinMap::LSM6DSOX::i2cAddress
#define LSM6DSOX_SAMPLE_COUNT 80
#define LSM6DSOX_SAMPLE_DELAY_MS 20
// ================================================================

Adafruit_LSM6DSOX imu;

static bool finiteAccel(const sensors_event_t &event)
{
    return isfinite(event.acceleration.x) &&
           isfinite(event.acceleration.y) &&
           isfinite(event.acceleration.z);
}

static bool finiteGyro(const sensors_event_t &event)
{
    return isfinite(event.gyro.x) &&
           isfinite(event.gyro.y) &&
           isfinite(event.gyro.z);
}

static void printSample(uint16_t index, const sensors_event_t &accel, const sensors_event_t &gyro, const sensors_event_t &temp)
{
    Serial.print("sample=");
    Serial.print(index);
    Serial.print(" accel_mps2=");
    Serial.print(accel.acceleration.x, 4);
    Serial.print(",");
    Serial.print(accel.acceleration.y, 4);
    Serial.print(",");
    Serial.print(accel.acceleration.z, 4);
    Serial.print(" gyro_radps=");
    Serial.print(gyro.gyro.x, 4);
    Serial.print(",");
    Serial.print(gyro.gyro.y, 4);
    Serial.print(",");
    Serial.print(gyro.gyro.z, 4);
    Serial.print(" temp_c=");
    Serial.println(temp.temperature, 2);
}

static bool runDefectTest()
{
    float minMag = 1000000.0f;
    float maxMag = 0.0f;
    bool anyInvalid = false;
    bool anyNonZero = false;

    for (uint16_t i = 0; i < LSM6DSOX_SAMPLE_COUNT; ++i)
    {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        if (!imu.getEvent(&accel, &gyro, &temp) || !finiteAccel(accel) || !finiteGyro(gyro) || !isfinite(temp.temperature))
        {
            anyInvalid = true;
            continue;
        }

        const float mag = sqrtf(accel.acceleration.x * accel.acceleration.x +
                                accel.acceleration.y * accel.acceleration.y +
                                accel.acceleration.z * accel.acceleration.z);
        minMag = min(minMag, mag);
        maxMag = max(maxMag, mag);

        if (fabsf(accel.acceleration.x) > 0.01f ||
            fabsf(accel.acceleration.y) > 0.01f ||
            fabsf(accel.acceleration.z) > 0.01f)
        {
            anyNonZero = true;
        }

        if ((i % 10U) == 0U)
        {
            printSample(i, accel, gyro, temp);
        }

        delay(LSM6DSOX_SAMPLE_DELAY_MS);
    }

    Serial.print("accel_mag_range_mps2=");
    Serial.print(minMag, 4);
    Serial.print("..");
    Serial.println(maxMag, 4);

    if (anyInvalid)
    {
        Serial.println("FAIL: invalid or non-finite sample detected");
        return false;
    }

    if (!anyNonZero)
    {
        Serial.println("FAIL: accelerometer appears stuck at zero");
        return false;
    }

    if (minMag < 5.0f || maxMag > 15.0f)
    {
        Serial.println("WARN: acceleration magnitude is outside normal stationary 1g range");
    }

    Serial.println("PASS: LSM6DSOX responded and produced usable IMU samples");
    return true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("LSM6DSOX low-g IMU defect test");

    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

    if (!imu.begin_I2C(LSM6DSOX_I2C_ADDR, &Wire))
    {
        Serial.println("FAIL: LSM6DSOX not found on configured I2C address");
        return;
    }

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_416_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_416_HZ);

    Serial.println("PASS: WHOAMI/init OK");
    runDefectTest();
}

void loop()
{
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    if (imu.getEvent(&accel, &gyro, &temp))
    {
        printSample(0, accel, gyro, temp);
    }
    delay(1000);
}
