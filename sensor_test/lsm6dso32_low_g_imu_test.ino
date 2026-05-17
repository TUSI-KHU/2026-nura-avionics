#include <Arduino.h>
#include <SPI.h>
#include "../include/board_pinmap.h"
#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define SPI_SCK_PIN BoardPinMap::LSM6DSO32::sckPin
#define SPI_MISO_PIN BoardPinMap::LSM6DSO32::misoPin
#define SPI_MOSI_PIN BoardPinMap::LSM6DSO32::mosiPin
#define LSM6DSO32_CS_PIN BoardPinMap::LSM6DSO32::chipSelectPin
#define LSM6DSO32_SPI_HZ BoardPinMap::LSM6DSO32::spiFrequencyHz
#define LSM6DSO32_SAMPLE_DELAY_MS 10U
#define LSM6DSO32_REPORT_INTERVAL 100U
#define LSM6DSO32_SUMMARY_INTERVAL 1000U
// ================================================================

Adafruit_LSM6DSO32 imu;

uint32_t sampleCount = 0U;
uint32_t invalidCount = 0U;
uint32_t outOfRangeCount = 0U;
float minAccelMagnitude = 1000000.0f;
float maxAccelMagnitude = 0.0f;

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

static float accelMagnitude(const sensors_event_t &accel)
{
    return sqrtf(accel.acceleration.x * accel.acceleration.x +
                 accel.acceleration.y * accel.acceleration.y +
                 accel.acceleration.z * accel.acceleration.z);
}

static void printSample(const sensors_event_t &accel, const sensors_event_t &gyro, const sensors_event_t &temp)
{
    Serial.print("sample=");
    Serial.print(sampleCount);
    Serial.print(" raw_accel=");
    Serial.print(imu.rawAccX);
    Serial.print(",");
    Serial.print(imu.rawAccY);
    Serial.print(",");
    Serial.print(imu.rawAccZ);
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

static void printSummary()
{
    Serial.print("SUMMARY samples=");
    Serial.print(sampleCount);
    Serial.print(" invalid=");
    Serial.print(invalidCount);
    Serial.print(" accel_out_of_range=");
    Serial.print(outOfRangeCount);
    Serial.print(" accel_mag_range_mps2=");
    Serial.print(minAccelMagnitude, 4);
    Serial.print("..");
    Serial.println(maxAccelMagnitude, 4);

    if (invalidCount == 0U && outOfRangeCount == 0U)
    {
        Serial.println("PASS: long-run window OK");
    }
    else
    {
        Serial.println("FAIL: long-run window detected bad samples");
    }
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000UL)
    {
    }

    Serial.println();
    Serial.println("LSM6DSO32 low-g IMU SPI long-run test");
    Serial.print("PINMAP sck=");
    Serial.print(SPI_SCK_PIN);
    Serial.print(" miso=");
    Serial.print(SPI_MISO_PIN);
    Serial.print(" mosi=");
    Serial.print(SPI_MOSI_PIN);
    Serial.print(" cs=");
    Serial.print(LSM6DSO32_CS_PIN);
    Serial.print(" spi_hz=");
    Serial.println(LSM6DSO32_SPI_HZ);

    if (!imu.begin_SPI(LSM6DSO32_CS_PIN, &SPI, 0, LSM6DSO32_SPI_HZ))
    {
        Serial.println("FAIL: LSM6DSO32 not found on configured SPI bus");
        while (true)
        {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(1000);
        }
    }

    imu.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);

    Serial.println("PASS: WHOAMI/init OK");
}

void loop()
{
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;

    const bool readOk = imu.getEvent(&accel, &gyro, &temp) &&
                        finiteAccel(accel) &&
                        finiteGyro(gyro) &&
                        isfinite(temp.temperature);

    ++sampleCount;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    if (!readOk)
    {
        ++invalidCount;
    }
    else
    {
        const float mag = accelMagnitude(accel);
        minAccelMagnitude = min(minAccelMagnitude, mag);
        maxAccelMagnitude = max(maxAccelMagnitude, mag);

        if (mag < 5.0f || mag > 15.0f)
        {
            ++outOfRangeCount;
        }

        if ((sampleCount % LSM6DSO32_REPORT_INTERVAL) == 0U)
        {
            printSample(accel, gyro, temp);
        }
    }

    if ((sampleCount % LSM6DSO32_SUMMARY_INTERVAL) == 0U)
    {
        printSummary();
    }

    delay(LSM6DSO32_SAMPLE_DELAY_MS);
}
