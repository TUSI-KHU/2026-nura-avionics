#pragma once

#include <stdint.h>

#include <Adafruit_LSM6DSO32.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct Lsm6dso32Reading
{
    // LSM6DSO32 IMU sample. Default setup still uses 16 g unless configured otherwise.
    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    float gyroXDps = 0.0f;
    float gyroYDps = 0.0f;
    float gyroZDps = 0.0f;

    float temperatureC = 0.0f;
    uint32_t sampleMs = 0;

    int16_t rawAccelX = 0;
    int16_t rawAccelY = 0;
    int16_t rawAccelZ = 0;
    int16_t rawGyroX = 0;
    int16_t rawGyroY = 0;
    int16_t rawGyroZ = 0;
};

struct Lsm6dso32Calibration
{
    float accelXMps2Offset = 0.0f;
    float accelYMps2Offset = 0.0f;
    float accelZMps2Offset = 0.0f;

    float gyroXDpsOffset = 0.0f;
    float gyroYDpsOffset = 0.0f;
    float gyroZDpsOffset = 0.0f;
};

class LSM6DSO32HAL
{
public:
    bool begin(uint8_t i2cAddress = LSM6DS_I2CADDR_DEFAULT,
               TwoWire &wire = Wire,
               lsm6dso32_accel_range_t accelRange = LSM6DSO32_ACCEL_RANGE_16_G,
               lsm6ds_gyro_range_t gyroRange = LSM6DS_GYRO_RANGE_2000_DPS,
               lsm6ds_data_rate_t dataRate = LSM6DS_RATE_416_HZ);
    bool read(Lsm6dso32Reading &out, uint32_t nowMs);
    void setCalibration(const Lsm6dso32Calibration &calibration);
    void clearCalibration();
    bool calibrateStationary(uint16_t sampleCount = 256U,
                             uint16_t sampleDelayMs = 3U,
                             float expectedAccelXMps2 = 0.0f,
                             float expectedAccelYMps2 = 0.0f,
                             float expectedAccelZMps2 = 9.80665f);
    const Lsm6dso32Calibration &calibration() const;

private:
    static bool validAccelEvent(const sensors_event_t &event);
    static bool validGyroEvent(const sensors_event_t &event);
    static bool validTempEvent(const sensors_event_t &event);

    Adafruit_LSM6DSO32 sensor_;
    Lsm6dso32Calibration calibration_;
    bool initialized_ = false;
};
