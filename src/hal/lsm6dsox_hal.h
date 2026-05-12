#pragma once

#include <stdint.h>

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct Lsm6dsoxReading
{
    // LSM6DSOX 16 g IMU sample normalized for flight-state consumers.
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

struct Lsm6dsoxCalibration
{
    float accelXMps2Offset = 0.0f;
    float accelYMps2Offset = 0.0f;
    float accelZMps2Offset = 0.0f;

    float gyroXDpsOffset = 0.0f;
    float gyroYDpsOffset = 0.0f;
    float gyroZDpsOffset = 0.0f;
};

class LSM6DSOXHAL
{
public:
    bool begin(uint8_t i2cAddress = LSM6DS_I2CADDR_DEFAULT,
               TwoWire &wire = Wire,
               lsm6ds_accel_range_t accelRange = LSM6DS_ACCEL_RANGE_16_G,
               lsm6ds_gyro_range_t gyroRange = LSM6DS_GYRO_RANGE_2000_DPS,
               lsm6ds_data_rate_t dataRate = LSM6DS_RATE_416_HZ);
    bool read(Lsm6dsoxReading &out, uint32_t nowMs);
    void setCalibration(const Lsm6dsoxCalibration &calibration);
    void clearCalibration();
    bool calibrateStationary(uint16_t sampleCount = 256U,
                             uint16_t sampleDelayMs = 3U,
                             float expectedAccelXMps2 = 0.0f,
                             float expectedAccelYMps2 = 0.0f,
                             float expectedAccelZMps2 = 9.80665f);
    const Lsm6dsoxCalibration &calibration() const;

private:
    static bool validAccelEvent(const sensors_event_t &event);
    static bool validGyroEvent(const sensors_event_t &event);
    static bool validTempEvent(const sensors_event_t &event);

    Adafruit_LSM6DSOX sensor_;
    Lsm6dsoxCalibration calibration_;
    bool initialized_ = false;
};
