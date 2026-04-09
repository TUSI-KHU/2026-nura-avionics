// mpu6050_hal.h
#pragma once

#include <stdint.h>
#include <stddef.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct Mpu6050Reading
{
    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    float gyroXDps = 0.0f;
    float gyroYDps = 0.0f;
    float gyroZDps = 0.0f;

    float temperatureC = 0.0f;
    uint32_t sampleMs = 0;
};

class MPU6050HAL
{
public:
    bool begin(uint8_t i2cAddress = MPU6050_I2CADDR_DEFAULT,
               TwoWire &wire = Wire,
               mpu6050_accel_range_t accelRange = MPU6050_RANGE_8_G,
               mpu6050_gyro_range_t gyroRange = MPU6050_RANGE_500_DEG);
    bool read(Mpu6050Reading &out, uint32_t nowMs);

private:
    bool readBurst(uint8_t startReg, uint8_t *buffer, size_t length);
    static float rawTempToC(int16_t raw);

    Adafruit_MPU6050 sensor_;
    TwoWire *wire_ = &Wire;
    uint8_t i2cAddress_ = MPU6050_I2CADDR_DEFAULT;
    mpu6050_accel_range_t accelRange_ = MPU6050_RANGE_8_G;
    float accelScale_ = 1.f;
    mpu6050_gyro_range_t gyroRange_ = MPU6050_RANGE_500_DEG;
    float gyroScale_ = 1.f;
};
