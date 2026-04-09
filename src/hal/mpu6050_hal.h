#pragma once

#include <stdint.h>

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
    bool begin(uint8_t i2cAddress = MPU6050_I2CADDR_DEFAULT, TwoWire &wire = Wire);
    bool read(Mpu6050Reading &out, uint32_t nowMs);
    bool readZ(float &accelZMps2, float &gyroZDps);
    bool isReady() const;

private:
    static float radPerSecToDegPerSec(float radPerSec);

    Adafruit_MPU6050 sensor_;
    bool ready_ = false;
};
