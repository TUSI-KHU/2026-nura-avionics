#include "mpu6050_hal.h"

namespace
{
    constexpr float kRadToDeg = 57.2957795f;
}

bool MPU6050HAL::begin(uint8_t i2cAddress, TwoWire &wire)
{
    i2cAddress_ = i2cAddress;
    wire_ = &wire;

    if (!sensor_.begin(i2cAddress_, wire_))
    {
        ready_ = false;
        return false;
    }

    configureSensor();

    ready_ = true;
    return true;
}

bool MPU6050HAL::read(Mpu6050Reading &out, uint32_t nowMs)
{
    if (wire_ == 0)
    {
        ready_ = false;
        return false;
    }

    if (!ready_)
    {
        return false;
    }

    if (!probeDevice())
    {
        ready_ = false;
        return false;
    }

    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;
    sensor_.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    out.accelXMps2 = accelEvent.acceleration.x;
    out.accelYMps2 = accelEvent.acceleration.y;
    out.accelZMps2 = accelEvent.acceleration.z;

    out.gyroXDps = radPerSecToDegPerSec(gyroEvent.gyro.x);
    out.gyroYDps = radPerSecToDegPerSec(gyroEvent.gyro.y);
    out.gyroZDps = radPerSecToDegPerSec(gyroEvent.gyro.z);

    out.temperatureC = tempEvent.temperature;
    out.sampleMs = nowMs;

    return true;
}

bool MPU6050HAL::isReady() const
{
    return ready_;
}

bool MPU6050HAL::probeDevice() const
{
    wire_->beginTransmission(i2cAddress_);
    const uint8_t err = wire_->endTransmission();
    return err == 0U;
}

void MPU6050HAL::configureSensor()
{
    sensor_.setAccelerometerRange(MPU6050_RANGE_8_G);
    sensor_.setGyroRange(MPU6050_RANGE_500_DEG);
    sensor_.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

float MPU6050HAL::radPerSecToDegPerSec(float radPerSec)
{
    return radPerSec * kRadToDeg;
}
