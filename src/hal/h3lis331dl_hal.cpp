#include "h3lis331dl_hal.h"

#include <Arduino.h>
#include <math.h>

namespace
{
    constexpr float kGravity = 9.80665f;
}

bool H3LIS331DLHAL::begin(uint8_t i2cAddress,
                          TwoWire &wire,
                          h3lis331dl_range_t range,
                          lis331_data_rate_t dataRate)
{
    initialized_ = false;

    if (!sensor_.begin_I2C(i2cAddress, &wire))
    {
        return false;
    }

    sensor_.setRange(range);
    sensor_.setDataRate(dataRate);

    initialized_ = true;
    return true;
}

bool H3LIS331DLHAL::read(H3lis331dlReading &out, uint32_t nowMs)
{
    if (!initialized_)
    {
        return false;
    }

    sensors_event_t event;
    if (!sensor_.getEvent(&event) || !validEvent(event))
    {
        return false;
    }

    out.accelXG = (event.acceleration.x / kGravity) - calibration_.accelXGOffset;
    out.accelYG = (event.acceleration.y / kGravity) - calibration_.accelYGOffset;
    out.accelZG = (event.acceleration.z / kGravity) - calibration_.accelZGOffset;

    out.accelXMps2 = out.accelXG * kGravity;
    out.accelYMps2 = out.accelYG * kGravity;
    out.accelZMps2 = out.accelZG * kGravity;

    out.rawX = sensor_.x;
    out.rawY = sensor_.y;
    out.rawZ = sensor_.z;
    out.sampleMs = nowMs;

    return true;
}

void H3LIS331DLHAL::setCalibration(const H3lis331dlCalibration &calibration)
{
    calibration_ = calibration;
}

void H3LIS331DLHAL::clearCalibration()
{
    calibration_ = H3lis331dlCalibration{};
}

bool H3LIS331DLHAL::calibrateStationary(uint16_t sampleCount,
                                        uint16_t sampleDelayMs,
                                        float expectedAccelXG,
                                        float expectedAccelYG,
                                        float expectedAccelZG)
{
    if (!initialized_ || sampleCount == 0U)
    {
        return false;
    }

    const H3lis331dlCalibration previousCalibration = calibration_;
    clearCalibration();

    double accelXGSum = 0.0;
    double accelYGSum = 0.0;
    double accelZGSum = 0.0;

    for (uint16_t i = 0U; i < sampleCount; ++i)
    {
        sensors_event_t event;
        if (!sensor_.getEvent(&event) || !validEvent(event))
        {
            calibration_ = previousCalibration;
            return false;
        }

        accelXGSum += event.acceleration.x / kGravity;
        accelYGSum += event.acceleration.y / kGravity;
        accelZGSum += event.acceleration.z / kGravity;

        if (sampleDelayMs > 0U)
        {
            delay(sampleDelayMs);
        }
    }

    const double sampleCountD = static_cast<double>(sampleCount);
    calibration_.accelXGOffset = static_cast<float>((accelXGSum / sampleCountD) - expectedAccelXG);
    calibration_.accelYGOffset = static_cast<float>((accelYGSum / sampleCountD) - expectedAccelYG);
    calibration_.accelZGOffset = static_cast<float>((accelZGSum / sampleCountD) - expectedAccelZG);

    return true;
}

const H3lis331dlCalibration &H3LIS331DLHAL::calibration() const
{
    return calibration_;
}

bool H3LIS331DLHAL::validEvent(const sensors_event_t &event)
{
    return isfinite(event.acceleration.x) &&
           isfinite(event.acceleration.y) &&
           isfinite(event.acceleration.z);
}
