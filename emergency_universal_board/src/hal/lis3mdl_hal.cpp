#include "lis3mdl_hal.h"

#include <Arduino.h>
#include <float.h>
#include <math.h>

bool LIS3MDLHAL::begin(uint8_t i2cAddress,
                       TwoWire &wire,
                       lis3mdl_range_t range,
                       lis3mdl_dataRate_t dataRate,
                       lis3mdl_performancemode_t performanceMode)
{
    initialized_ = false;

    if (!sensor_.begin_I2C(i2cAddress, &wire))
    {
        return false;
    }

    sensor_.setRange(range);
    sensor_.setDataRate(dataRate);
    sensor_.setPerformanceMode(performanceMode);
    sensor_.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    initialized_ = true;
    return true;
}

bool LIS3MDLHAL::read(Lis3mdlReading &out, uint32_t nowMs)
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

    out.magXuT = (event.magnetic.x - calibration_.hardIronXuT) * calibration_.scaleX;
    out.magYuT = (event.magnetic.y - calibration_.hardIronYuT) * calibration_.scaleY;
    out.magZuT = (event.magnetic.z - calibration_.hardIronZuT) * calibration_.scaleZ;

    out.magXGauss = out.magXuT * 0.01f;
    out.magYGauss = out.magYuT * 0.01f;
    out.magZGauss = out.magZuT * 0.01f;

    out.rawX = sensor_.x;
    out.rawY = sensor_.y;
    out.rawZ = sensor_.z;
    out.sampleMs = nowMs;

    return true;
}

void LIS3MDLHAL::setCalibration(const Lis3mdlCalibration &calibration)
{
    calibration_ = calibration;
    if (calibration_.scaleX == 0.0f)
    {
        calibration_.scaleX = 1.0f;
    }
    if (calibration_.scaleY == 0.0f)
    {
        calibration_.scaleY = 1.0f;
    }
    if (calibration_.scaleZ == 0.0f)
    {
        calibration_.scaleZ = 1.0f;
    }
}

void LIS3MDLHAL::clearCalibration()
{
    calibration_ = Lis3mdlCalibration{};
}

bool LIS3MDLHAL::calibrateHardIron(uint32_t durationMs, uint16_t sampleDelayMs)
{
    if (!initialized_ || durationMs == 0UL)
    {
        return false;
    }

    float minX = FLT_MAX;
    float minY = FLT_MAX;
    float minZ = FLT_MAX;
    float maxX = -FLT_MAX;
    float maxY = -FLT_MAX;
    float maxZ = -FLT_MAX;
    uint16_t sampleCount = 0U;

    const uint32_t startMs = millis();
    while ((millis() - startMs) < durationMs)
    {
        sensors_event_t event;
        if (!sensor_.getEvent(&event) || !validEvent(event))
        {
            return false;
        }

        if (event.magnetic.x < minX)
        {
            minX = event.magnetic.x;
        }
        if (event.magnetic.y < minY)
        {
            minY = event.magnetic.y;
        }
        if (event.magnetic.z < minZ)
        {
            minZ = event.magnetic.z;
        }
        if (event.magnetic.x > maxX)
        {
            maxX = event.magnetic.x;
        }
        if (event.magnetic.y > maxY)
        {
            maxY = event.magnetic.y;
        }
        if (event.magnetic.z > maxZ)
        {
            maxZ = event.magnetic.z;
        }

        ++sampleCount;
        if (sampleDelayMs > 0U)
        {
            delay(sampleDelayMs);
        }
    }

    if (sampleCount < 2U)
    {
        return false;
    }

    calibration_.hardIronXuT = (minX + maxX) * 0.5f;
    calibration_.hardIronYuT = (minY + maxY) * 0.5f;
    calibration_.hardIronZuT = (minZ + maxZ) * 0.5f;
    calibration_.scaleX = 1.0f;
    calibration_.scaleY = 1.0f;
    calibration_.scaleZ = 1.0f;

    return true;
}

const Lis3mdlCalibration &LIS3MDLHAL::calibration() const
{
    return calibration_;
}

bool LIS3MDLHAL::validEvent(const sensors_event_t &event)
{
    return isfinite(event.magnetic.x) &&
           isfinite(event.magnetic.y) &&
           isfinite(event.magnetic.z);
}
