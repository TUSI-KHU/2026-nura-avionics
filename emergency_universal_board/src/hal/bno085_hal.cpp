#include "hal/bno085_hal.h"

#include <Arduino.h>
#include <math.h>

#include "nura_constants.h"

namespace
{
constexpr uint32_t kReportIntervalUs = 10000UL;

float clampUnit(float value)
{
    if (value > 1.0f)
    {
        return 1.0f;
    }
    if (value < -1.0f)
    {
        return -1.0f;
    }
    return value;
}
}

BNO085HAL::BNO085HAL(int8_t resetPin)
    : sensor_(resetPin)
{
}

bool BNO085HAL::begin(TwoWire &wire, uint8_t i2cAddress)
{
    initialized_ = sensor_.begin_I2C(i2cAddress, &wire);
    if (!initialized_)
    {
        return false;
    }

    haveAccel_ = false;
    haveGyro_ = false;
    latest_ = Bno085Reading{};
    return enableReports();
}

bool BNO085HAL::enableReports()
{
    return sensor_.enableReport(SH2_ACCELEROMETER, kReportIntervalUs) &&
           sensor_.enableReport(SH2_GYROSCOPE_CALIBRATED, kReportIntervalUs) &&
           sensor_.enableReport(SH2_ROTATION_VECTOR, kReportIntervalUs);
}

bool BNO085HAL::read(Bno085Reading &out, uint32_t nowMs)
{
    if (!initialized_)
    {
        return false;
    }

    if (sensor_.wasReset() && !enableReports())
    {
        return false;
    }

    bool received = false;
    sh2_SensorValue_t event;
    while (sensor_.getSensorEvent(&event))
    {
        received = true;
        latest_.sampleMs = nowMs;
        switch (event.sensorId)
        {
        case SH2_ACCELEROMETER:
            latest_.accelXMps2 = event.un.accelerometer.x;
            latest_.accelYMps2 = event.un.accelerometer.y;
            latest_.accelZMps2 = event.un.accelerometer.z;
            haveAccel_ = true;
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            latest_.gyroXDps = event.un.gyroscope.x * NuraConstants::Physics::kRadToDeg;
            latest_.gyroYDps = event.un.gyroscope.y * NuraConstants::Physics::kRadToDeg;
            latest_.gyroZDps = event.un.gyroscope.z * NuraConstants::Physics::kRadToDeg;
            haveGyro_ = true;
            break;
        case SH2_ROTATION_VECTOR:
            publishQuaternion(event.un.rotationVector.real,
                              event.un.rotationVector.i,
                              event.un.rotationVector.j,
                              event.un.rotationVector.k,
                              latest_);
            break;
        default:
            break;
        }
    }

    if (!received || !haveAccel_ || !haveGyro_)
    {
        return false;
    }

    out = latest_;
    out.sampleMs = nowMs;
    return true;
}

void BNO085HAL::publishQuaternion(float real, float i, float j, float k, Bno085Reading &out) const
{
    if (!isfinite(real) || !isfinite(i) || !isfinite(j) || !isfinite(k))
    {
        out.attitudeValid = false;
        return;
    }

    const float sinRollCosPitch = 2.0f * ((real * i) + (j * k));
    const float cosRollCosPitch = 1.0f - (2.0f * ((i * i) + (j * j)));
    const float sinPitch = clampUnit(2.0f * ((real * j) - (k * i)));
    const float sinYawCosPitch = 2.0f * ((real * k) + (i * j));
    const float cosYawCosPitch = 1.0f - (2.0f * ((j * j) + (k * k)));

    out.rollDeg = atan2f(sinRollCosPitch, cosRollCosPitch) * NuraConstants::Physics::kRadToDeg;
    out.pitchDeg = asinf(sinPitch) * NuraConstants::Physics::kRadToDeg;
    out.yawDeg = atan2f(sinYawCosPitch, cosYawCosPitch) * NuraConstants::Physics::kRadToDeg;
    out.attitudeValid = isfinite(out.rollDeg) && isfinite(out.pitchDeg) && isfinite(out.yawDeg);
}
