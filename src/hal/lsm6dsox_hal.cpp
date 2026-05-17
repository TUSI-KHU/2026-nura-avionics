#include "lsm6dsox_hal.h"

#include <Arduino.h>
#include <math.h>

namespace
{
    constexpr float kRadToDeg = 57.2957795f;
}

bool LSM6DSOXHAL::begin(uint8_t csPin,
                        SPIClass &spi,
                        lsm6ds_accel_range_t accelRange,
                        lsm6ds_gyro_range_t gyroRange,
                        lsm6ds_data_rate_t dataRate)
{
    initialized_ = false;

    if (!sensor_.begin_SPI(csPin, &spi))
    {
        return false;
    }

    sensor_.setAccelRange(accelRange);
    sensor_.setGyroRange(gyroRange);
    sensor_.setAccelDataRate(dataRate);
    sensor_.setGyroDataRate(dataRate);

    initialized_ = true;
    return true;
}

bool LSM6DSOXHAL::read(Lsm6dsoxReading &out, uint32_t nowMs)
{
    if (!initialized_)
    {
        return false;
    }

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    if (!sensor_.getEvent(&accel, &gyro, &temp))
    {
        return false;
    }

    if (!validAccelEvent(accel) || !validGyroEvent(gyro) || !validTempEvent(temp))
    {
        return false;
    }

    out.accelXMps2 = accel.acceleration.x - calibration_.accelXMps2Offset;
    out.accelYMps2 = accel.acceleration.y - calibration_.accelYMps2Offset;
    out.accelZMps2 = accel.acceleration.z - calibration_.accelZMps2Offset;

    out.gyroXDps = (gyro.gyro.x * kRadToDeg) - calibration_.gyroXDpsOffset;
    out.gyroYDps = (gyro.gyro.y * kRadToDeg) - calibration_.gyroYDpsOffset;
    out.gyroZDps = (gyro.gyro.z * kRadToDeg) - calibration_.gyroZDpsOffset;

    out.temperatureC = temp.temperature;
    out.sampleMs = nowMs;

    out.rawAccelX = sensor_.rawAccX;
    out.rawAccelY = sensor_.rawAccY;
    out.rawAccelZ = sensor_.rawAccZ;
    out.rawGyroX = sensor_.rawGyroX;
    out.rawGyroY = sensor_.rawGyroY;
    out.rawGyroZ = sensor_.rawGyroZ;

    return true;
}

void LSM6DSOXHAL::setCalibration(const Lsm6dsoxCalibration &calibration)
{
    calibration_ = calibration;
}

void LSM6DSOXHAL::clearCalibration()
{
    calibration_ = Lsm6dsoxCalibration{};
}

bool LSM6DSOXHAL::calibrateStationary(uint16_t sampleCount,
                                      uint16_t sampleDelayMs,
                                      float expectedAccelXMps2,
                                      float expectedAccelYMps2,
                                      float expectedAccelZMps2)
{
    if (!initialized_ || sampleCount == 0U)
    {
        return false;
    }

    const Lsm6dsoxCalibration previousCalibration = calibration_;
    clearCalibration();

    double accelXSum = 0.0;
    double accelYSum = 0.0;
    double accelZSum = 0.0;
    double gyroXSum = 0.0;
    double gyroYSum = 0.0;
    double gyroZSum = 0.0;

    for (uint16_t i = 0U; i < sampleCount; ++i)
    {
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temp;
        if (!sensor_.getEvent(&accel, &gyro, &temp) ||
            !validAccelEvent(accel) ||
            !validGyroEvent(gyro) ||
            !validTempEvent(temp))
        {
            calibration_ = previousCalibration;
            return false;
        }

        accelXSum += accel.acceleration.x;
        accelYSum += accel.acceleration.y;
        accelZSum += accel.acceleration.z;
        gyroXSum += gyro.gyro.x * kRadToDeg;
        gyroYSum += gyro.gyro.y * kRadToDeg;
        gyroZSum += gyro.gyro.z * kRadToDeg;

        if (sampleDelayMs > 0U)
        {
            delay(sampleDelayMs);
        }
    }

    const double sampleCountD = static_cast<double>(sampleCount);
    calibration_.accelXMps2Offset = static_cast<float>((accelXSum / sampleCountD) - expectedAccelXMps2);
    calibration_.accelYMps2Offset = static_cast<float>((accelYSum / sampleCountD) - expectedAccelYMps2);
    calibration_.accelZMps2Offset = static_cast<float>((accelZSum / sampleCountD) - expectedAccelZMps2);
    calibration_.gyroXDpsOffset = static_cast<float>(gyroXSum / sampleCountD);
    calibration_.gyroYDpsOffset = static_cast<float>(gyroYSum / sampleCountD);
    calibration_.gyroZDpsOffset = static_cast<float>(gyroZSum / sampleCountD);

    return true;
}

const Lsm6dsoxCalibration &LSM6DSOXHAL::calibration() const
{
    return calibration_;
}

bool LSM6DSOXHAL::validAccelEvent(const sensors_event_t &event)
{
    return isfinite(event.acceleration.x) &&
           isfinite(event.acceleration.y) &&
           isfinite(event.acceleration.z);
}

bool LSM6DSOXHAL::validGyroEvent(const sensors_event_t &event)
{
    return isfinite(event.gyro.x) &&
           isfinite(event.gyro.y) &&
           isfinite(event.gyro.z);
}

bool LSM6DSOXHAL::validTempEvent(const sensors_event_t &event)
{
    return isfinite(event.temperature);
}
