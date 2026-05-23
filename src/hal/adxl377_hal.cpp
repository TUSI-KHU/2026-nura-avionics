#include "adxl377_hal.h"

#include <math.h>

#include "nura_constants.h"

bool ADXL377HAL::begin(uint8_t xPin,
                       uint8_t yPin,
                       uint8_t zPin,
                       float referenceVoltage,
                       uint8_t adcResolutionBits,
                       float zeroGVoltage,
                       float sensitivityMvPerG)
{
    if (referenceVoltage <= 0.0f || adcResolutionBits == 0U || adcResolutionBits > 16U)
    {
        initialized_ = false;
        return false;
    }

    xPin_ = xPin;
    yPin_ = yPin;
    zPin_ = zPin;
    referenceVoltage_ = referenceVoltage;
    adcMax_ = static_cast<uint16_t>((1UL << adcResolutionBits) - 1UL);

    zeroGVoltage_ = zeroGVoltage > 0.0f ? zeroGVoltage : referenceVoltage_ * 0.5f;
    const float scaledSensitivity = NuraConstants::ADXL377::kDatasheetSensitivityMvPerG *
                                    (referenceVoltage_ / NuraConstants::ADXL377::kDatasheetReferenceVoltage);
    sensitivityVPerG_ = (sensitivityMvPerG > 0.0f ? sensitivityMvPerG : scaledSensitivity) * 0.001f;
    zeroGVoltageX_ = zeroGVoltage_;
    zeroGVoltageY_ = zeroGVoltage_;
    zeroGVoltageZ_ = zeroGVoltage_;

    analogReadResolution(adcResolutionBits);
    initialized_ = true;
    return true;
}

bool ADXL377HAL::read(Adxl377Reading &out, uint32_t nowMs) const
{
    if (!initialized_)
    {
        return false;
    }

    out.rawX = static_cast<uint16_t>(analogRead(xPin_));
    out.rawY = static_cast<uint16_t>(analogRead(yPin_));
    out.rawZ = static_cast<uint16_t>(analogRead(zPin_));

    out.voltageX = rawToVoltage(out.rawX);
    out.voltageY = rawToVoltage(out.rawY);
    out.voltageZ = rawToVoltage(out.rawZ);

    out.accelXG = voltageToG(out.voltageX, zeroGVoltageX_);
    out.accelYG = voltageToG(out.voltageY, zeroGVoltageY_);
    out.accelZG = voltageToG(out.voltageZ, zeroGVoltageZ_);

    out.accelXMps2 = out.accelXG * NuraConstants::Physics::kGravityMps2;
    out.accelYMps2 = out.accelYG * NuraConstants::Physics::kGravityMps2;
    out.accelZMps2 = out.accelZG * NuraConstants::Physics::kGravityMps2;
    out.sampleMs = nowMs;

    return true;
}

void ADXL377HAL::setCalibration(const Adxl377Calibration &calibration)
{
    if (calibration.zeroGVoltageX > 0.0f)
    {
        zeroGVoltageX_ = calibration.zeroGVoltageX;
    }
    if (calibration.zeroGVoltageY > 0.0f)
    {
        zeroGVoltageY_ = calibration.zeroGVoltageY;
    }
    if (calibration.zeroGVoltageZ > 0.0f)
    {
        zeroGVoltageZ_ = calibration.zeroGVoltageZ;
    }
    if (calibration.sensitivityMvPerG > 0.0f)
    {
        sensitivityVPerG_ = calibration.sensitivityMvPerG * 0.001f;
    }
}

void ADXL377HAL::clearCalibration()
{
    zeroGVoltage_ = referenceVoltage_ * 0.5f;
    zeroGVoltageX_ = zeroGVoltage_;
    zeroGVoltageY_ = zeroGVoltage_;
    zeroGVoltageZ_ = zeroGVoltage_;
    sensitivityVPerG_ = (NuraConstants::ADXL377::kDatasheetSensitivityMvPerG *
                         (referenceVoltage_ / NuraConstants::ADXL377::kDatasheetReferenceVoltage)) *
                        0.001f;
}

bool ADXL377HAL::calibrateStationary(uint16_t sampleCount,
                                     uint16_t sampleDelayMs,
                                     float expectedAccelXG,
                                     float expectedAccelYG,
                                     float expectedAccelZG)
{
    if (!initialized_ || sampleCount == 0U)
    {
        return false;
    }

    double voltageXSum = 0.0;
    double voltageYSum = 0.0;
    double voltageZSum = 0.0;

    for (uint16_t i = 0U; i < sampleCount; ++i)
    {
        const uint16_t rawX = static_cast<uint16_t>(analogRead(xPin_));
        const uint16_t rawY = static_cast<uint16_t>(analogRead(yPin_));
        const uint16_t rawZ = static_cast<uint16_t>(analogRead(zPin_));

        voltageXSum += rawToVoltage(rawX);
        voltageYSum += rawToVoltage(rawY);
        voltageZSum += rawToVoltage(rawZ);

        if (sampleDelayMs > 0U)
        {
            delay(sampleDelayMs);
        }
    }

    const double sampleCountD = static_cast<double>(sampleCount);
    zeroGVoltageX_ = static_cast<float>((voltageXSum / sampleCountD) -
                                        (expectedAccelXG * sensitivityVPerG_));
    zeroGVoltageY_ = static_cast<float>((voltageYSum / sampleCountD) -
                                        (expectedAccelYG * sensitivityVPerG_));
    zeroGVoltageZ_ = static_cast<float>((voltageZSum / sampleCountD) -
                                        (expectedAccelZG * sensitivityVPerG_));

    return isfinite(zeroGVoltageX_) &&
           isfinite(zeroGVoltageY_) &&
           isfinite(zeroGVoltageZ_);
}

Adxl377Calibration ADXL377HAL::calibration() const
{
    Adxl377Calibration calibration;
    calibration.zeroGVoltageX = zeroGVoltageX_;
    calibration.zeroGVoltageY = zeroGVoltageY_;
    calibration.zeroGVoltageZ = zeroGVoltageZ_;
    calibration.sensitivityMvPerG = sensitivityVPerG_ * 1000.0f;
    return calibration;
}

float ADXL377HAL::rawToVoltage(uint16_t raw) const
{
    return (static_cast<float>(raw) / static_cast<float>(adcMax_)) * referenceVoltage_;
}

float ADXL377HAL::voltageToG(float voltage, float zeroGVoltage) const
{
    return (voltage - zeroGVoltage) / sensitivityVPerG_;
}
