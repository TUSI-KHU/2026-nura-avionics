#include "mpl3115a2_hal.h"

#include <Arduino.h>
#include <math.h>

#include "nura_constants.h"

bool MPL3115A2HAL::begin(TwoWire &wire,
                         uint16_t conversionTimeoutMs,
                         float seaLevelPressureHpa)
{
    initialized_ = false;
    groundBaselineValid_ = false;
    conversionPending_ = false;
    conversionTimeoutMs_ = conversionTimeoutMs;
    seaLevelPressureHpa_ = seaLevelPressureHpa;

    if (!sensor_.begin(&wire))
    {
        return false;
    }

    sensor_.setMode(MPL3115A2_BAROMETER);
    sensor_.write8(MPL3115A2_CTRL_REG1, NuraConstants::MPL3115A2::kFastBarometerCtrlReg1);
    sensor_.setSeaPressure(seaLevelPressureHpa_);

    initialized_ = true;
    return true;
}

bool MPL3115A2HAL::read(Mpl3115a2Reading &out, uint32_t nowMs)
{
    if (!initialized_)
    {
        return false;
    }

    sensor_.setMode(MPL3115A2_BAROMETER);
    sensor_.startOneShot();
    if (!waitForConversion())
    {
        return false;
    }

    const float pressureHpa = sensor_.getLastConversionResults(MPL3115A2_PRESSURE);
    const float pressurePa = pressureHpa * NuraConstants::MPL3115A2::kHpaToPa;
    const float temperatureC = sensor_.getLastConversionResults(MPL3115A2_TEMPERATURE);
    if (!validPressure(pressurePa) || !isfinite(temperatureC))
    {
        return false;
    }

    out.pressurePa = pressurePa;
    out.pressureHpa = pressureHpa;
    out.temperatureC = temperatureC;
    out.relativeAltitudeM = groundBaselineValid_ ? pressureToAltitudeM(pressurePa, groundPressurePa_) : 0.0f;
    out.sampleMs = nowMs;

    return true;
}

Mpl3115a2PollResult MPL3115A2HAL::poll(Mpl3115a2Reading &out, uint32_t nowMs)
{
    out = Mpl3115a2Reading{};
    if (!initialized_)
    {
        return Mpl3115a2PollResult::ERROR;
    }

    if (!conversionPending_)
    {
        startConversion(nowMs);
        return Mpl3115a2PollResult::PENDING;
    }

    if (sensor_.conversionComplete())
    {
        conversionPending_ = false;
        if (!readConversionResult(out, nowMs))
        {
            startConversion(nowMs);
            return Mpl3115a2PollResult::ERROR;
        }
        startConversion(nowMs);
        return Mpl3115a2PollResult::READY;
    }

    if ((nowMs - conversionStartMs_) > conversionTimeoutMs_)
    {
        conversionPending_ = false;
        return Mpl3115a2PollResult::ERROR;
    }

    return Mpl3115a2PollResult::PENDING;
}

void MPL3115A2HAL::setSeaLevelPressureHpa(float seaLevelPressureHpa)
{
    if (!isfinite(seaLevelPressureHpa) || seaLevelPressureHpa <= 0.0f)
    {
        return;
    }

    seaLevelPressureHpa_ = seaLevelPressureHpa;
    if (initialized_)
    {
        sensor_.setSeaPressure(seaLevelPressureHpa_);
    }
}

bool MPL3115A2HAL::calibrateGroundBaseline(uint16_t sampleCount,
                                           uint16_t sampleDelayMs,
                                           float knownGroundAltitudeM)
{
    if (!initialized_ || sampleCount == 0U)
    {
        return false;
    }

    double pressurePaSum = 0.0;
    for (uint16_t i = 0U; i < sampleCount; ++i)
    {
        Mpl3115a2Reading sample;
        if (!read(sample, millis()))
        {
            return false;
        }

        pressurePaSum += sample.pressurePa;
        if (sampleDelayMs > 0U)
        {
            delay(sampleDelayMs);
        }
    }

    groundPressurePa_ = static_cast<float>(pressurePaSum / static_cast<double>(sampleCount));
    if (!validPressure(groundPressurePa_))
    {
        groundBaselineValid_ = false;
        return false;
    }

    seaLevelPressureHpa_ = pressureToSeaLevelPressureHpa(groundPressurePa_ / NuraConstants::MPL3115A2::kHpaToPa,
                                                         knownGroundAltitudeM);
    groundBaselineValid_ = isfinite(seaLevelPressureHpa_) && seaLevelPressureHpa_ > 0.0f;
    if (groundBaselineValid_)
    {
        sensor_.setSeaPressure(seaLevelPressureHpa_);
    }
    return groundBaselineValid_;
}

void MPL3115A2HAL::clearGroundBaseline()
{
    groundPressurePa_ = 0.0f;
    groundBaselineValid_ = false;
}

bool MPL3115A2HAL::groundBaselineValid() const
{
    return groundBaselineValid_;
}

void MPL3115A2HAL::startConversion(uint32_t nowMs)
{
    sensor_.setMode(MPL3115A2_BAROMETER);
    sensor_.startOneShot();
    conversionStartMs_ = nowMs;
    conversionPending_ = true;
}

bool MPL3115A2HAL::readConversionResult(Mpl3115a2Reading &out, uint32_t nowMs)
{
    const float pressureHpa = sensor_.getLastConversionResults(MPL3115A2_PRESSURE);
    const float pressurePa = pressureHpa * NuraConstants::MPL3115A2::kHpaToPa;
    const float temperatureC = sensor_.getLastConversionResults(MPL3115A2_TEMPERATURE);
    if (!validPressure(pressurePa) || !isfinite(temperatureC))
    {
        return false;
    }

    out.pressurePa = pressurePa;
    out.pressureHpa = pressureHpa;
    out.temperatureC = temperatureC;
    out.relativeAltitudeM = groundBaselineValid_ ? pressureToAltitudeM(pressurePa, groundPressurePa_) : 0.0f;
    out.sampleMs = nowMs;
    return true;
}

bool MPL3115A2HAL::waitForConversion()
{
    const uint32_t startMs = millis();
    while ((millis() - startMs) <= conversionTimeoutMs_)
    {
        if (sensor_.conversionComplete())
        {
            return true;
        }
        delay(1);
    }
    return false;
}

bool MPL3115A2HAL::validPressure(float pressurePa)
{
    return isfinite(pressurePa) &&
           pressurePa >= NuraConstants::MPL3115A2::kMinDatasheetPressurePa &&
           pressurePa <= NuraConstants::MPL3115A2::kMaxDatasheetPressurePa;
}

float MPL3115A2HAL::pressureToAltitudeM(float pressurePa, float referencePressurePa)
{
    if (!validPressure(pressurePa) || !validPressure(referencePressurePa))
    {
        return 0.0f;
    }

    return NuraConstants::Atmosphere::kStandardAtmosphereMeters *
           (1.0f - powf(pressurePa / referencePressurePa, NuraConstants::Atmosphere::kPressureExponent));
}

float MPL3115A2HAL::pressureToSeaLevelPressureHpa(float pressureHpa, float altitudeM)
{
    if (!isfinite(pressureHpa) || pressureHpa <= 0.0f || !isfinite(altitudeM))
    {
        return 0.0f;
    }

    const float altitudeScale = 1.0f - (altitudeM / NuraConstants::Atmosphere::kStandardAtmosphereMeters);
    if (altitudeScale <= 0.0f)
    {
        return 0.0f;
    }

    return pressureHpa / powf(altitudeScale, NuraConstants::Atmosphere::kInversePressureExponent);
}
