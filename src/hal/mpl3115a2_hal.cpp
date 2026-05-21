#include "mpl3115a2_hal.h"

#include <Arduino.h>
#include <math.h>

namespace
{
    constexpr float kHpaToPa = 100.0f;
    constexpr float kMinDatasheetPressurePa = 20000.0f;
    constexpr float kMaxDatasheetPressurePa = 110000.0f;
    constexpr float kStandardAtmosphereMeters = 44330.0f;
    constexpr float kPressureExponent = 0.19029495f;
    constexpr float kInversePressureExponent = 5.255f;
}

bool MPL3115A2HAL::begin(TwoWire &wire,
                         uint16_t conversionTimeoutMs,
                         float seaLevelPressureHpa)
{
    initialized_ = false;
    groundBaselineValid_ = false;
    conversionTimeoutMs_ = conversionTimeoutMs;
    seaLevelPressureHpa_ = seaLevelPressureHpa;

    if (!sensor_.begin(&wire))
    {
        return false;
    }

    sensor_.setMode(MPL3115A2_BAROMETER);
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
    const float pressurePa = pressureHpa * kHpaToPa;
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

    seaLevelPressureHpa_ = pressureToSeaLevelPressureHpa(groundPressurePa_ / kHpaToPa,
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
           pressurePa >= kMinDatasheetPressurePa &&
           pressurePa <= kMaxDatasheetPressurePa;
}

float MPL3115A2HAL::pressureToAltitudeM(float pressurePa, float referencePressurePa)
{
    if (!validPressure(pressurePa) || !validPressure(referencePressurePa))
    {
        return 0.0f;
    }

    return kStandardAtmosphereMeters * (1.0f - powf(pressurePa / referencePressurePa, kPressureExponent));
}

float MPL3115A2HAL::pressureT   oSeaLevelPressureHpa(float pressureHpa, float altitudeM)
{
    if (!isfinite(pressureHpa) || pressureHpa <= 0.0f || !isfinite(altitudeM))
    {
        return 0.0f;
    }

    const float altitudeScale = 1.0f - (altitudeM / kStandardAtmosphereMeters);
    if (altitudeScale <= 0.0f)
    {
        return 0.0f;
    }

    return pressureHpa / powf(altitudeScale, kInversePressureExponent);
}
