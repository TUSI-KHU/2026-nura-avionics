#include "ms5611_hal.h"

#include <Arduino.h>
#include <math.h>

namespace
{
    constexpr float kMinDatasheetPressurePa = 1000.0f;
    constexpr float kMaxDatasheetPressurePa = 120000.0f;
    constexpr float kStandardAtmosphereMeters = 44330.0f;
    constexpr float kPressureExponent = 0.19029495f;
    constexpr float kInversePressureExponent = 5.255f;
}

bool MS5611HAL::begin(uint8_t i2cAddress,
                      TwoWire &wire,
                      osr_t oversampling,
                      float seaLevelPressureMbar)
{
    initialized_ = false;
    promCrcValid_ = false;
    groundBaselineValid_ = false;
    sensor_ = MS5611(i2cAddress, &wire);
    oversampling_ = oversampling;
    seaLevelPressureMbar_ = seaLevelPressureMbar;

    if (!sensor_.begin())
    {
        return false;
    }

    sensor_.setOversampling(oversampling_);
    if (!promLooksValid())
    {
        return false;
    }

    initialized_ = true;
    return true;
}

bool MS5611HAL::read(Ms5611Reading &out, uint32_t nowMs)
{
    if (!initialized_)
    {
        return false;
    }

    const int result = sensor_.read();
    if (result != MS5611_READ_OK)
    {
        return false;
    }

    const float pressurePa = sensor_.getPressurePascal();
    const float temperatureC = sensor_.getTemperature();
    if (!validPressure(pressurePa) || !isfinite(temperatureC))
    {
        return false;
    }

    out.pressurePa = pressurePa;
    out.pressureMbar = pressurePa * 0.01f;
    out.temperatureC = temperatureC;
    out.altitudeM = sensor_.getAltitude(seaLevelPressureMbar_);
    out.relativeAltitudeM = groundBaselineValid_ ? pressureToAltitudeM(pressurePa, groundPressurePa_) : 0.0f;
    out.sampleMs = nowMs;
    out.deviceId = sensor_.getDeviceID();

    return true;
}

uint16_t MS5611HAL::promWord(uint8_t index)
{
    if (!initialized_ || index > 7U)
    {
        return 0U;
    }
    return prom_[index];
}

uint16_t MS5611HAL::promCrc()
{
    return prom_[7] & 0x0FU;
}

bool MS5611HAL::promCrcValid() const
{
    return promCrcValid_;
}

void MS5611HAL::setSeaLevelPressureMbar(float seaLevelPressureMbar)
{
    if (isfinite(seaLevelPressureMbar) && seaLevelPressureMbar > 0.0f)
    {
        seaLevelPressureMbar_ = seaLevelPressureMbar;
    }
}

bool MS5611HAL::calibrateGroundBaseline(uint16_t sampleCount,
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
        Ms5611Reading sample;
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

    seaLevelPressureMbar_ = pressureToSeaLevelPressureMbar(groundPressurePa_ * 0.01f,
                                                           knownGroundAltitudeM);
    groundBaselineValid_ = isfinite(seaLevelPressureMbar_) && seaLevelPressureMbar_ > 0.0f;
    return groundBaselineValid_;
}

void MS5611HAL::clearGroundBaseline()
{
    groundPressurePa_ = 0.0f;
    groundBaselineValid_ = false;
}

bool MS5611HAL::groundBaselineValid() const
{
    return groundBaselineValid_;
}

bool MS5611HAL::promLooksValid()
{
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        prom_[i] = sensor_.getProm(i);
    }

    for (uint8_t i = 1U; i <= 6U; ++i)
    {
        if (prom_[i] == 0U)
        {
            return false;
        }
    }

    promCrcValid_ = calculatePromCrc4(prom_) == (prom_[7] & 0x0FU);
    return promCrcValid_;
}

uint8_t MS5611HAL::calculatePromCrc4(const uint16_t prom[8])
{
    uint16_t promCopy[8];
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        promCopy[i] = prom[i];
    }

    promCopy[7] &= 0xFF00U;
    uint16_t remainder = 0U;
    for (uint8_t i = 0U; i < 16U; ++i)
    {
        if ((i & 1U) != 0U)
        {
            remainder ^= promCopy[i >> 1U] & 0x00FFU;
        }
        else
        {
            remainder ^= promCopy[i >> 1U] >> 8U;
        }

        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((remainder & 0x8000U) != 0U)
            {
                remainder = static_cast<uint16_t>((remainder << 1U) ^ 0x3000U);
            }
            else
            {
                remainder = static_cast<uint16_t>(remainder << 1U);
            }
        }
    }

    return static_cast<uint8_t>((remainder >> 12U) & 0x0FU);
}

bool MS5611HAL::validPressure(float pressurePa)
{
    return isfinite(pressurePa) &&
           pressurePa >= kMinDatasheetPressurePa &&
           pressurePa <= kMaxDatasheetPressurePa;
}

float MS5611HAL::pressureToAltitudeM(float pressurePa, float referencePressurePa)
{
    if (!validPressure(pressurePa) || !validPressure(referencePressurePa))
    {
        return 0.0f;
    }

    return kStandardAtmosphereMeters * (1.0f - powf(pressurePa / referencePressurePa, kPressureExponent));
}

float MS5611HAL::pressureToSeaLevelPressureMbar(float pressureMbar, float altitudeM)
{
    if (!isfinite(pressureMbar) || pressureMbar <= 0.0f || !isfinite(altitudeM))
    {
        return 0.0f;
    }

    const float altitudeScale = 1.0f - (altitudeM / kStandardAtmosphereMeters);
    if (altitudeScale <= 0.0f)
    {
        return 0.0f;
    }

    return pressureMbar / powf(altitudeScale, kInversePressureExponent);
}
