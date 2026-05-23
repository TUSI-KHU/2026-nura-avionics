#include "mock_flight_data_hal.h"

#include <math.h>

namespace
{
constexpr float kGravityMps2 = 9.80665f;
constexpr float kPressureSeaLevelPa = 101325.0f;
constexpr float kStandardAtmosphereMeters = 44330.0f;
constexpr float kPressureExponent = 0.19029495f;
constexpr float kAltitudeFilterAlpha = 0.35f;
constexpr uint8_t kAltitudeMedianWindow = 3U;
constexpr uint32_t kMockBarometerPeriodMs = 50UL;

#if defined(NURA_MOCK_SCENARIO_ID)
constexpr uint8_t kConfiguredScenarioId = static_cast<uint8_t>(NURA_MOCK_SCENARIO_ID);
#else
constexpr uint8_t kConfiguredScenarioId = 0U;
#endif

MockFlightScenarioId configuredScenario()
{
    switch (kConfiguredScenarioId)
    {
    case 1U:
        return MockFlightScenarioId::LOW_APOGEE;
    case 2U:
        return MockFlightScenarioId::BARO_NOISE;
    case 3U:
        return MockFlightScenarioId::BARO_GUST;
    case 4U:
        return MockFlightScenarioId::BARO_DROPOUT;
    case 5U:
        return MockFlightScenarioId::PAD_FALSE_ACCEL;
    default:
        return MockFlightScenarioId::NOMINAL;
    }
}

float median3(float values[], uint8_t count)
{
    for (uint8_t i = 1U; i < count; ++i)
    {
        const float value = values[i];
        uint8_t j = i;
        while (j > 0U && values[j - 1U] > value)
        {
            values[j] = values[j - 1U];
            --j;
        }
        values[j] = value;
    }
    return values[count / 2U];
}
} // namespace

bool MockFlightDataHAL::begin()
{
    scenario_ = configuredScenario();
    altitudeWindowHead_ = 0U;
    altitudeWindowCount_ = 0U;
    filteredAltitudeM_ = 0.0f;
    filterReady_ = false;
    return true;
}

bool MockFlightDataHAL::read(MockFlightDataReading &out, uint32_t nowMs)
{
    const ScenarioParams params = paramsFor(scenario_);
    const float timeS = static_cast<float>(nowMs) / 1000.0f;

    float altitudeM = altitudeFor(params, timeS);
    if (scenario_ == MockFlightScenarioId::BARO_NOISE)
    {
        altitudeM += deterministicNoiseM(nowMs, 0.8f);
    }
    else if (scenario_ == MockFlightScenarioId::BARO_GUST)
    {
        altitudeM += deterministicNoiseM(nowMs, 0.6f);
        const float pulseDtS = timeS - (params.apogeeTimeS - 1.2f);
        altitudeM += -6.0f * expf(-0.5f * (pulseDtS * pulseDtS) / (0.16f * 0.16f));
    }

    const bool falseAccelWindow = scenario_ == MockFlightScenarioId::PAD_FALSE_ACCEL && nowMs >= 300UL && nowMs < 330UL;
    const bool motorBurnWindow = timeS >= params.launchTimeS && timeS < params.burnoutTimeS;
    const bool coastWindow = timeS >= params.burnoutTimeS;
    const float highAccelNormG = falseAccelWindow ? 2.5f : (motorBurnWindow ? 3.8f : (coastWindow ? 0.55f : 0.15f));

    out.highAccelXG = 0.02f;
    out.highAccelYG = 0.01f;
    out.highAccelZG = highAccelNormG;
    out.accelXMps2 = out.highAccelXG * kGravityMps2;
    out.accelYMps2 = out.highAccelYG * kGravityMps2;
    out.accelZMps2 = out.highAccelZG * kGravityMps2;
    out.gyroXDps = deterministicNoiseM(nowMs + 17UL, 4.0f);
    out.gyroYDps = deterministicNoiseM(nowMs + 31UL, 4.0f);
    out.gyroZDps = deterministicNoiseM(nowMs + 47UL, 4.0f);
    out.rawAltitudeM = altitudeM;
    out.barometerValid = !(scenario_ == MockFlightScenarioId::BARO_DROPOUT && nowMs > 8200UL && nowMs < 8800UL);
    out.barometerUpdated = out.barometerValid && ((nowMs % kMockBarometerPeriodMs) == 0UL);
    out.filteredAltitudeM = out.barometerUpdated ? filteredAltitude(altitudeM) : filteredAltitudeM_;
    out.pressurePa = pressureForAltitude(out.rawAltitudeM);
    out.latitudeDeg = 37.1234567 + (static_cast<double>(timeS) * 0.000001);
    out.longitudeDeg = 127.1234567 + (static_cast<double>(timeS) * 0.000001);
    out.altitudeM = 50.0 + static_cast<double>(out.rawAltitudeM);
    out.speedMps = timeS < params.apogeeTimeS ? 35.0 : params.descentRateMps;
    out.courseDeg = 85.2;
    out.hdop = 1.2;
    out.satellites = 9U;
    out.batteryMv = static_cast<uint16_t>(12000U - static_cast<uint16_t>((nowMs / 10000UL) % 500UL));
    out.sampleMs = nowMs;
    return true;
}

void MockFlightDataHAL::setScenario(MockFlightScenarioId scenario)
{
    scenario_ = scenario;
    altitudeWindowHead_ = 0U;
    altitudeWindowCount_ = 0U;
    filteredAltitudeM_ = 0.0f;
    filterReady_ = false;
}

MockFlightScenarioId MockFlightDataHAL::scenario() const
{
    return scenario_;
}

const char *MockFlightDataHAL::scenarioName() const
{
    return nameFor(scenario_);
}

MockFlightDataHAL::ScenarioParams MockFlightDataHAL::paramsFor(MockFlightScenarioId scenario)
{
    switch (scenario)
    {
    case MockFlightScenarioId::LOW_APOGEE:
        return {1.0f, 2.45f, 9.8f, 250.0f, 22.0f};
    case MockFlightScenarioId::BARO_GUST:
        return {1.0f, 2.55f, 10.5f, 400.0f, 25.0f};
    case MockFlightScenarioId::BARO_DROPOUT:
        return {1.0f, 2.55f, 10.5f, 400.0f, 25.0f};
    case MockFlightScenarioId::PAD_FALSE_ACCEL:
        return {1.0f, 2.55f, 10.5f, 400.0f, 25.0f};
    case MockFlightScenarioId::BARO_NOISE:
        return {1.0f, 2.55f, 10.5f, 400.0f, 25.0f};
    case MockFlightScenarioId::NOMINAL:
    default:
        return {1.0f, 2.55f, 10.5f, 400.0f, 25.0f};
    }
}

const char *MockFlightDataHAL::nameFor(MockFlightScenarioId scenario)
{
    switch (scenario)
    {
    case MockFlightScenarioId::LOW_APOGEE:
        return "low_apogee";
    case MockFlightScenarioId::BARO_NOISE:
        return "baro_noise";
    case MockFlightScenarioId::BARO_GUST:
        return "baro_gust";
    case MockFlightScenarioId::BARO_DROPOUT:
        return "baro_dropout";
    case MockFlightScenarioId::PAD_FALSE_ACCEL:
        return "pad_false_accel";
    case MockFlightScenarioId::NOMINAL:
    default:
        return "nominal";
    }
}

float MockFlightDataHAL::altitudeFor(const ScenarioParams &params, float timeS)
{
    if (timeS <= params.launchTimeS)
    {
        return 0.0f;
    }

    if (timeS <= params.apogeeTimeS)
    {
        const float durationS = params.apogeeTimeS - params.launchTimeS;
        const float remaining = (params.apogeeTimeS - timeS) / durationS;
        return params.apogeeAltitudeM * (1.0f - (remaining * remaining));
    }

    const float descentAltitude = params.apogeeAltitudeM - ((timeS - params.apogeeTimeS) * params.descentRateMps);
    return descentAltitude > 0.0f ? descentAltitude : 0.0f;
}

float MockFlightDataHAL::pressureForAltitude(float altitudeM)
{
    const float clampedAltitudeM = altitudeM < 0.0f ? 0.0f : altitudeM;
    const float ratio = 1.0f - (clampedAltitudeM / kStandardAtmosphereMeters);
    return kPressureSeaLevelPa * powf(ratio, 1.0f / kPressureExponent);
}

float MockFlightDataHAL::deterministicNoiseM(uint32_t nowMs, float amplitudeM)
{
    const float slow = sinf(static_cast<float>(nowMs) * 0.017f);
    const float fast = sinf(static_cast<float>(nowMs + 113UL) * 0.071f);
    return amplitudeM * ((0.65f * slow) + (0.35f * fast));
}

float MockFlightDataHAL::filteredAltitude(float rawAltitudeM)
{
    altitudeWindowM_[altitudeWindowHead_] = rawAltitudeM;
    altitudeWindowHead_ = static_cast<uint8_t>((altitudeWindowHead_ + 1U) % kAltitudeMedianWindow);
    if (altitudeWindowCount_ < kAltitudeMedianWindow)
    {
        ++altitudeWindowCount_;
    }

    float windowCopy[kAltitudeMedianWindow] = {0.0f, 0.0f, 0.0f};
    for (uint8_t i = 0U; i < altitudeWindowCount_; ++i)
    {
        windowCopy[i] = altitudeWindowM_[i];
    }

    const float medianAltitudeM = median3(windowCopy, altitudeWindowCount_);
    if (!filterReady_)
    {
        filteredAltitudeM_ = medianAltitudeM;
        filterReady_ = true;
    }
    else
    {
        filteredAltitudeM_ += kAltitudeFilterAlpha * (medianAltitudeM - filteredAltitudeM_);
    }
    return filteredAltitudeM_;
}
