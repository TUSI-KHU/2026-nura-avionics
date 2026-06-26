#include "mock_flight_data_hal.h"

#include <math.h>

namespace
{
#if defined(NURA_MOCK_SCENARIO_ID)
constexpr uint8_t kConfiguredScenarioId = static_cast<uint8_t>(NURA_MOCK_SCENARIO_ID);
#else
constexpr uint8_t kConfiguredScenarioId = NuraConstants::Mock::kDefaultScenarioId;
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
        altitudeM += deterministicNoiseM(nowMs, NuraConstants::Mock::kNoiseAmplitudeM);
    }
    else if (scenario_ == MockFlightScenarioId::BARO_GUST)
    {
        altitudeM += deterministicNoiseM(nowMs, NuraConstants::Mock::kGustNoiseAmplitudeM);
        const float pulseDtS = timeS - (params.apogeeTimeS - NuraConstants::Mock::kGustPulseLeadS);
        altitudeM += NuraConstants::Mock::kGustPulseAmplitudeM *
                     expf(-0.5f * (pulseDtS * pulseDtS) /
                          (NuraConstants::Mock::kGustPulseSigmaS * NuraConstants::Mock::kGustPulseSigmaS));
    }

    const bool falseAccelWindow = scenario_ == MockFlightScenarioId::PAD_FALSE_ACCEL &&
                                  nowMs >= NuraConstants::Mock::kPadFalseAccelStartMs &&
                                  nowMs < NuraConstants::Mock::kPadFalseAccelEndMs;
    const bool motorBurnWindow = timeS >= params.launchTimeS && timeS < params.burnoutTimeS;
    const bool coastWindow = timeS >= params.burnoutTimeS;
    const float highAccelNormG = falseAccelWindow ? NuraConstants::Mock::kPadFalseAccelG
                                                  : (motorBurnWindow ? NuraConstants::Mock::kMotorBurnAccelG
                                                                     : (coastWindow ? NuraConstants::Mock::kCoastAccelG
                                                                                    : NuraConstants::Mock::kPrelaunchAccelG));

    out.highAccelXG = NuraConstants::Mock::kAccelXG;
    out.highAccelYG = NuraConstants::Mock::kAccelYG;
    out.highAccelZG = highAccelNormG;
    out.accelXMps2 = out.highAccelXG * NuraConstants::Physics::kGravityMps2;
    out.accelYMps2 = out.highAccelYG * NuraConstants::Physics::kGravityMps2;
    out.accelZMps2 = out.highAccelZG * NuraConstants::Physics::kGravityMps2;
    out.gyroXDps = deterministicNoiseM(nowMs + 17UL, 4.0f);
    out.gyroYDps = deterministicNoiseM(nowMs + 31UL, 4.0f);
    out.gyroZDps = deterministicNoiseM(nowMs + 47UL, 4.0f);
    out.attitudeValid = true;
    out.rollDeg = 0.0f;
    out.pitchDeg = timeS > params.apogeeTimeS ? 80.0f : 0.0f;
    out.yawDeg = deterministicNoiseM(nowMs + 61UL, 2.0f);
    out.tiltValid = true;
    out.tiltAngleDeg = timeS > params.apogeeTimeS ? 80.0f : 0.0f;
    out.rawAltitudeM = altitudeM;
    out.barometerValid = !(scenario_ == MockFlightScenarioId::BARO_DROPOUT &&
                           nowMs > NuraConstants::Mock::kBarometerDropoutStartMs &&
                           nowMs < NuraConstants::Mock::kBarometerDropoutEndMs);
    out.barometerUpdated = out.barometerValid && ((nowMs % NuraConstants::Mock::kBarometerPeriodMs) == 0UL);
    out.filteredAltitudeM = out.barometerUpdated ? filteredAltitude(altitudeM) : filteredAltitudeM_;
    out.pressurePa = pressureForAltitude(out.rawAltitudeM);
    out.latitudeDeg = NuraConstants::Mock::kBaseLatitudeDeg +
                      (static_cast<double>(timeS) * NuraConstants::Mock::kGpsDriftDegPerS);
    out.longitudeDeg = NuraConstants::Mock::kBaseLongitudeDeg +
                       (static_cast<double>(timeS) * NuraConstants::Mock::kGpsDriftDegPerS);
    out.altitudeM = NuraConstants::Mock::kGpsBaseAltitudeM + static_cast<double>(out.rawAltitudeM);
    out.speedMps = timeS < params.apogeeTimeS ? NuraConstants::Mock::kAscentSpeedMps : params.descentRateMps;
    out.courseDeg = NuraConstants::Mock::kCourseDeg;
    out.hdop = NuraConstants::Mock::kHdop;
    out.satellites = NuraConstants::Mock::kSatellites;
    out.batteryMv = static_cast<uint16_t>(NuraConstants::Mock::kBatteryBaseMv -
                                          static_cast<uint16_t>((nowMs / NuraConstants::Mock::kBatterySawtoothPeriodMs) %
                                                                NuraConstants::Mock::kBatterySawtoothMv));
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
        return NuraConstants::Mock::kLowApogeeProfile;
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
        return NuraConstants::Mock::kNominalProfile;
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
    const float ratio = 1.0f - (clampedAltitudeM / NuraConstants::Atmosphere::kStandardAtmosphereMeters);
    return NuraConstants::Atmosphere::kSeaLevelPressurePa *
           powf(ratio, 1.0f / NuraConstants::Atmosphere::kPressureExponent);
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
    altitudeWindowHead_ = static_cast<uint8_t>((altitudeWindowHead_ + 1U) %
                                               NuraConstants::Sensors::kBarometerMedianWindowSamples);
    if (altitudeWindowCount_ < NuraConstants::Sensors::kBarometerMedianWindowSamples)
    {
        ++altitudeWindowCount_;
    }

    float windowCopy[NuraConstants::Sensors::kBarometerMedianWindowSamples] = {0.0f, 0.0f, 0.0f};
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
        filteredAltitudeM_ += NuraConstants::Sensors::kBarometerAltitudeLpfAlpha *
                              (medianAltitudeM - filteredAltitudeM_);
    }
    return filteredAltitudeM_;
}
