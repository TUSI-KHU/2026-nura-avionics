#include <cmath>
#include <cstdio>
#include <cstring>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "hal/mock_flight_data_hal.h"
#include "missions/fsm_task.h"
#include "missions/mission_constants.h"

namespace
{
constexpr float kGravityMps2 = 9.80665f;
constexpr uint32_t kTickMs = 10U;
constexpr uint32_t kBaroPeriodMs = 50U;

struct FakeConfig : public IAppConfig
{
    unsigned long serialBaudRate() const override { return 115200UL; }
    uint8_t statusIndicatorPin() const override { return 13U; }
    uint16_t faultBlinkIntervalMs() const override { return 1000U; }
    uint8_t imuCsPin() const override { return 6U; }
    uint8_t imuReadFailureThreshold() const override { return 3U; }
    uint8_t imuMaxRecoveryAttempts() const override { return 5U; }
    uint32_t imuRecoveryIntervalMs() const override { return 1000U; }
    uint32_t imuTaskPeriodMs() const override { return 10U; }
    uint32_t magnetometerTaskPeriodMs() const override { return 100U; }
    uint32_t barometerTaskPeriodMs() const override { return 50U; }
    uint32_t barometerRecoveryIntervalMs() const override { return 1000U; }
    uint32_t gnssTaskPeriodMs() const override { return 50U; }
    uint16_t gnssPollByteBudget() const override { return 128U; }
    uint32_t gnssMaxFixAgeMs() const override { return 2000U; }
    uint32_t watchdogTaskPeriodMs() const override { return 50U; }
    uint32_t flightStateTaskPeriodMs() const override { return kTickMs; }
    uint32_t loggerTaskPeriodMs() const override { return 20U; }
    uint32_t telemetryTaskPeriodMs() const override { return 20U; }
    uint32_t telemetryFastPeriodMs() const override { return 200U; }
    uint32_t telemetryGpsPeriodMs() const override { return 1000U; }
    uint32_t telemetrySensorFreshMs() const override { return 1500U; }
    uint8_t loggerDrainBudget() const override { return 4U; }
    uint8_t loggerOutputFailThreshold() const override { return 3U; }
    long loraFrequencyHz() const override { return 920900000L; }
    uint32_t loraSpiFrequencyHz() const override { return 8000000UL; }
    int loraTxPowerDbm() const override { return 17; }
    int loraSpreadingFactor() const override { return 7; }
    long loraSignalBandwidthHz() const override { return 125000L; }
    int loraCodingRateDenominator() const override { return 5; }
    long loraPreambleLength() const override { return 8L; }
    int loraSyncWord() const override { return 0x12; }
    uint8_t loraInitAttempts() const override { return 1U; }
    uint8_t loraSpiMode() const override { return 0U; }
    bool loraProbeSpiMode() const override { return false; }
};

struct FakePanicHandler : public IPanicHandler
{
    bool panicked = false;
    void panic() override { panicked = true; }
};

struct BaroFilter
{
    float window[3] = {0.0f, 0.0f, 0.0f};
    uint8_t head = 0U;
    uint8_t count = 0U;
    float filtered = 0.0f;
    bool ready = false;

    float push(float raw)
    {
        window[head] = raw;
        head = static_cast<uint8_t>((head + 1U) % 3U);
        if (count < 3U)
        {
            ++count;
        }

        float values[3] = {window[0], window[1], window[2]};
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

        const float median = values[count / 2U];
        if (!ready)
        {
            filtered = median;
            ready = true;
        }
        else
        {
            filtered += 0.35f * (median - filtered);
        }
        return filtered;
    }
};

struct Scenario
{
    const char *name;
    float launchS;
    float burnoutS;
    float apogeeS;
    float apogeeM;
    float descentRateMps;
    float noiseM;
    bool gustPulse;
    bool dropout;
    bool falseAccel;
};

struct ReplayResult
{
    uint32_t launchMs = 0U;
    uint32_t coastMs = 0U;
    uint32_t apogeeMs = 0U;
    uint32_t drogueMs = 0U;
    uint32_t deployMs = 0U;
    uint32_t groundMs = 0U;
    float trueAltitudeAtApogeeDecisionM = 0.0f;
    bool launchedBeforeRealLaunch = false;
};

float deterministicNoise(uint32_t nowMs, float amplitudeM)
{
    const float slow = std::sin(static_cast<float>(nowMs) * 0.017f);
    const float fast = std::sin(static_cast<float>(nowMs + 113UL) * 0.071f);
    return amplitudeM * ((0.65f * slow) + (0.35f * fast));
}

float trueAltitude(const Scenario &scenario, float timeS)
{
    if (timeS <= scenario.launchS)
    {
        return 0.0f;
    }

    if (timeS <= scenario.apogeeS)
    {
        const float durationS = scenario.apogeeS - scenario.launchS;
        const float remaining = (scenario.apogeeS - timeS) / durationS;
        return scenario.apogeeM * (1.0f - (remaining * remaining));
    }

    const float altitude = scenario.apogeeM - ((timeS - scenario.apogeeS) * scenario.descentRateMps);
    return altitude > 0.0f ? altitude : 0.0f;
}

float measuredAltitude(const Scenario &scenario, uint32_t nowMs)
{
    const float timeS = static_cast<float>(nowMs) / 1000.0f;
    float altitude = trueAltitude(scenario, timeS);
    altitude += deterministicNoise(nowMs, scenario.noiseM);
    if (scenario.gustPulse)
    {
        const float pulseDtS = timeS - (scenario.apogeeS - 1.2f);
        altitude += -6.0f * std::exp(-0.5f * (pulseDtS * pulseDtS) / (0.16f * 0.16f));
    }
    return altitude;
}

bool baroAvailable(const Scenario &scenario, uint32_t nowMs)
{
    if (!scenario.dropout)
    {
        return true;
    }
    return !(nowMs > 8200UL && nowMs < 8800UL);
}

float highGNorm(const Scenario &scenario, uint32_t nowMs)
{
    const float timeS = static_cast<float>(nowMs) / 1000.0f;
    if (scenario.falseAccel && nowMs >= 300UL && nowMs < 330UL)
    {
        return 2.5f;
    }
    if (timeS >= scenario.launchS && timeS < scenario.burnoutS)
    {
        return 3.8f;
    }
    if (timeS >= scenario.burnoutS)
    {
        return 0.55f;
    }
    return 0.15f;
}

void publishHighG(HighGImuState &highG, const Scenario &scenario, uint32_t nowMs)
{
    const float normG = highGNorm(scenario, nowMs);
    highG.accelXG = 0.02f;
    highG.accelYG = 0.01f;
    highG.accelZG = normG;
    highG.accelXMps2 = highG.accelXG * kGravityMps2;
    highG.accelYMps2 = highG.accelYG * kGravityMps2;
    highG.accelZMps2 = highG.accelZG * kGravityMps2;
    highG.connected = true;
    highG.hasNewData = true;
    highG.lastUpdatedMs = nowMs;
}

void publishBaro(TelemetryState &telemetry, BaroFilter &filter, const Scenario &scenario, uint32_t nowMs)
{
    if ((nowMs % kBaroPeriodMs) != 0U || !baroAvailable(scenario, nowMs))
    {
        return;
    }

    BarometerTelemetryData &baro = telemetry.barometer;
    baro.valid = true;
    baro.referenceValid = true;
    baro.referencePressurePa = 101325.0f;
    baro.rawAltitudeM = measuredAltitude(scenario, nowMs);
    baro.altitudeM = filter.push(baro.rawAltitudeM);
    baro.pressurePa = 101325.0f;
    baro.lastUpdatedMs = nowMs;
}

void publishMockSample(HighGImuState &highG, TelemetryState &telemetry, const MockFlightDataReading &sample)
{
    highG.accelXG = sample.highAccelXG;
    highG.accelYG = sample.highAccelYG;
    highG.accelZG = sample.highAccelZG;
    highG.accelXMps2 = sample.accelXMps2;
    highG.accelYMps2 = sample.accelYMps2;
    highG.accelZMps2 = sample.accelZMps2;
    highG.connected = true;
    highG.hasNewData = true;
    highG.lastUpdatedMs = sample.sampleMs;

    if (!sample.barometerUpdated)
    {
        return;
    }

    BarometerTelemetryData &baro = telemetry.barometer;
    baro.valid = true;
    baro.referenceValid = true;
    baro.referencePressurePa = 101325.0f;
    baro.rawAltitudeM = sample.rawAltitudeM;
    baro.altitudeM = sample.filteredAltitudeM;
    baro.pressurePa = sample.pressurePa;
    baro.lastUpdatedMs = sample.sampleMs;
}

bool fail(const char *test, const char *message)
{
    std::printf("FAIL %-28s %s\n", test, message);
    return false;
}

bool pass(const char *test)
{
    std::printf("PASS %-28s\n", test);
    return true;
}

ReplayResult runReplay(const Scenario &scenario, uint32_t endMs)
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    HighGImuState highG;
    TelemetryState telemetry;
    BaroFilter filter;
    FlightStateMachineTask fsm(flight, abort, highG, telemetry, logger, config, panic);
    fsm.init();
    fsm.tick(0U);
    flight.state = State::ARMED;
    flight.stateEnteredMs = 0U;

    ReplayResult result;
    State previous = flight.state;

    for (uint32_t nowMs = 0U; nowMs <= endMs; nowMs += kTickMs)
    {
        publishHighG(highG, scenario, nowMs);
        publishBaro(telemetry, filter, scenario, nowMs);
        fsm.tick(nowMs);

        if (flight.state != previous)
        {
            if (flight.state == State::LAUNCH)
            {
                result.launchMs = nowMs;
                if (static_cast<float>(nowMs) < (scenario.launchS * 1000.0f))
                {
                    result.launchedBeforeRealLaunch = true;
                }
            }
            else if (flight.state == State::COAST)
            {
                result.coastMs = nowMs;
            }
            else if (flight.state == State::APOGEE)
            {
                result.apogeeMs = nowMs;
                result.trueAltitudeAtApogeeDecisionM = trueAltitude(scenario, static_cast<float>(nowMs) / 1000.0f);
            }
            else if (flight.state == State::DROGUE)
            {
                result.drogueMs = nowMs;
            }
            else if (flight.state == State::DEPLOY)
            {
                result.deployMs = nowMs;
            }
            else if (flight.state == State::GROUND)
            {
                result.groundMs = nowMs;
            }
            previous = flight.state;
        }
    }

    return result;
}

ReplayResult runMockHalReplay(MockFlightScenarioId scenarioId, uint32_t endMs)
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    HighGImuState highG;
    TelemetryState telemetry;
    MockFlightDataHAL mockHal;
    mockHal.setScenario(scenarioId);
    mockHal.begin();
    FlightStateMachineTask fsm(flight, abort, highG, telemetry, logger, config, panic);
    fsm.init();
    fsm.tick(0U);
    flight.state = State::ARMED;
    flight.stateEnteredMs = 0U;

    ReplayResult result;
    State previous = flight.state;

    for (uint32_t nowMs = 0U; nowMs <= endMs; nowMs += kTickMs)
    {
        MockFlightDataReading sample;
        if (mockHal.read(sample, nowMs))
        {
            publishMockSample(highG, telemetry, sample);
        }
        fsm.tick(nowMs);

        if (flight.state != previous)
        {
            if (flight.state == State::LAUNCH)
            {
                result.launchMs = nowMs;
                if (nowMs < 1000U)
                {
                    result.launchedBeforeRealLaunch = true;
                }
            }
            else if (flight.state == State::COAST)
            {
                result.coastMs = nowMs;
            }
            else if (flight.state == State::APOGEE)
            {
                result.apogeeMs = nowMs;
            }
            else if (flight.state == State::DROGUE)
            {
                result.drogueMs = nowMs;
            }
            else if (flight.state == State::DEPLOY)
            {
                result.deployMs = nowMs;
            }
            else if (flight.state == State::GROUND)
            {
                result.groundMs = nowMs;
            }
            previous = flight.state;
        }
    }

    return result;
}

bool checkFullFlight(const Scenario &scenario)
{
    const ReplayResult result = runReplay(scenario, 36000U);
    const uint32_t launchMs = static_cast<uint32_t>(scenario.launchS * 1000.0f);
    const uint32_t burnoutMs = static_cast<uint32_t>(scenario.burnoutS * 1000.0f);
    const float apogeeDecisionTimeS = static_cast<float>(result.apogeeMs) / 1000.0f;

    if (result.launchedBeforeRealLaunch)
    {
        return fail(scenario.name, "false launch before real launch");
    }
    if (result.launchMs < launchMs || result.launchMs > launchMs + 120U)
    {
        return fail(scenario.name, "launch transition outside 4-sample window");
    }
    if (result.coastMs < burnoutMs || result.coastMs > burnoutMs + 160U)
    {
        return fail(scenario.name, "coast transition outside burnout window");
    }
    if (result.apogeeMs == 0U)
    {
        return fail(scenario.name, "no apogee transition");
    }
    if (apogeeDecisionTimeS < scenario.apogeeS - 1.0f)
    {
        return fail(scenario.name, "dangerously early apogee transition by time");
    }
    if (result.trueAltitudeAtApogeeDecisionM < scenario.apogeeM - 10.0f)
    {
        return fail(scenario.name, "dangerously early apogee transition by altitude");
    }
    if (result.drogueMs < result.apogeeMs + MissionConstants::kDrogueBackupDelayMs)
    {
        return fail(scenario.name, "drogue transition before backup sequence complete");
    }
    if (result.deployMs == 0U || result.groundMs == 0U)
    {
        return fail(scenario.name, "recovery sequence did not reach ground");
    }
    return pass(scenario.name);
}

bool checkMockHalScenario(const char *testName, MockFlightScenarioId scenarioId)
{
    const ReplayResult result = runMockHalReplay(scenarioId, 36000U);
    if (result.launchedBeforeRealLaunch)
    {
        return fail(testName, "mock HAL caused false launch before real launch");
    }
    if (result.launchMs < 1000U || result.launchMs > 1120U)
    {
        return fail(testName, "mock HAL launch transition outside expected window");
    }
    if (result.coastMs == 0U || result.apogeeMs == 0U || result.drogueMs == 0U ||
        result.deployMs == 0U || result.groundMs == 0U)
    {
        return fail(testName, "mock HAL scenario did not complete recovery states");
    }
    return pass(testName);
}

bool checkAbortToSafe()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::COAST;
    flight.coastMs = 1000U;
    abort.status.active = true;
    fsm.tick(2000U);
    if (flight.state != State::SAFE)
    {
        return fail("abort_to_safe", "abort did not force SAFE");
    }
    return pass("abort_to_safe");
}

bool checkForceDeployFromCoast()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::COAST;
    flight.coastMs = 3000U;
    flight.forceRecoveryDeployRequested = true;
    flight.forceRecoveryDeployRequestSeq = 42U;
    fsm.tick(5000U);
    if (flight.state != State::APOGEE)
    {
        return fail("force_deploy_coast", "request did not enter APOGEE through FSM");
    }
    if (!flight.forceRecoveryDeployExecuted || flight.forceRecoveryDeployExecutedSeq != 42U)
    {
        return fail("force_deploy_coast", "request execution was not recorded");
    }
    return pass("force_deploy_coast");
}

bool checkForceDeployRejectedOnPad()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::ARMED;
    flight.forceRecoveryDeployRequested = true;
    flight.forceRecoveryDeployRequestSeq = 43U;
    fsm.tick(500U);
    if (flight.state == State::APOGEE)
    {
        return fail("force_deploy_pad", "pad request entered APOGEE");
    }
    if (flight.forceRecoveryDeployRequested || flight.forceRecoveryDeployExecuted)
    {
        return fail("force_deploy_pad", "rejected request was left active");
    }
    return pass("force_deploy_pad");
}

bool checkLaunchConfirmation()
{
    const Scenario scenario = {"launch_confirm", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.0f, false, false, true};
    const ReplayResult result = runReplay(scenario, 1800U);
    if (result.launchedBeforeRealLaunch)
    {
        return fail("launch_confirm", "3-sample pad acceleration caused launch");
    }
    if (result.launchMs == 0U)
    {
        return fail("launch_confirm", "real 4-sample launch was not detected");
    }
    return pass("launch_confirm");
}
} // namespace

int main()
{
    const Scenario scenarios[] = {
        {"nominal_clean", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.0f, false, false, false},
        {"low_apogee_clean", 1.0f, 2.45f, 9.8f, 250.0f, 22.0f, 0.0f, false, false, false},
        {"baro_noise", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.8f, false, false, false},
        {"baro_gust", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.6f, true, false, false},
        {"baro_dropout", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.4f, false, true, false},
    };

    bool ok = true;
    for (const Scenario &scenario : scenarios)
    {
        ok = checkFullFlight(scenario) && ok;
    }
    ok = checkLaunchConfirmation() && ok;
    ok = checkAbortToSafe() && ok;
    ok = checkForceDeployFromCoast() && ok;
    ok = checkForceDeployRejectedOnPad() && ok;
    ok = checkMockHalScenario("mock_hal_nominal", MockFlightScenarioId::NOMINAL) && ok;
    ok = checkMockHalScenario("mock_hal_low_apogee", MockFlightScenarioId::LOW_APOGEE) && ok;
    ok = checkMockHalScenario("mock_hal_baro_noise", MockFlightScenarioId::BARO_NOISE) && ok;
    ok = checkMockHalScenario("mock_hal_baro_gust", MockFlightScenarioId::BARO_GUST) && ok;
    ok = checkMockHalScenario("mock_hal_baro_dropout", MockFlightScenarioId::BARO_DROPOUT) && ok;
    ok = checkMockHalScenario("mock_hal_pad_false_accel", MockFlightScenarioId::PAD_FALSE_ACCEL) && ok;

    return ok ? 0 : 1;
}
