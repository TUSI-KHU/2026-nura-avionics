#include <cmath>
#include <cstdio>
#include <cstring>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "hal/mock_flight_data_hal.h"
#include "missions/fsm_task.h"
#include "nura_constants.h"

namespace
{
constexpr float kGravityMps2 = NuraConstants::Physics::kGravityMps2;
constexpr uint32_t kTickMs = NuraConstants::Tasks::kFlightStateTaskPeriodMs;
constexpr uint32_t kBaroPeriodMs = NuraConstants::Sensors::kBarometerTaskPeriodMs;

struct FakeConfig : public IAppConfig
{
    unsigned long serialBaudRate() const override { return NuraConstants::App::kSerialBaudRate; }
    uint8_t statusIndicatorPin() const override { return 13U; }
    uint16_t faultBlinkIntervalMs() const override { return NuraConstants::App::kFaultBlinkIntervalMs; }
    uint8_t imuCsPin() const override { return 6U; }
    uint8_t imuReadFailureThreshold() const override { return NuraConstants::Sensors::kImuReadFailureThreshold; }
    uint8_t imuMaxRecoveryAttempts() const override { return NuraConstants::Sensors::kImuMaxRecoveryAttempts; }
    uint32_t imuRecoveryIntervalMs() const override { return NuraConstants::Sensors::kImuRecoveryIntervalMs; }
    uint32_t imuTaskPeriodMs() const override { return NuraConstants::Sensors::kImuTaskPeriodMs; }
    uint32_t magnetometerTaskPeriodMs() const override { return NuraConstants::Sensors::kMagnetometerTaskPeriodMs; }
    uint32_t barometerTaskPeriodMs() const override { return NuraConstants::Sensors::kBarometerTaskPeriodMs; }
    uint32_t barometerRecoveryIntervalMs() const override { return NuraConstants::Sensors::kBarometerRecoveryIntervalMs; }
    uint32_t gnssTaskPeriodMs() const override { return NuraConstants::Sensors::kGnssTaskPeriodMs; }
    uint16_t gnssPollByteBudget() const override { return NuraConstants::Sensors::kGnssPollByteBudget; }
    uint32_t gnssMaxFixAgeMs() const override { return NuraConstants::Sensors::kGnssMaxFixAgeMs; }
    uint32_t watchdogTaskPeriodMs() const override { return NuraConstants::Tasks::kWatchdogTaskPeriodMs; }
    uint32_t flightStateTaskPeriodMs() const override { return kTickMs; }
    uint32_t loggerTaskPeriodMs() const override { return NuraConstants::Tasks::kLoggerTaskPeriodMs; }
    uint32_t telemetryTaskPeriodMs() const override { return NuraConstants::Tasks::kTelemetryTaskPeriodMs; }
    uint32_t telemetryFastPeriodMs() const override { return NuraConstants::Telemetry::kFastPeriodMs; }
    uint32_t telemetryGpsPeriodMs() const override { return NuraConstants::Telemetry::kGpsPeriodMs; }
    uint32_t telemetrySensorFreshMs() const override { return NuraConstants::Telemetry::kSensorFreshMs; }
    uint8_t loggerDrainBudget() const override { return NuraConstants::Logger::kDrainBudget; }
    uint8_t loggerOutputFailThreshold() const override { return NuraConstants::Logger::kOutputFailThreshold; }
    long loraFrequencyHz() const override { return NuraConstants::LoRa::kFlightFrequencyHz; }
    uint32_t loraSpiFrequencyHz() const override { return NuraConstants::LoRa::kFlightSpiFrequencyHz; }
    int loraTxPowerDbm() const override { return NuraConstants::LoRa::kFlightTxPowerDbm; }
    int loraSpreadingFactor() const override { return NuraConstants::LoRa::kSpreadingFactor; }
    long loraSignalBandwidthHz() const override { return NuraConstants::LoRa::kSignalBandwidthHz; }
    int loraCodingRateDenominator() const override { return NuraConstants::LoRa::kCodingRateDenominator; }
    long loraPreambleLength() const override { return NuraConstants::LoRa::kPreambleLength; }
    int loraSyncWord() const override { return NuraConstants::LoRa::kSyncWord; }
    uint8_t loraInitAttempts() const override { return NuraConstants::LoRa::kFlightInitAttempts; }
    uint8_t loraSpiMode() const override { return NuraConstants::LoRa::kFlightSpiMode; }
    bool loraProbeSpiMode() const override { return NuraConstants::LoRa::kFlightProbeSpiMode; }
};

struct FakePanicHandler : public IPanicHandler
{
    bool panicked = false;
    void panic() override { panicked = true; }
};

struct RecordingPyroOutput : public IPyroOutput
{
    bool beginCalled = false;
    bool drogueEnabled = false;
    bool mainEnabled = false;
    uint8_t allOffCount = 0U;
    uint8_t drogueOnCount = 0U;
    uint8_t drogueOffCount = 0U;
    uint8_t mainOnCount = 0U;
    uint8_t mainOffCount = 0U;

    bool begin() override
    {
        beginCalled = true;
        return true;
    }

    void allOff() override
    {
        ++allOffCount;
        drogueEnabled = false;
        mainEnabled = false;
    }

    void setDrogue(bool enabled) override
    {
        drogueEnabled = enabled;
        if (enabled)
        {
            ++drogueOnCount;
        }
        else
        {
            ++drogueOffCount;
        }
    }

    void setMain(bool enabled) override
    {
        mainEnabled = enabled;
        if (enabled)
        {
            ++mainOnCount;
        }
        else
        {
            ++mainOffCount;
        }
    }
};

struct RecordingBuzzerOutput : public IBuzzerOutput
{
    bool beginCalled = false;
    uint16_t currentFrequencyHz = 0U;
    uint16_t toneCount = 0U;
    uint16_t silenceCount = 0U;

    bool begin() override
    {
        beginCalled = true;
        return true;
    }

    void playTone(uint16_t frequencyHz) override
    {
        currentFrequencyHz = frequencyHz;
        ++toneCount;
    }

    void silence() override
    {
        currentFrequencyHz = 0U;
        ++silenceCount;
    }
};

struct BaroFilter
{
    float window[NuraConstants::Sensors::kBarometerMedianWindowSamples] = {0.0f, 0.0f, 0.0f};
    uint8_t head = 0U;
    uint8_t count = 0U;
    float filtered = 0.0f;
    bool ready = false;

    float push(float raw)
    {
        window[head] = raw;
        head = static_cast<uint8_t>((head + 1U) % NuraConstants::Sensors::kBarometerMedianWindowSamples);
        if (count < NuraConstants::Sensors::kBarometerMedianWindowSamples)
        {
            ++count;
        }

        float values[NuraConstants::Sensors::kBarometerMedianWindowSamples] = {window[0], window[1], window[2]};
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
            filtered += NuraConstants::Sensors::kBarometerAltitudeLpfAlpha * (median - filtered);
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

void publishHighGNorm(HighGImuState &highG, TelemetryState &telemetry, float normG, uint32_t nowMs)
{
    highG.accelXG = 0.02f;
    highG.accelYG = 0.01f;
    highG.accelZG = normG;
    highG.accelXMps2 = highG.accelXG * kGravityMps2;
    highG.accelYMps2 = highG.accelYG * kGravityMps2;
    highG.accelZMps2 = highG.accelZG * kGravityMps2;
    highG.connected = true;
    highG.hasNewData = true;
    highG.lastUpdatedMs = nowMs;
    telemetry.health.highAccelOk = true;
}

void publishHighG(HighGImuState &highG, TelemetryState &telemetry, const Scenario &scenario, uint32_t nowMs)
{
    publishHighGNorm(highG, telemetry, highGNorm(scenario, nowMs), nowMs);
}

void publishLowGNorm(ImuState &imu, float normG, uint32_t nowMs)
{
    imu.data.accelXMps2 = 0.02f * kGravityMps2;
    imu.data.accelYMps2 = 0.01f * kGravityMps2;
    imu.data.accelZMps2 = normG * kGravityMps2;
    imu.data.lastUpdatedMs = nowMs;
}

void markHighGFault(HighGImuState &highG, TelemetryState &telemetry)
{
    highG.connected = false;
    highG.hasNewData = false;
    telemetry.health.highAccelOk = false;
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

void publishMockSample(ImuState &imu, HighGImuState &highG, TelemetryState &telemetry, const MockFlightDataReading &sample)
{
    imu.data.accelXMps2 = sample.accelXMps2;
    imu.data.accelYMps2 = sample.accelYMps2;
    imu.data.accelZMps2 = sample.accelZMps2;
    imu.data.gyroXDps = sample.gyroXDps;
    imu.data.gyroYDps = sample.gyroYDps;
    imu.data.gyroZDps = sample.gyroZDps;
    imu.data.attitudeValid = sample.attitudeValid;
    imu.data.rollDeg = sample.rollDeg;
    imu.data.pitchDeg = sample.pitchDeg;
    imu.data.yawDeg = sample.yawDeg;
    imu.data.tiltValid = sample.tiltValid;
    imu.data.tiltAngleDeg = sample.tiltAngleDeg;
    imu.data.lastUpdatedMs = sample.sampleMs;

    highG.accelXG = sample.highAccelXG;
    highG.accelYG = sample.highAccelYG;
    highG.accelZG = sample.highAccelZG;
    highG.accelXMps2 = sample.accelXMps2;
    highG.accelYMps2 = sample.accelYMps2;
    highG.accelZMps2 = sample.accelZMps2;
    highG.connected = true;
    highG.hasNewData = true;
    highG.lastUpdatedMs = sample.sampleMs;
    telemetry.health.highAccelOk = true;

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
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    BaroFilter filter;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();
    fsm.tick(0U);
    flight.state = State::ARMED;
    flight.stateEnteredMs = 0U;

    ReplayResult result;
    State previous = flight.state;

    for (uint32_t nowMs = 0U; nowMs <= endMs; nowMs += kTickMs)
    {
        publishHighG(highG, telemetry, scenario, nowMs);
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
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    MockFlightDataHAL mockHal;
    mockHal.setScenario(scenarioId);
    mockHal.begin();
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
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
            publishMockSample(imu, highG, telemetry, sample);
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
    const float landingTimeS = scenario.apogeeS + (scenario.apogeeM / scenario.descentRateMps);

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
    if (result.drogueMs < result.apogeeMs + NuraConstants::Flight::kDrogueBackupDelayMs)
    {
        return fail(scenario.name, "drogue transition before backup sequence complete");
    }
    if (result.deployMs == 0U || result.groundMs == 0U)
    {
        return fail(scenario.name, "recovery sequence did not reach ground");
    }
    if (static_cast<float>(result.groundMs) < ((landingTimeS * 1000.0f) - 200.0f))
    {
        return fail(scenario.name, "ground transition before landing");
    }
    return pass(scenario.name);
}

bool checkLandingWindowRejectsSlowDescent()
{
    const Scenario scenario = {"landing_window", 1.0f, 2.55f, 10.5f, 400.0f, 5.0f, 0.0f, false, false, false};
    const float landingTimeS = scenario.apogeeS + (scenario.apogeeM / scenario.descentRateMps);
    const ReplayResult early = runReplay(scenario, 56000U);
    if (early.deployMs == 0U)
    {
        return fail("landing_window", "slow descent scenario did not reach DEPLOY");
    }
    if (early.groundMs != 0U)
    {
        return fail("landing_window", "landing detector accepted adjacent descent deltas");
    }

    const ReplayResult full = runReplay(scenario, static_cast<uint32_t>((landingTimeS + 5.0f) * 1000.0f));
    if (full.groundMs == 0U)
    {
        return fail("landing_window", "landing detector did not reach GROUND after touchdown");
    }
    if (static_cast<float>(full.groundMs) < ((landingTimeS * 1000.0f) - 200.0f))
    {
        return fail("landing_window", "ground transition happened before touchdown");
    }
    return pass("landing_window");
}

bool checkBaroFaultUsesApogeeTimeout()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::COAST;
    flight.launchMs = 0U;
    flight.coastMs = 8000U;
    flight.stateEnteredMs = 8000U;
    telemetry.barometer.valid = false;
    telemetry.barometer.referenceValid = true;
    telemetry.barometer.fault = true;
    telemetry.barometer.faultFlags = BARO_FAULT_BAD_VALUE;

    fsm.tick(10000U);
    if (flight.state != State::COAST)
    {
        return fail("baro_fault_timeout", "baro fault left COAST before timeout");
    }

    fsm.tick(flight.coastMs + NuraConstants::Flight::kApogeeTimeoutMs);
    if (flight.state != State::APOGEE)
    {
        return fail("baro_fault_timeout", "baro fault did not use apogee timer fallback");
    }
    return pass("baro_fault_timeout");
}

bool checkBaroFaultUsesAttitudeFallback()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::COAST;
    flight.launchMs = 0U;
    flight.coastMs = 3000U;
    flight.stateEnteredMs = 3000U;
    telemetry.barometer.valid = false;
    telemetry.barometer.referenceValid = true;
    telemetry.barometer.fault = true;
    telemetry.barometer.faultFlags = BARO_FAULT_BAD_VALUE;

    for (uint32_t nowMs = 7960U; nowMs < NuraConstants::Flight::kBaroFaultAttitudeFallbackMinFlightTimeMs; nowMs += kTickMs)
    {
        imu.data.tiltValid = true;
        imu.data.tiltAngleDeg = NuraConstants::Flight::kBaroFaultAttitudeFallbackTiltDeg + 5.0f;
        imu.data.lastUpdatedMs = nowMs;
        fsm.tick(nowMs);
    }
    if (flight.state != State::COAST)
    {
        return fail("baro_fault_attitude", "tilt fallback fired before min flight time");
    }

    for (uint8_t i = 0U; i < NuraConstants::Flight::kBaroFaultAttitudeFallbackConfirmSamples; ++i)
    {
        const uint32_t nowMs = NuraConstants::Flight::kBaroFaultAttitudeFallbackMinFlightTimeMs +
                               (static_cast<uint32_t>(i) * kTickMs);
        imu.data.tiltValid = true;
        imu.data.tiltAngleDeg = NuraConstants::Flight::kBaroFaultAttitudeFallbackTiltDeg + 5.0f;
        imu.data.lastUpdatedMs = nowMs;
        fsm.tick(nowMs);
    }

    if (flight.state != State::APOGEE)
    {
        return fail("baro_fault_attitude", "baro fault tilt fallback did not enter APOGEE");
    }
    return pass("baro_fault_attitude");
}

bool checkStuckBaroFaultIsLatched()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::COAST;
    flight.launchMs = 0U;
    flight.coastMs = 8000U;
    flight.stateEnteredMs = 8000U;

    for (uint32_t nowMs = 8000U; nowMs <= 13200U; nowMs += kBaroPeriodMs)
    {
        if (!telemetry.barometer.fault)
        {
            telemetry.barometer.valid = true;
            telemetry.barometer.referenceValid = true;
            telemetry.barometer.pressurePa = 101325.0f;
            telemetry.barometer.referencePressurePa = 101325.0f;
            telemetry.barometer.rawAltitudeM = 120.0f;
            telemetry.barometer.altitudeM = 120.0f;
            telemetry.barometer.lastUpdatedMs = nowMs;
        }
        fsm.tick(nowMs);
    }

    if (!telemetry.barometer.fault || (telemetry.barometer.faultFlags & BARO_FAULT_STUCK) == 0U)
    {
        return fail("baro_stuck_fault", "constant in-flight baro altitude did not latch stuck fault");
    }
    if (flight.state != State::COAST)
    {
        return fail("baro_stuck_fault", "stuck baro fault caused immediate apogee transition");
    }
    return pass("baro_stuck_fault");
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
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
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
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
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
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
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

bool checkLowGFallbackLaunch()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::ARMED;
    flight.stateEnteredMs = 0U;
    markHighGFault(highG, telemetry);

    for (uint8_t i = 0U; i < NuraConstants::Flight::kLaunchConfirmSamples; ++i)
    {
        const uint32_t nowMs = 1000U + (static_cast<uint32_t>(i) * kTickMs);
        publishLowGNorm(imu, NuraConstants::Flight::kLaunchAccelThresholdG + 0.4f, nowMs);
        fsm.tick(nowMs);
    }

    if (flight.state != State::LAUNCH)
    {
        return fail("low_g_launch_fallback", "low-g accel norm did not replace failed high-g launch detector");
    }
    return pass("low_g_launch_fallback");
}

bool checkLowGFallbackBurnout()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::LAUNCH;
    flight.launchMs = 1000U;
    flight.stateEnteredMs = 1000U;
    markHighGFault(highG, telemetry);

    for (uint8_t i = 0U; i < NuraConstants::Flight::kBurnoutConfirmSamples; ++i)
    {
        const uint32_t nowMs = 1500U + (static_cast<uint32_t>(i) * kTickMs);
        publishLowGNorm(imu, NuraConstants::Flight::kBurnoutAccelThresholdG - 0.3f, nowMs);
        fsm.tick(nowMs);
    }

    if (flight.state != State::COAST)
    {
        return fail("low_g_burnout_fallback", "low-g accel norm did not replace failed high-g burnout detector");
    }
    return pass("low_g_burnout_fallback");
}

bool checkHighGPreferredOverLowG()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();
    flight.state = State::ARMED;
    flight.stateEnteredMs = 0U;

    for (uint8_t i = 0U; i < NuraConstants::Flight::kLaunchConfirmSamples; ++i)
    {
        const uint32_t nowMs = 1000U + (static_cast<uint32_t>(i) * kTickMs);
        publishHighGNorm(highG, telemetry, NuraConstants::Flight::kLaunchAccelThresholdG - 0.6f, nowMs);
        publishLowGNorm(imu, NuraConstants::Flight::kLaunchAccelThresholdG + 0.6f, nowMs);
        fsm.tick(nowMs);
    }

    if (flight.state != State::ARMED)
    {
        return fail("high_g_preferred", "healthy high-g detector was bypassed by low-g samples");
    }
    return pass("high_g_preferred");
}

bool checkPyroOutputSequence()
{
    FakeConfig config;
    FakePanicHandler panic;
    RecordingPyroOutput pyro;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic, &pyro);
    fsm.init();
    fsm.tick(0U);

    flight.state = State::COAST;
    flight.stateEnteredMs = 1000U;
    flight.launchMs = 0U;
    flight.coastMs = 1000U;
    flight.forceRecoveryDeployRequested = true;
    flight.forceRecoveryDeployRequestSeq = 7U;

    const uint32_t apogeeMs = 9000U;
    fsm.tick(apogeeMs);
    if (flight.state != State::APOGEE || !pyro.drogueEnabled || pyro.drogueOnCount != 1U)
    {
        return fail("pyro_output_sequence", "drogue did not fire on APOGEE entry");
    }

    fsm.tick(apogeeMs + NuraConstants::Flight::kPyroFireDurationMs);
    if (pyro.drogueEnabled || pyro.drogueOffCount != 1U)
    {
        return fail("pyro_output_sequence", "drogue primary pulse did not turn off");
    }

    fsm.tick(apogeeMs + NuraConstants::Flight::kDrogueBackupDelayMs);
    if (!pyro.drogueEnabled || pyro.drogueOnCount != 2U)
    {
        return fail("pyro_output_sequence", "drogue retry pulse did not turn on");
    }

    const uint32_t drogueMs = apogeeMs +
                              NuraConstants::Flight::kDrogueBackupDelayMs +
                              NuraConstants::Flight::kPyroFireDurationMs;
    fsm.tick(drogueMs);
    if (flight.state != State::DROGUE || pyro.drogueEnabled || pyro.drogueOffCount != 2U)
    {
        return fail("pyro_output_sequence", "drogue retry pulse did not complete");
    }

    const uint32_t deployMs = drogueMs + NuraConstants::Flight::kMainTimeoutMs;
    fsm.tick(deployMs);
    if (flight.state != State::DEPLOY || !pyro.mainEnabled || pyro.mainOnCount != 1U)
    {
        return fail("pyro_output_sequence", "main did not fire on DEPLOY entry");
    }

    fsm.tick(deployMs + NuraConstants::Flight::kPyroFireDurationMs);
    if (pyro.mainEnabled || pyro.mainOffCount != 1U || !flight.mainSequenceComplete)
    {
        return fail("pyro_output_sequence", "main pulse did not turn off");
    }

    return pass("pyro_output_sequence");
}

bool checkBuzzerStateTransitionPatterns()
{
    FakeConfig config;
    FakePanicHandler panic;
    RecordingBuzzerOutput buzzer;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic, nullptr, &buzzer);
    fsm.init();

    fsm.tick(0U);
    if (!buzzer.beginCalled ||
        buzzer.currentFrequencyHz != NuraConstants::Buzzer::kInitSafeToneFrequencyHz)
    {
        return fail("buzzer_patterns", "INIT to SAFE did not start the first medium tone");
    }

    fsm.tick(kTickMs);
    fsm.tick(NuraConstants::Buzzer::kInitSafeBeepMs);
    if (buzzer.currentFrequencyHz != 0U)
    {
        return fail("buzzer_patterns", "INIT to SAFE seven-tone pattern did not include its first gap");
    }

    const uint32_t initSlotMs = NuraConstants::Buzzer::kInitSafeBeepMs +
                                NuraConstants::Buzzer::kInitSafeGapMs;
    fsm.tick(initSlotMs * 6U);
    if (buzzer.currentFrequencyHz != NuraConstants::Buzzer::kInitSafeToneFrequencyHz)
    {
        return fail("buzzer_patterns", "INIT to SAFE seven-tone pattern did not play its seventh tone");
    }

    const uint32_t alertStartMs = initSlotMs * NuraConstants::Buzzer::kInitSafeBeepCount;
    fsm.tick(alertStartMs);
    if (buzzer.currentFrequencyHz != NuraConstants::Buzzer::kArmedAlertToneFrequencyHz)
    {
        return fail("buzzer_patterns", "SAFE to ARMED flat alert did not start after seven tones");
    }

    for (uint8_t i = 0U; i < NuraConstants::Flight::kLaunchConfirmSamples; ++i)
    {
        const uint32_t nowMs = alertStartMs + 100U + (static_cast<uint32_t>(i) * kTickMs);
        publishHighGNorm(highG, telemetry, NuraConstants::Flight::kLaunchAccelThresholdG + 0.5f, nowMs);
        fsm.tick(nowMs);
    }
    if (flight.state != State::LAUNCH || buzzer.currentFrequencyHz != NuraConstants::Buzzer::kToneFrequencyHz)
    {
        return fail("buzzer_patterns", "ARMED to LAUNCH did not replace the alert with five tones");
    }

    const uint32_t launchMs = alertStartMs + 100U +
                              ((NuraConstants::Flight::kLaunchConfirmSamples - 1U) * kTickMs);
    fsm.tick(launchMs + NuraConstants::Buzzer::kTransitionBeepMs);
    if (buzzer.currentFrequencyHz != 0U)
    {
        return fail("buzzer_patterns", "five-tone transition pattern did not include a gap");
    }
    fsm.tick(launchMs + (NuraConstants::Buzzer::kTransitionBeepMs +
                         NuraConstants::Buzzer::kTransitionGapMs));
    if (buzzer.currentFrequencyHz != NuraConstants::Buzzer::kToneFrequencyHz)
    {
        return fail("buzzer_patterns", "five-tone transition pattern did not repeat");
    }

    const uint32_t transitionDurationMs =
        (NuraConstants::Buzzer::kTransitionBeepMs + NuraConstants::Buzzer::kTransitionGapMs) *
        NuraConstants::Buzzer::kTransitionBeepCount;
    fsm.tick(launchMs + transitionDurationMs);
    for (uint8_t i = 0U; i < NuraConstants::Flight::kBurnoutConfirmSamples; ++i)
    {
        const uint32_t nowMs = launchMs + transitionDurationMs + 100U +
                               (static_cast<uint32_t>(i) * kTickMs);
        publishHighGNorm(highG, telemetry, NuraConstants::Flight::kBurnoutAccelThresholdG - 0.3f, nowMs);
        fsm.tick(nowMs);
    }
    if (flight.state != State::COAST || buzzer.currentFrequencyHz != NuraConstants::Buzzer::kToneFrequencyHz)
    {
        return fail("buzzer_patterns", "LAUNCH to COAST did not start the common five tones");
    }

    const uint32_t coastMs = launchMs + transitionDurationMs + 100U +
                             ((NuraConstants::Flight::kBurnoutConfirmSamples - 1U) * kTickMs);
    const uint32_t transitionSlotMs = NuraConstants::Buzzer::kTransitionBeepMs +
                                      NuraConstants::Buzzer::kTransitionGapMs;
    for (uint8_t i = 0U; i < NuraConstants::Buzzer::kTransitionBeepCount; ++i)
    {
        fsm.tick(coastMs + (static_cast<uint32_t>(i) * transitionSlotMs) +
                 NuraConstants::Buzzer::kTransitionBeepMs);
        if (buzzer.currentFrequencyHz != 0U)
        {
            return fail("buzzer_patterns", "common five-tone transition missed a gap");
        }
        if ((i + 1U) < NuraConstants::Buzzer::kTransitionBeepCount)
        {
            fsm.tick(coastMs + (static_cast<uint32_t>(i + 1U) * transitionSlotMs));
            if (buzzer.currentFrequencyHz != NuraConstants::Buzzer::kToneFrequencyHz)
            {
                return fail("buzzer_patterns", "common five-tone transition missed a tone");
            }
        }
    }
    fsm.tick(coastMs + transitionDurationMs);
    if (buzzer.currentFrequencyHz != 0U || buzzer.toneCount < 10U)
    {
        return fail("buzzer_patterns", "common five-tone transition did not complete");
    }

    return pass("buzzer_patterns");
}

#if defined(NURA_BENCH_FSM_AUTOFLOW)
bool checkBenchAutoFlow()
{
    FakeConfig config;
    FakePanicHandler panic;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    TelemetryState telemetry;
    FlightStateMachineTask fsm(flight, abort, highG, imu, telemetry, logger, config, panic);
    fsm.init();

    const uint32_t endMs = NuraConstants::BenchIntegration::kFsmAutoArmDelayMs +
                           NuraConstants::BenchIntegration::kFsmAutoLaunchDelayMs +
                           NuraConstants::BenchIntegration::kFsmAutoBurnoutDelayMs +
                           NuraConstants::Flight::kApogeeTimeoutMs +
                           NuraConstants::Flight::kDrogueBackupDelayMs +
                           NuraConstants::Flight::kPyroFireDurationMs +
                           NuraConstants::Flight::kMainTimeoutMs +
                           NuraConstants::BenchIntegration::kFsmAutoGroundDelayMs +
                           1000U;

    for (uint32_t nowMs = 0U; nowMs <= endMs; nowMs += kTickMs)
    {
        fsm.tick(nowMs);
    }

    if (panic.panicked)
    {
        return fail("bench_auto_flow", "panic handler was called");
    }
    if (flight.state != State::GROUND)
    {
        return fail("bench_auto_flow", "bench auto-flow did not reach GROUND without sensor events");
    }
    return pass("bench_auto_flow");
}
#endif
} // namespace

int main()
{
    bool ok = true;

#if defined(NURA_BENCH_FSM_AUTOFLOW)
    ok = checkBenchAutoFlow() && ok;
    return ok ? 0 : 1;
#else
    const Scenario scenarios[] = {
        {"nominal_clean", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.0f, false, false, false},
        {"low_apogee_clean", 1.0f, 2.45f, 9.8f, 250.0f, 22.0f, 0.0f, false, false, false},
        {"baro_noise", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.8f, false, false, false},
        {"baro_gust", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.6f, true, false, false},
        {"baro_dropout", 1.0f, 2.55f, 10.5f, 400.0f, 25.0f, 0.4f, false, true, false},
    };

    for (const Scenario &scenario : scenarios)
    {
        ok = checkFullFlight(scenario) && ok;
    }
    ok = checkLaunchConfirmation() && ok;
    ok = checkLowGFallbackLaunch() && ok;
    ok = checkLowGFallbackBurnout() && ok;
    ok = checkHighGPreferredOverLowG() && ok;
    ok = checkPyroOutputSequence() && ok;
    ok = checkBuzzerStateTransitionPatterns() && ok;
    ok = checkAbortToSafe() && ok;
    ok = checkForceDeployFromCoast() && ok;
    ok = checkForceDeployRejectedOnPad() && ok;
    ok = checkLandingWindowRejectsSlowDescent() && ok;
    ok = checkBaroFaultUsesApogeeTimeout() && ok;
    ok = checkBaroFaultUsesAttitudeFallback() && ok;
    ok = checkStuckBaroFaultIsLatched() && ok;
    ok = checkMockHalScenario("mock_hal_nominal", MockFlightScenarioId::NOMINAL) && ok;
    ok = checkMockHalScenario("mock_hal_low_apogee", MockFlightScenarioId::LOW_APOGEE) && ok;
    ok = checkMockHalScenario("mock_hal_baro_noise", MockFlightScenarioId::BARO_NOISE) && ok;
    ok = checkMockHalScenario("mock_hal_baro_gust", MockFlightScenarioId::BARO_GUST) && ok;
    ok = checkMockHalScenario("mock_hal_baro_dropout", MockFlightScenarioId::BARO_DROPOUT) && ok;
    ok = checkMockHalScenario("mock_hal_pad_false_accel", MockFlightScenarioId::PAD_FALSE_ACCEL) && ok;

    return ok ? 0 : 1;
#endif
}
