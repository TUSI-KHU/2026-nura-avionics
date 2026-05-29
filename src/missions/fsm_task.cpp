#include "fsm_task.h"

#include <math.h>

FlightStateMachineTask::FlightStateMachineTask(FlightState &flightState,
                                               AbortState &abortState,
                                               HighGImuState &highGImuState,
                                               const ImuState &imuState,
                                               TelemetryState &telemetryState,
                                               Logger &logger,
                                               const IAppConfig &config,
                                               IPanicHandler &panicHandler)
    : flightState_(flightState),
      abortState_(abortState),
      highGImuState_(highGImuState),
      imuState_(imuState),
      telemetryState_(telemetryState),
      logger_(logger),
      config_(config),
      panicHandler_(panicHandler) {}

const char *FlightStateMachineTask::name() const
{
    return "fsm";
}

bool FlightStateMachineTask::init()
{
    flightState_ = FlightState{};
    resetFlightScratch();
    return true;
}

bool FlightStateMachineTask::tick(uint32_t nowMs)
{
    if (abortState_.status.active && flightState_.state != State::SAFE)
    {
        transitionTo(State::SAFE, nowMs);
        return true;
    }

    if (consumeForceRecoveryDeployRequest(nowMs))
    {
        return true;
    }

    if (tickBenchAutoFlow(nowMs))
    {
        return true;
    }

    switch (flightState_.state)
    {
    case State::INIT:
        transitionTo(State::SAFE, nowMs);
        break;
    case State::SAFE:
        // TODO: connect arming switch / command input.
        break;
    case State::ARMED:
        tickArmed(nowMs);
        break;
    case State::LAUNCH:
        tickLaunch(nowMs);
        break;
    case State::COAST:
        tickCoast(nowMs);
        break;
    case State::APOGEE:
        tickApogee(nowMs);
        break;
    case State::DROGUE:
        tickDrogue(nowMs);
        break;
    case State::DEPLOY:
        tickDeploy(nowMs);
        break;
    case State::GROUND:
        break;
    case State::FAULT:
        break;
    default:
        transitionTo(State::FAULT, nowMs);
        break;
    }

    return true;
}

bool FlightStateMachineTask::tickBenchAutoFlow(uint32_t nowMs)
{
#if defined(NURA_BENCH_FSM_AUTOFLOW)
    if (abortState_.status.active)
    {
        return false;
    }

    const uint32_t elapsedMs = nowMs - flightState_.stateEnteredMs;
    switch (flightState_.state)
    {
    case State::SAFE:
        if (elapsedMs >= NuraConstants::BenchIntegration::kFsmAutoArmDelayMs)
        {
            LOGW(logger_, nowMs, "bench", "auto arm");
            transitionTo(State::ARMED, nowMs);
            return true;
        }
        break;
    case State::ARMED:
        if (elapsedMs >= NuraConstants::BenchIntegration::kFsmAutoLaunchDelayMs)
        {
            recordDecision(FlightDecisionKind::LAUNCH_ACCEL,
                           FlightDecisionResult::ACCEPT,
                           DECISION_REASON_FORCED,
                           nowMs,
                           static_cast<float>(elapsedMs),
                           static_cast<float>(NuraConstants::BenchIntegration::kFsmAutoLaunchDelayMs),
                           0.0f,
                           0.0f,
                           0U,
                           0U);
            LOGW(logger_, nowMs, "bench", "auto launch");
            transitionTo(State::LAUNCH, nowMs);
            return true;
        }
        break;
    case State::LAUNCH:
        if (elapsedMs >= NuraConstants::BenchIntegration::kFsmAutoBurnoutDelayMs)
        {
            recordDecision(FlightDecisionKind::BURNOUT_ACCEL,
                           FlightDecisionResult::ACCEPT,
                           DECISION_REASON_FORCED,
                           nowMs,
                           static_cast<float>(elapsedMs),
                           static_cast<float>(NuraConstants::BenchIntegration::kFsmAutoBurnoutDelayMs),
                           0.0f,
                           0.0f,
                           0U,
                           0U);
            LOGW(logger_, nowMs, "bench", "auto coast");
            transitionTo(State::COAST, nowMs);
            return true;
        }
        break;
    case State::DEPLOY:
        if (elapsedMs >= NuraConstants::BenchIntegration::kFsmAutoGroundDelayMs)
        {
            mainPyroOff_ = true;
            flightState_.mainSequenceComplete = true;
            recordDecision(FlightDecisionKind::LANDING,
                           FlightDecisionResult::ACCEPT,
                           DECISION_REASON_FORCED,
                           nowMs,
                           static_cast<float>(elapsedMs),
                           static_cast<float>(NuraConstants::BenchIntegration::kFsmAutoGroundDelayMs),
                           0.0f,
                           0.0f,
                           0U,
                           0U);
            LOGW(logger_, nowMs, "bench", "auto ground");
            transitionTo(State::GROUND, nowMs);
            return true;
        }
        break;
    default:
        break;
    }
#else
    (void)nowMs;
#endif

    return false;
}

uint32_t FlightStateMachineTask::periodMs() const
{
    return config_.flightStateTaskPeriodMs();
}

void FlightStateMachineTask::tickArmed(uint32_t nowMs)
{
    AccelSample accel;
    if (!consumeFlightAccelSample(nowMs, lastLaunchAccelSampleMs_, lastLaunchAccelSource_, accel))
    {
        return;
    }

    if (accel.normG >= NuraConstants::Flight::kLaunchAccelThresholdG)
    {
        ++launchConfirmCount_;
    }
    else
    {
        launchConfirmCount_ = 0U;
    }

    uint16_t reason = accel.source == AccelSource::HIGH_G ? DECISION_REASON_PRIMARY_SENSOR : DECISION_REASON_FALLBACK_SENSOR;
    reason = static_cast<uint16_t>(reason |
                                   (accel.normG >= NuraConstants::Flight::kLaunchAccelThresholdG
                                        ? DECISION_REASON_THRESHOLD_MET
                                        : DECISION_REASON_THRESHOLD_NOT_MET));
    const bool launchAccepted = launchConfirmCount_ >= NuraConstants::Flight::kLaunchConfirmSamples;
    if (launchAccepted)
    {
        reason = static_cast<uint16_t>(reason | DECISION_REASON_CONFIRMATION_MET);
    }
    recordDecision(FlightDecisionKind::LAUNCH_ACCEL,
                   launchAccepted ? FlightDecisionResult::ACCEPT : FlightDecisionResult::OBSERVE,
                   reason,
                   accel.sampleMs,
                   accel.normG,
                   NuraConstants::Flight::kLaunchAccelThresholdG,
                   0.0f,
                   0.0f,
                   launchConfirmCount_,
                   static_cast<uint8_t>(accel.source));

    if (launchConfirmCount_ >= NuraConstants::Flight::kLaunchConfirmSamples)
    {
        transitionTo(State::LAUNCH, accel.sampleMs);
    }
}

void FlightStateMachineTask::tickLaunch(uint32_t nowMs)
{
    AccelSample accel;
    if (!consumeFlightAccelSample(nowMs, lastBurnoutAccelSampleMs_, lastBurnoutAccelSource_, accel))
    {
        return;
    }

    if (accel.normG < NuraConstants::Flight::kBurnoutAccelThresholdG)
    {
        ++burnoutConfirmCount_;
    }
    else
    {
        burnoutConfirmCount_ = 0U;
    }

    uint16_t reason = accel.source == AccelSource::HIGH_G ? DECISION_REASON_PRIMARY_SENSOR : DECISION_REASON_FALLBACK_SENSOR;
    reason = static_cast<uint16_t>(reason |
                                   (accel.normG < NuraConstants::Flight::kBurnoutAccelThresholdG
                                        ? DECISION_REASON_THRESHOLD_MET
                                        : DECISION_REASON_THRESHOLD_NOT_MET));
    const bool burnoutAccepted = burnoutConfirmCount_ >= NuraConstants::Flight::kBurnoutConfirmSamples;
    if (burnoutAccepted)
    {
        reason = static_cast<uint16_t>(reason | DECISION_REASON_CONFIRMATION_MET);
    }
    recordDecision(FlightDecisionKind::BURNOUT_ACCEL,
                   burnoutAccepted ? FlightDecisionResult::ACCEPT : FlightDecisionResult::OBSERVE,
                   reason,
                   accel.sampleMs,
                   accel.normG,
                   NuraConstants::Flight::kBurnoutAccelThresholdG,
                   0.0f,
                   0.0f,
                   burnoutConfirmCount_,
                   static_cast<uint8_t>(accel.source));

    if (burnoutConfirmCount_ >= NuraConstants::Flight::kBurnoutConfirmSamples)
    {
        transitionTo(State::COAST, accel.sampleMs);
    }
}

void FlightStateMachineTask::tickCoast(uint32_t nowMs)
{
    const uint32_t coastElapsedMs = nowMs - flightState_.coastMs;
    if (coastElapsedMs >= NuraConstants::Flight::kApogeeTimeoutMs)
    {
        recordDecision(FlightDecisionKind::APOGEE_TIMER,
                       FlightDecisionResult::ACCEPT,
                       DECISION_REASON_TIMEOUT,
                       nowMs,
                       static_cast<float>(coastElapsedMs),
                       static_cast<float>(NuraConstants::Flight::kApogeeTimeoutMs),
                       telemetryState_.barometer.altitudeM,
                       maxCoastAltitudeM_,
                       0U,
                       0U);
        transitionTo(State::APOGEE, nowMs);
        return;
    }

    if (telemetryState_.barometer.fault)
    {
        if (baroFaultAttitudeFallbackReady(nowMs))
        {
            transitionTo(State::APOGEE, nowMs);
        }
        return;
    }

    if (!consumeBarometerSample())
    {
        return;
    }

    const float currentAltitudeM = telemetryState_.barometer.altitudeM;
    if (currentAltitudeM > maxCoastAltitudeM_)
    {
        maxCoastAltitudeM_ = currentAltitudeM;
    }

    const uint32_t launchElapsedMs = telemetryState_.barometer.lastUpdatedMs - flightState_.launchMs;
    const bool allowApogee = launchElapsedMs >= NuraConstants::Flight::kApogeeMinFlightTimeMs;
    if (!allowApogee)
    {
        apogeeConfirmCount_ = 0U;
        descentConfirmCount_ = 0U;
        recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                       FlightDecisionResult::REJECT,
                       DECISION_REASON_TOO_EARLY,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       static_cast<float>(launchElapsedMs),
                       static_cast<float>(NuraConstants::Flight::kApogeeMinFlightTimeMs),
                       maxCoastAltitudeM_,
                       apogeeConfirmCount_,
                       descentConfirmCount_);
        return;
    }

    if (apogeePredictionReady(currentAltitudeM))
    {
        ++apogeeConfirmCount_;
    }
    else
    {
        apogeeConfirmCount_ = 0U;
    }

    if ((maxCoastAltitudeM_ - currentAltitudeM) >= NuraConstants::Flight::kApogeeDropThresholdM)
    {
        ++descentConfirmCount_;
    }
    else
    {
        descentConfirmCount_ = 0U;
    }

    const bool predictionAccepted = apogeeConfirmCount_ >= NuraConstants::Flight::kApogeeConfirmSamples;
    const bool descentAccepted = descentConfirmCount_ >= NuraConstants::Flight::kApogeeDescentConfirmSamples;
    if (predictionAccepted)
    {
        recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                       FlightDecisionResult::ACCEPT,
                       DECISION_REASON_CONFIRMATION_MET,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       maxCoastAltitudeM_,
                       0.0f,
                       0.0f,
                       apogeeConfirmCount_,
                       descentConfirmCount_);
    }
    else
    {
        recordDecision(FlightDecisionKind::APOGEE_DESCENT,
                       descentAccepted ? FlightDecisionResult::ACCEPT : FlightDecisionResult::OBSERVE,
                       descentAccepted ? DECISION_REASON_CONFIRMATION_MET : DECISION_REASON_THRESHOLD_NOT_MET,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       maxCoastAltitudeM_,
                       maxCoastAltitudeM_ - currentAltitudeM,
                       NuraConstants::Flight::kApogeeDropThresholdM,
                       apogeeConfirmCount_,
                       descentConfirmCount_);
    }

    if (apogeeConfirmCount_ >= NuraConstants::Flight::kApogeeConfirmSamples ||
        descentConfirmCount_ >= NuraConstants::Flight::kApogeeDescentConfirmSamples)
    {
        transitionTo(State::APOGEE, telemetryState_.barometer.lastUpdatedMs);
    }
}

void FlightStateMachineTask::tickApogee(uint32_t nowMs)
{
    const uint32_t elapsedMs = nowMs - flightState_.apogeeMs;

    if (!primaryDrogueOff_ && elapsedMs >= NuraConstants::Flight::kPyroFireDurationMs)
    {
        // TODO: drive drogue primary pyro OFF when pyro HAL/pinmap is defined.
        primaryDrogueOff_ = true;
    }

    if (!backupDrogueOn_ && elapsedMs >= NuraConstants::Flight::kDrogueBackupDelayMs)
    {
        // TODO: drive drogue backup pyro ON when pyro HAL/pinmap is defined.
        backupDrogueOn_ = true;
    }

    if (backupDrogueOn_ && !backupDrogueOff_ &&
        elapsedMs >= (NuraConstants::Flight::kDrogueBackupDelayMs + NuraConstants::Flight::kPyroFireDurationMs))
    {
        // TODO: drive drogue backup pyro OFF when pyro HAL/pinmap is defined.
        backupDrogueOff_ = true;
        flightState_.drogueSequenceComplete = true;
        transitionTo(State::DROGUE, nowMs);
    }
}

void FlightStateMachineTask::tickDrogue(uint32_t nowMs)
{
    const uint32_t drogueElapsedMs = nowMs - flightState_.drogueMs;
    if (barometerPrimaryUsable(nowMs) &&
        telemetryState_.barometer.lastUpdatedMs != lastBarometerSampleMs_)
    {
        lastBarometerSampleMs_ = telemetryState_.barometer.lastUpdatedMs;
        trackBarometerStuck(telemetryState_.barometer.lastUpdatedMs,
                            telemetryState_.barometer.altitudeM,
                            nowMs);
    }

    const bool mainAltitudeReached = barometerPrimaryUsable(nowMs) &&
                                     telemetryState_.barometer.altitudeM <= NuraConstants::Flight::kMainDeployAltitudeM;
    const bool mainTimerReached = drogueElapsedMs >= NuraConstants::Flight::kMainTimeoutMs;
    recordDecision(FlightDecisionKind::MAIN_DEPLOY,
                   (mainAltitudeReached || mainTimerReached) ? FlightDecisionResult::ACCEPT : FlightDecisionResult::OBSERVE,
                   mainAltitudeReached ? DECISION_REASON_THRESHOLD_MET : (mainTimerReached ? DECISION_REASON_TIMEOUT : DECISION_REASON_THRESHOLD_NOT_MET),
                   nowMs,
                   telemetryState_.barometer.altitudeM,
                   NuraConstants::Flight::kMainDeployAltitudeM,
                   static_cast<float>(drogueElapsedMs),
                   static_cast<float>(NuraConstants::Flight::kMainTimeoutMs),
                   0U,
                   0U);
    if (mainAltitudeReached || mainTimerReached)
    {
        transitionTo(State::DEPLOY, nowMs);
    }
}

void FlightStateMachineTask::tickDeploy(uint32_t nowMs)
{
    const uint32_t elapsedMs = nowMs - flightState_.deployMs;
    if (!mainPyroOff_ && elapsedMs >= NuraConstants::Flight::kPyroFireDurationMs)
    {
        // TODO: drive main pyro OFF when pyro HAL/pinmap is defined.
        mainPyroOff_ = true;
        flightState_.mainSequenceComplete = true;
    }

    if (!flightState_.mainSequenceComplete)
    {
        return;
    }

    if (consumeLandingSample())
    {
        const bool stable = landingStable();
        recordDecision(FlightDecisionKind::LANDING,
                       stable ? FlightDecisionResult::ACCEPT : FlightDecisionResult::OBSERVE,
                       stable ? DECISION_REASON_CONFIRMATION_MET : DECISION_REASON_THRESHOLD_NOT_MET,
                       telemetryState_.barometer.lastUpdatedMs,
                       telemetryState_.barometer.altitudeM,
                       NuraConstants::Flight::kLandingStableAltitudeRangeM,
                       static_cast<float>(landingSampleCount_),
                       static_cast<float>(NuraConstants::Flight::kLandingStableWindowSamples),
                       landingSampleCount_,
                       0U);
        if (stable)
        {
            transitionTo(State::GROUND, telemetryState_.barometer.lastUpdatedMs);
        }
    }
}

void FlightStateMachineTask::transitionTo(State next, uint32_t nowMs)
{
    if (flightState_.state == next)
    {
        return;
    }

    LOGI(logger_, nowMs, "fsm", stateName(next));
    flightState_.state = next;
    flightState_.stateEnteredMs = nowMs;
    onEnter(next, nowMs);
}

bool FlightStateMachineTask::consumeForceRecoveryDeployRequest(uint32_t nowMs)
{
    if (!flightState_.forceRecoveryDeployRequested)
    {
        return false;
    }

    const uint16_t requestSeq = flightState_.forceRecoveryDeployRequestSeq;
    flightState_.forceRecoveryDeployRequested = false;

    if (!forceRecoveryDeployAllowed())
    {
        recordDecision(FlightDecisionKind::FORCE_DEPLOY,
                       FlightDecisionResult::REJECT,
                       DECISION_REASON_FORCED,
                       nowMs,
                       static_cast<float>(requestSeq),
                       static_cast<float>(flightState_.stateEnteredMs),
                       0.0f,
                       0.0f,
                       0U,
                       0U);
        LOGW(logger_, nowMs, "fsm", "force deploy rejected by state");
        return false;
    }

    flightState_.forceRecoveryDeployExecuted = true;
    flightState_.forceRecoveryDeployExecutedSeq = requestSeq;
    recordDecision(FlightDecisionKind::FORCE_DEPLOY,
                   FlightDecisionResult::ACCEPT,
                   DECISION_REASON_FORCED,
                   nowMs,
                   static_cast<float>(requestSeq),
                   static_cast<float>(flightState_.stateEnteredMs),
                   0.0f,
                   0.0f,
                   0U,
                   0U);
    transitionTo(State::APOGEE, nowMs);
    return true;
}

bool FlightStateMachineTask::forceRecoveryDeployAllowed() const
{
    return stateAllowsForceRecoveryDeploy(flightState_.state);
}

void FlightStateMachineTask::recordDecision(FlightDecisionKind kind,
                                            FlightDecisionResult result,
                                            uint16_t reason,
                                            uint32_t timestampMs,
                                            float value0,
                                            float value1,
                                            float value2,
                                            float value3,
                                            uint8_t count0,
                                            uint8_t count1)
{
    FlightDecisionTrace &trace = flightState_.decisionTrace;
    ++trace.seq;
    trace.timestampMs = timestampMs;
    trace.state = flightState_.state;
    trace.kind = kind;
    trace.result = result;
    trace.reason = reason;
    trace.value0 = value0;
    trace.value1 = value1;
    trace.value2 = value2;
    trace.value3 = value3;
    trace.count0 = count0;
    trace.count1 = count1;
    flightState_.pushDecisionTrace(trace);
}

void FlightStateMachineTask::onEnter(State next, uint32_t nowMs)
{
    switch (next)
    {
    case State::SAFE:
        resetFlightScratch();
        break;
    case State::ARMED:
        launchConfirmCount_ = 0U;
        lastLaunchAccelSampleMs_ = 0U;
        lastLaunchAccelSource_ = AccelSource::NONE;
        resetApogeeScratch();
        break;
    case State::LAUNCH:
        flightState_.launchMs = nowMs;
        burnoutConfirmCount_ = 0U;
        lastBurnoutAccelSampleMs_ = 0U;
        lastBurnoutAccelSource_ = AccelSource::NONE;
        break;
    case State::COAST:
        flightState_.coastMs = nowMs;
        resetApogeeScratch();
        maxCoastAltitudeM_ = telemetryState_.barometer.altitudeM;
        break;
    case State::APOGEE:
        flightState_.apogeeMs = nowMs;
        primaryDrogueOff_ = false;
        backupDrogueOn_ = false;
        backupDrogueOff_ = false;
        flightState_.drogueSequenceComplete = false;
        telemetryState_.health.deployFired = true;
        // TODO: drive drogue primary pyro ON when pyro HAL/pinmap is defined.
        break;
    case State::DROGUE:
        flightState_.drogueMs = nowMs;
        resetBarometerStuckScratch();
        break;
    case State::DEPLOY:
        flightState_.deployMs = nowMs;
        mainPyroOff_ = false;
        flightState_.mainSequenceComplete = false;
        resetLandingScratch();
        // TODO: drive main pyro ON when pyro HAL/pinmap is defined.
        break;
    case State::GROUND:
    case State::FAULT:
        // TODO: force all pyro outputs OFF when pyro HAL/pinmap is defined.
        break;
    default:
        break;
    }
}

void FlightStateMachineTask::resetFlightScratch()
{
    launchConfirmCount_ = 0U;
    burnoutConfirmCount_ = 0U;
    lastLaunchAccelSampleMs_ = 0U;
    lastBurnoutAccelSampleMs_ = 0U;
    lastLaunchAccelSource_ = AccelSource::NONE;
    lastBurnoutAccelSource_ = AccelSource::NONE;
    primaryDrogueOff_ = false;
    backupDrogueOn_ = false;
    backupDrogueOff_ = false;
    mainPyroOff_ = false;
    resetApogeeScratch();
    resetLandingScratch();
}

void FlightStateMachineTask::resetApogeeScratch()
{
    apogeeConfirmCount_ = 0U;
    descentConfirmCount_ = 0U;
    attitudeFallbackConfirmCount_ = 0U;
    lastBarometerSampleMs_ = 0U;
    lastAttitudeFallbackSampleMs_ = 0U;
    apogeeSampleHead_ = 0U;
    apogeeSampleCount_ = 0U;
    apogeePredictionHead_ = 0U;
    apogeePredictionCount_ = 0U;
    maxCoastAltitudeM_ = 0.0f;
    resetBarometerStuckScratch();
}

void FlightStateMachineTask::resetLandingScratch()
{
    lastLandingBarometerSampleMs_ = 0U;
    landingSampleHead_ = 0U;
    landingSampleCount_ = 0U;
}

bool FlightStateMachineTask::consumeFlightAccelSample(uint32_t nowMs,
                                                      uint32_t &lastSeenMs,
                                                      AccelSource &lastSeenSource,
                                                      AccelSample &sample) const
{
    AccelSample candidate;
    if (!highGAccelSample(nowMs, candidate) && !lowGAccelSample(nowMs, candidate))
    {
        return false;
    }

    if (candidate.source == lastSeenSource && candidate.sampleMs == lastSeenMs)
    {
        return false;
    }

    lastSeenSource = candidate.source;
    lastSeenMs = candidate.sampleMs;
    sample = candidate;
    return true;
}

bool FlightStateMachineTask::highGAccelSample(uint32_t nowMs, AccelSample &sample) const
{
    if (!telemetryState_.health.highAccelOk ||
        !highGImuState_.connected ||
        !highGImuState_.hasNewData ||
        highGImuState_.lastUpdatedMs == 0U ||
        (nowMs - highGImuState_.lastUpdatedMs) > NuraConstants::Flight::kAccelFallbackMaxSampleAgeMs ||
        !finite3(highGImuState_.accelXG, highGImuState_.accelYG, highGImuState_.accelZG))
    {
        return false;
    }

    const float normG = accelNormG(highGImuState_.accelXG, highGImuState_.accelYG, highGImuState_.accelZG);
    if (!isfinite(normG))
    {
        return false;
    }

    sample.source = AccelSource::HIGH_G;
    sample.sampleMs = highGImuState_.lastUpdatedMs;
    sample.normG = normG;
    return true;
}

bool FlightStateMachineTask::lowGAccelSample(uint32_t nowMs, AccelSample &sample) const
{
    const ImuData &imu = imuState_.data;
    if (imu.lastUpdatedMs == 0U ||
        (nowMs - imu.lastUpdatedMs) > NuraConstants::Flight::kAccelFallbackMaxSampleAgeMs ||
        !finite3(imu.accelXMps2, imu.accelYMps2, imu.accelZMps2))
    {
        return false;
    }

    const float normG = accelNormGFromMps2(imu.accelXMps2, imu.accelYMps2, imu.accelZMps2);
    if (!isfinite(normG))
    {
        return false;
    }

    sample.source = AccelSource::LOW_G;
    sample.sampleMs = imu.lastUpdatedMs;
    sample.normG = normG;
    return true;
}

bool FlightStateMachineTask::consumeBarometerSample()
{
    BarometerTelemetryData &baro = telemetryState_.barometer;
    if (baro.fault || !baro.valid || !baro.referenceValid || baro.lastUpdatedMs == 0U ||
        baro.lastUpdatedMs == lastBarometerSampleMs_)
    {
        return false;
    }

    if (lastBarometerSampleMs_ != 0U &&
        (baro.lastUpdatedMs - lastBarometerSampleMs_) > NuraConstants::Flight::kApogeeMaxBarometerSampleGapMs)
    {
        apogeeSampleHead_ = 0U;
        apogeeSampleCount_ = 0U;
        apogeePredictionHead_ = 0U;
        apogeePredictionCount_ = 0U;
        apogeeConfirmCount_ = 0U;
    }

    lastBarometerSampleMs_ = baro.lastUpdatedMs;
    trackBarometerStuck(baro.lastUpdatedMs, baro.altitudeM, baro.lastUpdatedMs);
    if (baro.fault)
    {
        return false;
    }

    pushApogeeSample(baro.lastUpdatedMs, baro.altitudeM);
    return true;
}

bool FlightStateMachineTask::baroFaultAttitudeFallbackReady(uint32_t nowMs)
{
    const uint32_t launchElapsedMs = nowMs - flightState_.launchMs;
    if (launchElapsedMs < NuraConstants::Flight::kBaroFaultAttitudeFallbackMinFlightTimeMs)
    {
        attitudeFallbackConfirmCount_ = 0U;
        return false;
    }

    const ImuData &imu = imuState_.data;
    if (imu.lastUpdatedMs == 0U ||
        imu.lastUpdatedMs == lastAttitudeFallbackSampleMs_)
    {
        return false;
    }

    lastAttitudeFallbackSampleMs_ = imu.lastUpdatedMs;
    if (!imu.tiltValid ||
        (nowMs - imu.lastUpdatedMs) > NuraConstants::Flight::kBaroFaultAttitudeFallbackMaxSampleAgeMs)
    {
        attitudeFallbackConfirmCount_ = 0U;
        return false;
    }

    if (imu.tiltAngleDeg >= NuraConstants::Flight::kBaroFaultAttitudeFallbackTiltDeg)
    {
        ++attitudeFallbackConfirmCount_;
    }
    else
    {
        attitudeFallbackConfirmCount_ = 0U;
    }

    const bool accepted = attitudeFallbackConfirmCount_ >= NuraConstants::Flight::kBaroFaultAttitudeFallbackConfirmSamples;
    recordDecision(FlightDecisionKind::BARO_FAULT_TILT,
                   accepted ? FlightDecisionResult::ACCEPT : FlightDecisionResult::OBSERVE,
                   static_cast<uint16_t>(DECISION_REASON_SENSOR_FAULT |
                                         (accepted ? DECISION_REASON_CONFIRMATION_MET : DECISION_REASON_THRESHOLD_NOT_MET)),
                   imu.lastUpdatedMs,
                   imu.tiltAngleDeg,
                   NuraConstants::Flight::kBaroFaultAttitudeFallbackTiltDeg,
                   static_cast<float>(launchElapsedMs),
                   static_cast<float>(NuraConstants::Flight::kBaroFaultAttitudeFallbackMinFlightTimeMs),
                   attitudeFallbackConfirmCount_,
                   0U);

    return attitudeFallbackConfirmCount_ >= NuraConstants::Flight::kBaroFaultAttitudeFallbackConfirmSamples;
}

bool FlightStateMachineTask::barometerPrimaryUsable(uint32_t nowMs) const
{
    const BarometerTelemetryData &baro = telemetryState_.barometer;
    return !baro.fault &&
           baro.valid &&
           baro.referenceValid &&
           baro.lastUpdatedMs != 0U &&
           (nowMs - baro.lastUpdatedMs) <= NuraConstants::Flight::kApogeeMaxBarometerSampleGapMs;
}

void FlightStateMachineTask::resetBarometerStuckScratch()
{
    barometerStuckWindowStartMs_ = 0U;
    lastBarometerStuckSampleMs_ = 0U;
    barometerStuckMinAltitudeM_ = 0.0f;
    barometerStuckMaxAltitudeM_ = 0.0f;
    barometerStuckWindowActive_ = false;
}

void FlightStateMachineTask::trackBarometerStuck(uint32_t sampleMs, float altitudeM, uint32_t nowMs)
{
    if (!isfinite(altitudeM))
    {
        resetBarometerStuckScratch();
        return;
    }

    if (!barometerStuckWindowActive_ ||
        (lastBarometerStuckSampleMs_ != 0U &&
         (sampleMs - lastBarometerStuckSampleMs_) > NuraConstants::Flight::kApogeeMaxBarometerSampleGapMs))
    {
        barometerStuckWindowActive_ = true;
        barometerStuckWindowStartMs_ = sampleMs;
        barometerStuckMinAltitudeM_ = altitudeM;
        barometerStuckMaxAltitudeM_ = altitudeM;
        lastBarometerStuckSampleMs_ = sampleMs;
        return;
    }

    if (altitudeM < barometerStuckMinAltitudeM_)
    {
        barometerStuckMinAltitudeM_ = altitudeM;
    }
    if (altitudeM > barometerStuckMaxAltitudeM_)
    {
        barometerStuckMaxAltitudeM_ = altitudeM;
    }
    lastBarometerStuckSampleMs_ = sampleMs;

    const float altitudeRangeM = barometerStuckMaxAltitudeM_ - barometerStuckMinAltitudeM_;
    if ((sampleMs - barometerStuckWindowStartMs_) >= NuraConstants::Sensors::kBarometerStuckWindowMs &&
        altitudeRangeM <= NuraConstants::Sensors::kBarometerStuckRangeM)
    {
        markBarometerFault(nowMs, BARO_FAULT_STUCK);
        return;
    }

    if ((sampleMs - barometerStuckWindowStartMs_) >= NuraConstants::Sensors::kBarometerStuckWindowMs)
    {
        barometerStuckWindowStartMs_ = sampleMs;
        barometerStuckMinAltitudeM_ = altitudeM;
        barometerStuckMaxAltitudeM_ = altitudeM;
    }
}

void FlightStateMachineTask::markBarometerFault(uint32_t nowMs, uint16_t faultFlag)
{
    (void)nowMs;
    BarometerTelemetryData &baro = telemetryState_.barometer;
    if ((baro.faultFlags & faultFlag) == 0U)
    {
        LOGW(logger_, nowMs, "fsm", "barometer fault");
    }
    baro.fault = true;
    baro.faultFlags = static_cast<uint16_t>(baro.faultFlags | faultFlag);
    baro.valid = false;
}

void FlightStateMachineTask::pushApogeeSample(uint32_t sampleMs, float altitudeM)
{
    apogeeSamples_[apogeeSampleHead_].sampleMs = sampleMs;
    apogeeSamples_[apogeeSampleHead_].altitudeM = altitudeM;
    apogeeSampleHead_ = static_cast<uint8_t>((apogeeSampleHead_ + 1U) % NuraConstants::Flight::kApogeeFitWindowSamples);
    if (apogeeSampleCount_ < NuraConstants::Flight::kApogeeFitWindowSamples)
    {
        ++apogeeSampleCount_;
    }
}

bool FlightStateMachineTask::consumeLandingSample()
{
    const BarometerTelemetryData &baro = telemetryState_.barometer;
    if (baro.fault || !baro.valid || !baro.referenceValid || baro.lastUpdatedMs == 0U ||
        baro.lastUpdatedMs == lastLandingBarometerSampleMs_)
    {
        return false;
    }

    if (lastLandingBarometerSampleMs_ != 0U &&
        (baro.lastUpdatedMs - lastLandingBarometerSampleMs_) > NuraConstants::Flight::kLandingMaxBarometerSampleGapMs)
    {
        resetLandingScratch();
    }

    lastLandingBarometerSampleMs_ = baro.lastUpdatedMs;
    pushLandingSample(baro.lastUpdatedMs, baro.altitudeM);
    return true;
}

void FlightStateMachineTask::pushLandingSample(uint32_t sampleMs, float altitudeM)
{
    landingSamples_[landingSampleHead_].sampleMs = sampleMs;
    landingSamples_[landingSampleHead_].altitudeM = altitudeM;
    landingSampleHead_ = static_cast<uint8_t>((landingSampleHead_ + 1U) %
                                              NuraConstants::Flight::kLandingStableWindowSamples);
    if (landingSampleCount_ < NuraConstants::Flight::kLandingStableWindowSamples)
    {
        ++landingSampleCount_;
    }
}

bool FlightStateMachineTask::landingStable() const
{
    if (landingSampleCount_ < NuraConstants::Flight::kLandingStableWindowSamples)
    {
        return false;
    }

    float minAltitudeM = landingSamples_[0].altitudeM;
    float maxAltitudeM = landingSamples_[0].altitudeM;
    for (uint8_t i = 1U; i < NuraConstants::Flight::kLandingStableWindowSamples; ++i)
    {
        if (landingSamples_[i].altitudeM < minAltitudeM)
        {
            minAltitudeM = landingSamples_[i].altitudeM;
        }
        if (landingSamples_[i].altitudeM > maxAltitudeM)
        {
            maxAltitudeM = landingSamples_[i].altitudeM;
        }
    }

    return (maxAltitudeM - minAltitudeM) <= NuraConstants::Flight::kLandingStableAltitudeRangeM;
}

bool FlightStateMachineTask::apogeePredictionReady(float currentAltitudeM)
{
    if (apogeeSampleCount_ < NuraConstants::Flight::kApogeeFitWindowSamples ||
        currentAltitudeM < NuraConstants::Flight::kMinApogeeDetectAltM)
    {
        recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                       FlightDecisionResult::REJECT,
                       DECISION_REASON_QUALITY_REJECT,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       static_cast<float>(apogeeSampleCount_),
                       NuraConstants::Flight::kMinApogeeDetectAltM,
                       0.0f,
                       apogeeConfirmCount_,
                       descentConfirmCount_);
        return false;
    }

    ApogeeFit fit;
    if (!solveQuadratic(fit) ||
        fit.a >= -NuraConstants::Flight::kApogeeMinCurvature ||
        fit.a <= -NuraConstants::Flight::kApogeeMaxCurvature ||
        fit.rmseM > NuraConstants::Flight::kApogeeMaxFitRmseM)
    {
        recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                       FlightDecisionResult::REJECT,
                       DECISION_REASON_QUALITY_REJECT,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       fit.a,
                       fit.rmseM,
                       NuraConstants::Flight::kApogeeMaxFitRmseM,
                       apogeeConfirmCount_,
                       descentConfirmCount_);
        return false;
    }

    const uint8_t lastIndex = static_cast<uint8_t>((apogeeSampleHead_ + NuraConstants::Flight::kApogeeFitWindowSamples - 1U) %
                                                  NuraConstants::Flight::kApogeeFitWindowSamples);
    const uint8_t firstIndex = apogeeSampleHead_;
    const float lastT = static_cast<float>(apogeeSamples_[lastIndex].sampleMs - apogeeSamples_[firstIndex].sampleMs) / 1000.0f;
    const float tApogee = -fit.b / (2.0f * fit.a);
    if (!isfinite(tApogee) || tApogee <= lastT ||
        (tApogee - lastT) > NuraConstants::Flight::kApogeeMaxPredictAheadS)
    {
        recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                       FlightDecisionResult::REJECT,
                       DECISION_REASON_QUALITY_REJECT,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       tApogee,
                       lastT,
                       NuraConstants::Flight::kApogeeMaxPredictAheadS,
                       apogeeConfirmCount_,
                       descentConfirmCount_);
        return false;
    }

    const float rawApogeeM = fit.c - ((fit.b * fit.b) / (4.0f * fit.a));
    const float rawMarginM = rawApogeeM - currentAltitudeM;
    if (!isfinite(rawApogeeM) ||
        rawMarginM < 0.0f ||
        rawMarginM > NuraConstants::Flight::kApogeeMaxAltMarginM ||
        !pushApogeePrediction(rawApogeeM))
    {
        recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                       FlightDecisionResult::REJECT,
                       DECISION_REASON_QUALITY_REJECT,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       rawApogeeM,
                       rawMarginM,
                       NuraConstants::Flight::kApogeeMaxAltMarginM,
                       apogeeConfirmCount_,
                       descentConfirmCount_);
        return false;
    }

    float aggregatedApogeeM = 0.0f;
    if (!plusTwoSigmaApogee(aggregatedApogeeM))
    {
        recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                       FlightDecisionResult::REJECT,
                       DECISION_REASON_QUALITY_REJECT,
                       telemetryState_.barometer.lastUpdatedMs,
                       currentAltitudeM,
                       rawApogeeM,
                       rawMarginM,
                       static_cast<float>(apogeePredictionCount_),
                       apogeeConfirmCount_,
                       descentConfirmCount_);
        return false;
    }

    const float aggregatedMarginM = aggregatedApogeeM - currentAltitudeM;
    const bool ready = isfinite(aggregatedApogeeM) &&
                       aggregatedMarginM >= 0.0f &&
                       aggregatedMarginM <= NuraConstants::Flight::kApogeeDeployAltMarginM &&
                       aggregatedMarginM <= NuraConstants::Flight::kApogeeMaxAltMarginM;
    recordDecision(FlightDecisionKind::APOGEE_PREDICTION,
                   ready ? FlightDecisionResult::ACCEPT : FlightDecisionResult::OBSERVE,
                   ready ? DECISION_REASON_THRESHOLD_MET : DECISION_REASON_THRESHOLD_NOT_MET,
                   telemetryState_.barometer.lastUpdatedMs,
                   currentAltitudeM,
                   aggregatedApogeeM,
                   aggregatedMarginM,
                   fit.rmseM,
                   apogeeConfirmCount_,
                   descentConfirmCount_);
    return ready;
}

bool FlightStateMachineTask::pushApogeePrediction(float predictionM)
{
    if (!isfinite(predictionM))
    {
        return false;
    }

    if (apogeePredictionCount_ > 0U)
    {
        const uint8_t lastIndex = static_cast<uint8_t>((apogeePredictionHead_ + NuraConstants::Flight::kApogeePredictionHistorySamples - 1U) %
                                                      NuraConstants::Flight::kApogeePredictionHistorySamples);
        if (fabsf(predictionM - apogeePredictions_[lastIndex]) > NuraConstants::Flight::kApogeeMaxPredictionJumpM)
        {
            return false;
        }
    }

    apogeePredictions_[apogeePredictionHead_] = predictionM;
    apogeePredictionHead_ = static_cast<uint8_t>((apogeePredictionHead_ + 1U) % NuraConstants::Flight::kApogeePredictionHistorySamples);
    if (apogeePredictionCount_ < NuraConstants::Flight::kApogeePredictionHistorySamples)
    {
        ++apogeePredictionCount_;
    }
    return true;
}

bool FlightStateMachineTask::plusTwoSigmaApogee(float &predictionM) const
{
    if (apogeePredictionCount_ < NuraConstants::Flight::kApogeePredictionHistorySamples)
    {
        return false;
    }

    float sum = 0.0f;
    for (uint8_t i = 0U; i < NuraConstants::Flight::kApogeePredictionHistorySamples; ++i)
    {
        sum += apogeePredictions_[i];
    }
    const float mean = sum / static_cast<float>(NuraConstants::Flight::kApogeePredictionHistorySamples);

    float variance = 0.0f;
    for (uint8_t i = 0U; i < NuraConstants::Flight::kApogeePredictionHistorySamples; ++i)
    {
        const float error = apogeePredictions_[i] - mean;
        variance += error * error;
    }
    variance /= static_cast<float>(NuraConstants::Flight::kApogeePredictionHistorySamples);

    const float sigma = sqrtf(variance);
    if (!isfinite(mean) || !isfinite(sigma) || sigma > NuraConstants::Flight::kApogeeMaxPredictionSigmaM)
    {
        return false;
    }

    predictionM = mean + (NuraConstants::Flight::kApogeeAggregationSigmaMultiplier * sigma);
    return isfinite(predictionM);
}

bool FlightStateMachineTask::solveQuadratic(ApogeeFit &fit) const
{
    const uint8_t n = NuraConstants::Flight::kApogeeFitWindowSamples;
    const uint8_t firstIndex = apogeeSampleHead_;
    const uint32_t t0 = apogeeSamples_[firstIndex].sampleMs;

    float s0 = static_cast<float>(n);
    float s1 = 0.0f;
    float s2 = 0.0f;
    float s3 = 0.0f;
    float s4 = 0.0f;
    float y0 = 0.0f;
    float y1 = 0.0f;
    float y2 = 0.0f;

    for (uint8_t i = 0U; i < n; ++i)
    {
        const uint8_t index = static_cast<uint8_t>((firstIndex + i) % n);
        const float x = static_cast<float>(apogeeSamples_[index].sampleMs - t0) / 1000.0f;
        const float y = apogeeSamples_[index].altitudeM;
        const float x2 = x * x;
        s1 += x;
        s2 += x2;
        s3 += x2 * x;
        s4 += x2 * x2;
        y0 += y;
        y1 += x * y;
        y2 += x2 * y;
    }

    float matrix[3][4] = {
        {s4, s3, s2, y2},
        {s3, s2, s1, y1},
        {s2, s1, s0, y0},
    };
    if (!solve3x3(matrix, fit.a, fit.b, fit.c))
    {
        return false;
    }

    float squaredErrorSum = 0.0f;
    for (uint8_t i = 0U; i < n; ++i)
    {
        const uint8_t index = static_cast<uint8_t>((firstIndex + i) % n);
        const float x = static_cast<float>(apogeeSamples_[index].sampleMs - t0) / 1000.0f;
        const float predictedY = (fit.a * x * x) + (fit.b * x) + fit.c;
        const float error = apogeeSamples_[index].altitudeM - predictedY;
        squaredErrorSum += error * error;
    }
    fit.rmseM = sqrtf(squaredErrorSum / static_cast<float>(n));
    return isfinite(fit.rmseM);
}

float FlightStateMachineTask::accelNormG(float xG, float yG, float zG)
{
    return sqrtf((xG * xG) + (yG * yG) + (zG * zG));
}

float FlightStateMachineTask::accelNormGFromMps2(float xMps2, float yMps2, float zMps2)
{
    const float normMps2 = sqrtf((xMps2 * xMps2) + (yMps2 * yMps2) + (zMps2 * zMps2));
    return normMps2 / NuraConstants::Physics::kGravityMps2;
}

bool FlightStateMachineTask::finite3(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z);
}

bool FlightStateMachineTask::solve3x3(float matrix[3][4], float &x0, float &x1, float &x2)
{
    constexpr float kEpsilon = 1.0e-6f;

    for (uint8_t col = 0U; col < 3U; ++col)
    {
        uint8_t pivot = col;
        float best = fabsf(matrix[col][col]);
        for (uint8_t row = static_cast<uint8_t>(col + 1U); row < 3U; ++row)
        {
            const float candidate = fabsf(matrix[row][col]);
            if (candidate > best)
            {
                best = candidate;
                pivot = row;
            }
        }

        if (best < kEpsilon)
        {
            return false;
        }

        if (pivot != col)
        {
            for (uint8_t j = col; j < 4U; ++j)
            {
                const float temp = matrix[col][j];
                matrix[col][j] = matrix[pivot][j];
                matrix[pivot][j] = temp;
            }
        }

        const float divisor = matrix[col][col];
        for (uint8_t j = col; j < 4U; ++j)
        {
            matrix[col][j] /= divisor;
        }

        for (uint8_t row = 0U; row < 3U; ++row)
        {
            if (row == col)
            {
                continue;
            }
            const float factor = matrix[row][col];
            for (uint8_t j = col; j < 4U; ++j)
            {
                matrix[row][j] -= factor * matrix[col][j];
            }
        }
    }

    x0 = matrix[0][3];
    x1 = matrix[1][3];
    x2 = matrix[2][3];
    return isfinite(x0) && isfinite(x1) && isfinite(x2);
}
