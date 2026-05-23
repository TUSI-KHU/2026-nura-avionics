#include "fsm_task.h"

#include <math.h>

FlightStateMachineTask::FlightStateMachineTask(FlightState &flightState,
                                               AbortState &abortState,
                                               HighGImuState &highGImuState,
                                               TelemetryState &telemetryState,
                                               Logger &logger,
                                               const IAppConfig &config,
                                               IPanicHandler &panicHandler)
    : flightState_(flightState),
      abortState_(abortState),
      highGImuState_(highGImuState),
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

uint32_t FlightStateMachineTask::periodMs() const
{
    return config_.flightStateTaskPeriodMs();
}

void FlightStateMachineTask::tickArmed(uint32_t nowMs)
{
    (void)nowMs;
    if (!consumeHighGSample(lastLaunchSampleMs_))
    {
        return;
    }

    if (highGAccelNorm() >= MissionConstants::kLaunchAccelThresholdG)
    {
        ++launchConfirmCount_;
    }
    else
    {
        launchConfirmCount_ = 0U;
    }

    if (launchConfirmCount_ >= MissionConstants::kLaunchConfirmSamples)
    {
        transitionTo(State::LAUNCH, highGImuState_.lastUpdatedMs);
    }
}

void FlightStateMachineTask::tickLaunch(uint32_t nowMs)
{
    (void)nowMs;
    if (!consumeHighGSample(lastBurnoutSampleMs_))
    {
        return;
    }

    if (highGAccelNorm() < MissionConstants::kBurnoutAccelThresholdG)
    {
        ++burnoutConfirmCount_;
    }
    else
    {
        burnoutConfirmCount_ = 0U;
    }

    if (burnoutConfirmCount_ >= MissionConstants::kBurnoutConfirmSamples)
    {
        transitionTo(State::COAST, highGImuState_.lastUpdatedMs);
    }
}

void FlightStateMachineTask::tickCoast(uint32_t nowMs)
{
    const uint32_t coastElapsedMs = nowMs - flightState_.coastMs;
    if (coastElapsedMs >= MissionConstants::kApogeeTimeoutMs)
    {
        transitionTo(State::APOGEE, nowMs);
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
    const bool allowApogee = launchElapsedMs >= MissionConstants::kApogeeMinFlightTimeMs;
    if (!allowApogee)
    {
        apogeeConfirmCount_ = 0U;
        descentConfirmCount_ = 0U;
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

    if ((maxCoastAltitudeM_ - currentAltitudeM) >= MissionConstants::kApogeeDropThresholdM)
    {
        ++descentConfirmCount_;
    }
    else
    {
        descentConfirmCount_ = 0U;
    }

    if (apogeeConfirmCount_ >= MissionConstants::kApogeeConfirmSamples ||
        descentConfirmCount_ >= MissionConstants::kApogeeDescentConfirmSamples)
    {
        transitionTo(State::APOGEE, telemetryState_.barometer.lastUpdatedMs);
    }
}

void FlightStateMachineTask::tickApogee(uint32_t nowMs)
{
    const uint32_t elapsedMs = nowMs - flightState_.apogeeMs;

    if (!primaryDrogueOff_ && elapsedMs >= MissionConstants::kPyroFireDurationMs)
    {
        // TODO: drive drogue primary pyro OFF when pyro HAL/pinmap is defined.
        primaryDrogueOff_ = true;
    }

    if (!backupDrogueOn_ && elapsedMs >= MissionConstants::kDrogueBackupDelayMs)
    {
        // TODO: drive drogue backup pyro ON when pyro HAL/pinmap is defined.
        backupDrogueOn_ = true;
    }

    if (backupDrogueOn_ && !backupDrogueOff_ &&
        elapsedMs >= (MissionConstants::kDrogueBackupDelayMs + MissionConstants::kPyroFireDurationMs))
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
    const bool mainAltitudeReached = telemetryState_.barometer.valid &&
                                     telemetryState_.barometer.referenceValid &&
                                     telemetryState_.barometer.altitudeM <= MissionConstants::kMainDeployAltitudeM;
    if (mainAltitudeReached || drogueElapsedMs >= MissionConstants::kMainTimeoutMs)
    {
        transitionTo(State::DEPLOY, nowMs);
    }
}

void FlightStateMachineTask::tickDeploy(uint32_t nowMs)
{
    const uint32_t elapsedMs = nowMs - flightState_.deployMs;
    if (!mainPyroOff_ && elapsedMs >= MissionConstants::kPyroFireDurationMs)
    {
        // TODO: drive main pyro OFF when pyro HAL/pinmap is defined.
        mainPyroOff_ = true;
        flightState_.mainSequenceComplete = true;
        transitionTo(State::GROUND, nowMs);
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
        LOGW(logger_, nowMs, "fsm", "force deploy rejected by state");
        return false;
    }

    flightState_.forceRecoveryDeployExecuted = true;
    flightState_.forceRecoveryDeployExecutedSeq = requestSeq;
    transitionTo(State::APOGEE, nowMs);
    return true;
}

bool FlightStateMachineTask::forceRecoveryDeployAllowed() const
{
    return stateAllowsForceRecoveryDeploy(flightState_.state);
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
        lastLaunchSampleMs_ = 0U;
        resetApogeeScratch();
        break;
    case State::LAUNCH:
        flightState_.launchMs = nowMs;
        burnoutConfirmCount_ = 0U;
        lastBurnoutSampleMs_ = 0U;
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
        break;
    case State::DEPLOY:
        flightState_.deployMs = nowMs;
        mainPyroOff_ = false;
        flightState_.mainSequenceComplete = false;
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
    lastLaunchSampleMs_ = 0U;
    lastBurnoutSampleMs_ = 0U;
    primaryDrogueOff_ = false;
    backupDrogueOn_ = false;
    backupDrogueOff_ = false;
    mainPyroOff_ = false;
    resetApogeeScratch();
}

void FlightStateMachineTask::resetApogeeScratch()
{
    apogeeConfirmCount_ = 0U;
    descentConfirmCount_ = 0U;
    lastBarometerSampleMs_ = 0U;
    apogeeSampleHead_ = 0U;
    apogeeSampleCount_ = 0U;
    apogeePredictionHead_ = 0U;
    apogeePredictionCount_ = 0U;
    maxCoastAltitudeM_ = 0.0f;
}

float FlightStateMachineTask::highGAccelNorm() const
{
    const float x = highGImuState_.accelXG;
    const float y = highGImuState_.accelYG;
    const float z = highGImuState_.accelZG;
    return sqrtf((x * x) + (y * y) + (z * z));
}

bool FlightStateMachineTask::consumeHighGSample(uint32_t &lastSeenMs)
{
    if (!highGImuState_.hasNewData || highGImuState_.lastUpdatedMs == 0U ||
        highGImuState_.lastUpdatedMs == lastSeenMs)
    {
        return false;
    }

    lastSeenMs = highGImuState_.lastUpdatedMs;
    return true;
}

bool FlightStateMachineTask::consumeBarometerSample()
{
    const BarometerTelemetryData &baro = telemetryState_.barometer;
    if (!baro.valid || !baro.referenceValid || baro.lastUpdatedMs == 0U ||
        baro.lastUpdatedMs == lastBarometerSampleMs_)
    {
        return false;
    }

    if (lastBarometerSampleMs_ != 0U &&
        (baro.lastUpdatedMs - lastBarometerSampleMs_) > MissionConstants::kApogeeMaxBarometerSampleGapMs)
    {
        apogeeSampleHead_ = 0U;
        apogeeSampleCount_ = 0U;
        apogeePredictionHead_ = 0U;
        apogeePredictionCount_ = 0U;
        apogeeConfirmCount_ = 0U;
    }

    lastBarometerSampleMs_ = baro.lastUpdatedMs;
    pushApogeeSample(baro.lastUpdatedMs, baro.altitudeM);
    return true;
}

void FlightStateMachineTask::pushApogeeSample(uint32_t sampleMs, float altitudeM)
{
    apogeeSamples_[apogeeSampleHead_].sampleMs = sampleMs;
    apogeeSamples_[apogeeSampleHead_].altitudeM = altitudeM;
    apogeeSampleHead_ = static_cast<uint8_t>((apogeeSampleHead_ + 1U) % MissionConstants::kApogeeFitWindowSamples);
    if (apogeeSampleCount_ < MissionConstants::kApogeeFitWindowSamples)
    {
        ++apogeeSampleCount_;
    }
}

bool FlightStateMachineTask::apogeePredictionReady(float currentAltitudeM)
{
    if (apogeeSampleCount_ < MissionConstants::kApogeeFitWindowSamples ||
        currentAltitudeM < MissionConstants::kMinApogeeDetectAltM)
    {
        return false;
    }

    ApogeeFit fit;
    if (!solveQuadratic(fit) ||
        fit.a >= -MissionConstants::kApogeeMinCurvature ||
        fit.a <= -MissionConstants::kApogeeMaxCurvature ||
        fit.rmseM > MissionConstants::kApogeeMaxFitRmseM)
    {
        return false;
    }

    const uint8_t lastIndex = static_cast<uint8_t>((apogeeSampleHead_ + MissionConstants::kApogeeFitWindowSamples - 1U) %
                                                  MissionConstants::kApogeeFitWindowSamples);
    const uint8_t firstIndex = apogeeSampleHead_;
    const float lastT = static_cast<float>(apogeeSamples_[lastIndex].sampleMs - apogeeSamples_[firstIndex].sampleMs) / 1000.0f;
    const float tApogee = -fit.b / (2.0f * fit.a);
    if (!isfinite(tApogee) || tApogee <= lastT ||
        (tApogee - lastT) > MissionConstants::kApogeeMaxPredictAheadS)
    {
        return false;
    }

    const float rawApogeeM = fit.c - ((fit.b * fit.b) / (4.0f * fit.a));
    const float rawMarginM = rawApogeeM - currentAltitudeM;
    if (!isfinite(rawApogeeM) ||
        rawMarginM < 0.0f ||
        rawMarginM > MissionConstants::kApogeeMaxAltMarginM ||
        !pushApogeePrediction(rawApogeeM))
    {
        return false;
    }

    float aggregatedApogeeM = 0.0f;
    if (!plusTwoSigmaApogee(aggregatedApogeeM))
    {
        return false;
    }

    const float aggregatedMarginM = aggregatedApogeeM - currentAltitudeM;
    return isfinite(aggregatedApogeeM) &&
           aggregatedMarginM >= 0.0f &&
           aggregatedMarginM <= MissionConstants::kApogeeDeployAltMarginM &&
           aggregatedMarginM <= MissionConstants::kApogeeMaxAltMarginM;
}

bool FlightStateMachineTask::pushApogeePrediction(float predictionM)
{
    if (!isfinite(predictionM))
    {
        return false;
    }

    if (apogeePredictionCount_ > 0U)
    {
        const uint8_t lastIndex = static_cast<uint8_t>((apogeePredictionHead_ + MissionConstants::kApogeePredictionHistorySamples - 1U) %
                                                      MissionConstants::kApogeePredictionHistorySamples);
        if (fabsf(predictionM - apogeePredictions_[lastIndex]) > MissionConstants::kApogeeMaxPredictionJumpM)
        {
            return false;
        }
    }

    apogeePredictions_[apogeePredictionHead_] = predictionM;
    apogeePredictionHead_ = static_cast<uint8_t>((apogeePredictionHead_ + 1U) % MissionConstants::kApogeePredictionHistorySamples);
    if (apogeePredictionCount_ < MissionConstants::kApogeePredictionHistorySamples)
    {
        ++apogeePredictionCount_;
    }
    return true;
}

bool FlightStateMachineTask::plusTwoSigmaApogee(float &predictionM) const
{
    if (apogeePredictionCount_ < MissionConstants::kApogeePredictionHistorySamples)
    {
        return false;
    }

    float sum = 0.0f;
    for (uint8_t i = 0U; i < MissionConstants::kApogeePredictionHistorySamples; ++i)
    {
        sum += apogeePredictions_[i];
    }
    const float mean = sum / static_cast<float>(MissionConstants::kApogeePredictionHistorySamples);

    float variance = 0.0f;
    for (uint8_t i = 0U; i < MissionConstants::kApogeePredictionHistorySamples; ++i)
    {
        const float error = apogeePredictions_[i] - mean;
        variance += error * error;
    }
    variance /= static_cast<float>(MissionConstants::kApogeePredictionHistorySamples);

    const float sigma = sqrtf(variance);
    if (!isfinite(mean) || !isfinite(sigma) || sigma > MissionConstants::kApogeeMaxPredictionSigmaM)
    {
        return false;
    }

    predictionM = mean + (MissionConstants::kApogeeAggregationSigmaMultiplier * sigma);
    return isfinite(predictionM);
}

bool FlightStateMachineTask::solveQuadratic(ApogeeFit &fit) const
{
    const uint8_t n = MissionConstants::kApogeeFitWindowSamples;
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
