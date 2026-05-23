#pragma once

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/tasks.h"
#include "nura_constants.h"
#include "state/abort_state.h"
#include "state/flight_state.h"
#include "state/high_g_imu_state.h"
#include "state/telemetry_state.h"
#include "hal/panic_handler.h"

class FlightStateMachineTask : public Task
{
public:
    // 전체 시스템 상태 전이를 담당하는 상태 머신 태스크다.
    FlightStateMachineTask(FlightState &flightState,
                           AbortState &abortState,
                           HighGImuState &highGImuState,
                           TelemetryState &telemetryState,
                           Logger &logger,
                           const IAppConfig &config,
                           IPanicHandler &panicHandler);

    const char *name() const;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    struct ApogeeSample
    {
        uint32_t sampleMs = 0;
        float altitudeM = 0.0f;
    };

    struct ApogeeFit
    {
        float a = 0.0f;
        float b = 0.0f;
        float c = 0.0f;
        float rmseM = 0.0f;
    };

    struct LandingSample
    {
        uint32_t sampleMs = 0;
        float altitudeM = 0.0f;
    };

    void resetFlightScratch();
    void resetApogeeScratch();
    void resetLandingScratch();
    void onEnter(State next, uint32_t nowMs);
    void tickArmed(uint32_t nowMs);
    void tickLaunch(uint32_t nowMs);
    void tickCoast(uint32_t nowMs);
    void tickApogee(uint32_t nowMs);
    void tickDrogue(uint32_t nowMs);
    void tickDeploy(uint32_t nowMs);
    void transitionTo(State next, uint32_t nowMs);
    bool consumeForceRecoveryDeployRequest(uint32_t nowMs);
    bool forceRecoveryDeployAllowed() const;
    float highGAccelNorm() const;
    bool consumeHighGSample(uint32_t &lastSeenMs);
    bool consumeBarometerSample();
    void pushApogeeSample(uint32_t sampleMs, float altitudeM);
    bool consumeLandingSample();
    void pushLandingSample(uint32_t sampleMs, float altitudeM);
    bool landingStable() const;
    bool apogeePredictionReady(float currentAltitudeM);
    bool pushApogeePrediction(float predictionM);
    bool plusTwoSigmaApogee(float &predictionM) const;
    bool solveQuadratic(ApogeeFit &fit) const;
    static bool solve3x3(float matrix[3][4], float &x0, float &x1, float &x2);

    FlightState &flightState_;
    AbortState &abortState_;
    HighGImuState &highGImuState_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
    IPanicHandler &panicHandler_;

    uint8_t launchConfirmCount_ = 0U;
    uint8_t burnoutConfirmCount_ = 0U;
    uint8_t apogeeConfirmCount_ = 0U;
    uint8_t descentConfirmCount_ = 0U;
    uint32_t lastLaunchSampleMs_ = 0U;
    uint32_t lastBurnoutSampleMs_ = 0U;
    uint32_t lastBarometerSampleMs_ = 0U;
    uint32_t lastLandingBarometerSampleMs_ = 0U;
    ApogeeSample apogeeSamples_[NuraConstants::Flight::kApogeeFitWindowSamples];
    float apogeePredictions_[NuraConstants::Flight::kApogeePredictionHistorySamples] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    LandingSample landingSamples_[NuraConstants::Flight::kLandingStableWindowSamples];
    uint8_t apogeeSampleHead_ = 0U;
    uint8_t apogeeSampleCount_ = 0U;
    uint8_t apogeePredictionHead_ = 0U;
    uint8_t apogeePredictionCount_ = 0U;
    uint8_t landingSampleHead_ = 0U;
    uint8_t landingSampleCount_ = 0U;
    float maxCoastAltitudeM_ = 0.0f;
    bool primaryDrogueOff_ = false;
    bool backupDrogueOn_ = false;
    bool backupDrogueOff_ = false;
    bool mainPyroOff_ = false;
};
