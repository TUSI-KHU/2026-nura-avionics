#pragma once

#include <cstdint>

namespace nura::flight
{

// Frozen migration policy. Do not tune these values as part of the RTOS port.
// Every behavioral change requires an updated flight-logic document and
// replay/bench evidence.
struct FlightPolicy
{
    static constexpr uint32_t kProfileRevision = 1U;
    static constexpr float kGravityMps2 = 9.80665f;

    static constexpr uint32_t kAccelFallbackMaxSampleAgeMs = 50U;
    static constexpr float kLaunchAccelThresholdG = 2.0f;
    static constexpr uint8_t kLaunchConfirmSamples = 4U;
    static constexpr float kBurnoutAccelThresholdG = 1.0f;
    static constexpr uint8_t kBurnoutConfirmSamples = 4U;

    static constexpr uint8_t kApogeeFitWindowSamples = 9U;
    static constexpr uint8_t kApogeePredictionHistorySamples = 5U;
    static constexpr uint8_t kApogeeConfirmSamples = 3U;
    static constexpr uint32_t kApogeeMinFlightTimeMs = 8000U;
    static constexpr float kApogeeMaxPredictAheadS = 1.0f;
    static constexpr float kApogeeDeployAltMarginM = 3.0f;
    static constexpr float kApogeeMaxAltMarginM = 20.0f;
    static constexpr float kApogeeMinCurvature = 0.05f;
    static constexpr float kApogeeMaxCurvature = 120.0f;
    static constexpr float kApogeeMaxFitRmseM = 2.5f;
    static constexpr float kApogeeMaxPredictionJumpM = 15.0f;
    static constexpr float kApogeeMaxPredictionSigmaM = 8.0f;
    static constexpr float kApogeeAggregationSigmaMultiplier = 2.0f;
    static constexpr uint32_t kApogeeMaxBarometerSampleGapMs = 150U;
    static constexpr float kMinApogeeDetectAltM = 30.0f;
    static constexpr float kApogeeDropThresholdM = 4.0f;
    static constexpr uint8_t kApogeeDescentConfirmSamples = 4U;
    static constexpr uint32_t kBaroFaultAttitudeFallbackMinFlightTimeMs = 8000U;
    static constexpr float kBaroFaultAttitudeFallbackTiltDeg = 70.0f;
    static constexpr uint8_t kBaroFaultAttitudeFallbackConfirmSamples = 5U;
    static constexpr uint32_t kBaroFaultAttitudeFallbackMaxSampleAgeMs = 150U;

    static constexpr uint32_t kApogeeTimeoutMs = 12000U;
    static constexpr uint32_t kMainTimeoutMs = 15000U;

    // Current approved mission policy: 1000 ms. Hardware qualification remains
    // mandatory before flight, but do not change this duration without a
    // documented flight-logic change and new verification evidence.
    static constexpr uint32_t kPyroFireDurationMs = 1000U;
    static constexpr uint32_t kDrogueBackupDelayMs = 2000U;
    static constexpr float kMainDeployAltitudeM = 200.0f;
    static constexpr uint8_t kLandingStableWindowSamples = 20U;
    static constexpr float kLandingStableAltitudeRangeM = 0.5f;
    static constexpr uint32_t kLandingMaxBarometerSampleGapMs = 150U;

    static constexpr uint32_t kBarometerStuckWindowMs = 5000U;
    static constexpr float kBarometerStuckRangeM = 0.2f;
};

struct RecoveryTimingPolicy
{
    uint32_t pyro_fire_duration_ms;
    uint32_t drogue_backup_delay_ms;
};

inline constexpr RecoveryTimingPolicy kRecoveryTimingPolicy{
    FlightPolicy::kPyroFireDurationMs,
    FlightPolicy::kDrogueBackupDelayMs,
};

} // namespace nura::flight
