#pragma once

#include <stdint.h>

namespace MissionConstants
{
constexpr float kLaunchAccelThresholdG = 2.0f;
constexpr uint8_t kLaunchConfirmSamples = 4U;

constexpr float kBurnoutAccelThresholdG = 1.0f;
constexpr uint8_t kBurnoutConfirmSamples = 4U;

constexpr uint8_t kApogeeFitWindowSamples = 9U;
constexpr uint8_t kApogeePredictionHistorySamples = 5U;
constexpr uint8_t kApogeeConfirmSamples = 3U;
constexpr uint32_t kApogeeMinFlightTimeMs = 8000UL;
constexpr float kApogeeMaxPredictAheadS = 1.0f;
constexpr float kApogeeDeployAltMarginM = 3.0f;
constexpr float kApogeeMaxAltMarginM = 20.0f;
constexpr float kApogeeMinCurvature = 0.05f;
constexpr float kApogeeMaxCurvature = 120.0f;
constexpr float kApogeeMaxFitRmseM = 2.5f;
constexpr float kApogeeMaxPredictionJumpM = 15.0f;
constexpr float kApogeeMaxPredictionSigmaM = 8.0f;
constexpr float kApogeeAggregationSigmaMultiplier = 2.0f;
constexpr uint32_t kApogeeMaxBarometerSampleGapMs = 150UL;
constexpr float kMinApogeeDetectAltM = 30.0f;
constexpr float kApogeeDropThresholdM = 4.0f;
constexpr uint8_t kApogeeDescentConfirmSamples = 4U;

constexpr uint32_t kApogeeTimeoutMs = 12000UL;
constexpr uint32_t kMainTimeoutMs = 15000UL;
constexpr uint32_t kGroundTimeoutMs = 60000UL;

constexpr uint32_t kPyroFireDurationMs = 50UL;
constexpr uint32_t kDrogueBackupDelayMs = 2000UL;
constexpr float kMainDeployAltitudeM = 200.0f;
} // namespace MissionConstants
