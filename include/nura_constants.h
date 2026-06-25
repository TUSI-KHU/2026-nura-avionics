#pragma once

#include <stdint.h>

#if __has_include("nura_radio_secrets.h")
#include "nura_radio_secrets.h"
#define NURA_RADIO_IDENTITY_PROVISIONED 1
#else
#define NURA_RADIO_VEHICLE_ID 0x4E555241UL
#define NURA_RADIO_AUTH_KEY_BYTES                                              \
    0x4e, 0x55, 0x52, 0x41, 0x2d, 0x56, 0x31, 0x4c,                         \
        0x49, 0x54, 0x45, 0x2d, 0x54, 0x45, 0x53, 0x54
#define NURA_RADIO_IDENTITY_PROVISIONED 0
#endif

namespace NuraConstants
{
namespace Physics
{
constexpr float kGravityMps2 = 9.80665f;
constexpr float kRadToDeg = 57.2957795f;
constexpr float kDegToRad = 0.0174532925f;
} // namespace Physics

namespace Atmosphere
{
constexpr float kSeaLevelPressurePa = 101325.0f;
constexpr float kStandardAtmosphereMeters = 44330.0f;
constexpr float kPressureExponent = 0.19029495f;
constexpr float kInversePressureExponent = 5.255f;
} // namespace Atmosphere

namespace App
{
constexpr unsigned long kSerialBaudRate = 115200UL;
constexpr uint16_t kFaultBlinkIntervalMs = 1000U;
constexpr uint32_t kBoardPowerSettleDelayMs = 5000UL;
constexpr uint32_t kBusSettleDelayMs = 250UL;
} // namespace App

namespace Sensors
{
constexpr uint8_t kSensorInitRetryAttempts = 5U;
constexpr uint32_t kSensorInitRetryDelayMs = 150UL;
constexpr uint8_t kImuReadFailureThreshold = 3U;
constexpr uint8_t kImuMaxRecoveryAttempts = 5U;
constexpr uint32_t kImuRecoveryIntervalMs = 1000UL;
constexpr uint32_t kImuTaskPeriodMs = 10UL;
constexpr uint32_t kMagnetometerTaskPeriodMs = 100UL;
constexpr uint32_t kBarometerTaskPeriodMs = 50UL;
constexpr uint32_t kBarometerRecoveryIntervalMs = 1000UL;
constexpr uint8_t kBarometerMedianWindowSamples = 3U;
constexpr float kBarometerAltitudeLpfAlpha = 0.35f;
constexpr uint32_t kBarometerStaleFaultMs = 300UL;
constexpr uint8_t kBarometerConsecutiveReadFailFault = 5U;
constexpr uint8_t kBarometerBadValueTotalFault = 10U;
constexpr uint8_t kBarometerBadValueConsecutiveFault = 5U;
constexpr float kBarometerMinAltitudeAglM = -200.0f;
constexpr float kBarometerMaxAltitudeAglM = 5000.0f;
constexpr uint32_t kBarometerStuckWindowMs = 5000UL;
constexpr float kBarometerStuckRangeM = 0.2f;
constexpr float kTiltMinAccelNormG = 0.2f;
constexpr float kTiltMaxAccelNormG = 3.0f;
constexpr float kAttitudeAccelCorrectionMinNormG = 0.7f;
constexpr float kAttitudeAccelCorrectionMaxNormG = 1.3f;
constexpr float kAttitudeAccelCorrectionGain = 0.8f;
constexpr uint32_t kAttitudeMaxDeltaMs = 100UL;
constexpr uint32_t kGnssTaskPeriodMs = 50UL;
constexpr uint16_t kGnssPollByteBudget = 128U;
constexpr uint32_t kGnssMaxFixAgeMs = 2000UL;
constexpr uint32_t kHighGSampleLogIntervalMs = 1000UL;
constexpr uint32_t kPowerSenseTaskPeriodMs = 100UL;
constexpr uint16_t kPowerSenseAdcReferenceMv = 3300U;
constexpr uint8_t kPowerSenseAdcResolutionBits = 10U;
constexpr uint16_t kPowerSenseDividerRatioNumerator = 5545U;
constexpr uint16_t kPowerSenseDividerRatioDenominator = 1000U;
constexpr uint16_t kPowerSenseExpectedMinBatteryMv = 11100U;
constexpr uint16_t kPowerSenseExpectedMaxBatteryMv = 12600U;
constexpr uint16_t kPowerSenseExpectedMinRawAdc = 620U;
constexpr uint16_t kPowerSenseExpectedMaxRawAdc = 704U;
constexpr uint16_t kPowerSenseMinValidBatteryMv = 6000U;
constexpr uint16_t kPowerSenseMaxValidBatteryMv = 14000U;
} // namespace Sensors

namespace Tasks
{
constexpr uint32_t kWatchdogTaskPeriodMs = 50UL;
constexpr uint32_t kFlightStateTaskPeriodMs = 10UL;
constexpr uint32_t kLoggerTaskPeriodMs = 20UL;
constexpr uint32_t kTelemetryTaskPeriodMs = 20UL;
} // namespace Tasks

namespace Logger
{
constexpr uint8_t kDrainBudget = 4U;
constexpr uint8_t kOutputFailThreshold = 3U;
constexpr uint8_t kSdInitRetryAttempts = 20U;
constexpr uint32_t kSdInitRetryDelayMs = 250UL;
constexpr uint16_t kFlightLogRamBufferBytes = 16U * 1024U;
constexpr uint32_t kFlightLogProgramFlashBytes = 6UL * 1024UL * 1024UL;
constexpr uint32_t kFlightLogFileSegmentBytes = 256UL * 1024UL;
constexpr uint32_t kFlightLogMinFreeBytes = 64UL * 1024UL;
constexpr uint16_t kFlightLogFastPeriodMs = 20U;
constexpr uint16_t kFlightLogSlowPeriodMs = 100U;
constexpr uint8_t kFlightLogDrainRecordsPerTick = 8U;
constexpr uint8_t kFlightDecisionTraceQueueDepth = 16U;
} // namespace Logger

namespace Diagnostics
{
constexpr uint32_t kHardwareIntegrationExerciseMs = 12000UL;
constexpr float kHardwareIntegrationMinGroundPressurePa = 80000.0f;
constexpr float kHardwareIntegrationMaxGroundPressurePa = 110000.0f;
constexpr uint32_t kGnssPrintPeriodMs = 1000UL;
} // namespace Diagnostics

namespace Telemetry
{
constexpr uint32_t kFastPeriodMs = 200UL;
constexpr uint32_t kGpsPeriodMs = 1000UL;
constexpr uint32_t kSensorFreshMs = 1500UL;
constexpr uint8_t kAckQueueDepth = 4U;
constexpr uint8_t kRecentCommandDepth = 4U;
constexpr uint32_t kVehicleId = NURA_RADIO_VEHICLE_ID;
constexpr uint8_t kControlAuthKey[16] = {
    NURA_RADIO_AUTH_KEY_BYTES};
constexpr bool kRadioIdentityProvisioned = NURA_RADIO_IDENTITY_PROVISIONED != 0;
} // namespace Telemetry

namespace Buzzer
{
constexpr uint16_t kToneFrequencyHz = 2400U;
constexpr uint16_t kInitSafeToneFrequencyHz = 2200U;
constexpr uint16_t kArmedAlertToneFrequencyHz = 3000U;
constexpr uint16_t kInitSafeBeepMs = 90U;
constexpr uint16_t kInitSafeGapMs = 80U;
constexpr uint8_t kInitSafeBeepCount = 7U;
constexpr uint32_t kArmedAlertDurationMs = 5000UL;
constexpr uint16_t kTransitionBeepMs = 80U;
constexpr uint16_t kTransitionGapMs = 80U;
constexpr uint8_t kTransitionBeepCount = 5U;
} // namespace Buzzer

namespace Panic
{
constexpr uint16_t kFailureToneFrequencyHz = 3200U;
constexpr uint16_t kFailureBeepMs = 90U;
constexpr uint16_t kFailureGapMs = 90U;
constexpr uint16_t kFailureRepeatPauseMs = 1000U;
constexpr uint8_t kDefaultFailureBeeps = 2U;
constexpr uint8_t kLoRaFailureBeeps = 3U;
constexpr uint8_t kStorageFailureBeeps = 4U;
} // namespace Panic

namespace LoRa
{
constexpr long kFlightFrequencyHz = 920900000L;
constexpr uint32_t kFlightSpiFrequencyHz = 250000UL;
constexpr int kFlightTxPowerDbm = 17;
constexpr uint8_t kFlightInitAttempts = 10U;
constexpr uint32_t kFlightInitRetryDelayMs = 250UL;
constexpr uint32_t kFlightInitPreBeginSettleMs = 150UL;
constexpr uint32_t kFlightBusyWaitTimeoutMs = 25UL;
constexpr uint32_t kFlightTxInterPacketGapMs = 150UL;
constexpr uint8_t kFlightSpiMode = 0x00U; // Teensy SPI_MODE0.
constexpr bool kFlightProbeSpiMode = false;
constexpr float kFlightTcxoVoltage = 0.0f; // 0 V assumes an XTAL, not a DIO3-controlled TCXO.
constexpr bool kFlightUseRegulatorLdo = false;
constexpr bool kFlightDownlinkOnly = true; // RX/uplink validation is a later stability phase.

constexpr int kSpreadingFactor = 7;
constexpr long kSignalBandwidthHz = 125000L;
constexpr int kCodingRateDenominator = 5;
constexpr long kPreambleLength = 8L;
constexpr int kSyncWord = 0x12;

constexpr uint8_t kRegVersion = 0x42U;
constexpr uint8_t kExpectedVersion = 0x12U;
} // namespace LoRa

namespace Flight
{
// No flight pyro GPIO, arming input, or continuity input is assigned yet.
constexpr bool kPyroOutputImplemented = false;
constexpr uint32_t kAccelFallbackMaxSampleAgeMs = 50UL;

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
constexpr uint32_t kBaroFaultAttitudeFallbackMinFlightTimeMs = 8000UL;
constexpr float kBaroFaultAttitudeFallbackTiltDeg = 70.0f;
constexpr uint8_t kBaroFaultAttitudeFallbackConfirmSamples = 5U;
constexpr uint32_t kBaroFaultAttitudeFallbackMaxSampleAgeMs = 150UL;

constexpr uint32_t kApogeeTimeoutMs = 12000UL;
constexpr uint32_t kMainTimeoutMs = 15000UL;

constexpr uint32_t kPyroFireDurationMs = 50UL;
constexpr uint32_t kDrogueBackupDelayMs = 2000UL;
constexpr float kMainDeployAltitudeM = 200.0f;
constexpr uint8_t kLandingStableWindowSamples = 20U;
constexpr float kLandingStableAltitudeRangeM = 0.5f;
constexpr uint32_t kLandingMaxBarometerSampleGapMs = 150UL;
} // namespace Flight

namespace BenchIntegration
{
constexpr uint32_t kFsmAutoArmDelayMs = 1000UL;
constexpr uint32_t kFsmAutoLaunchDelayMs = 1000UL;
constexpr uint32_t kFsmAutoBurnoutDelayMs = 500UL;
constexpr uint32_t kFsmAutoGroundDelayMs = 1000UL;
} // namespace BenchIntegration

namespace LSM6DSO32
{
constexpr uint8_t kWhoAmIRegister = 0x0FU;
constexpr uint8_t kExpectedWhoAmI = 0x6CU;
constexpr uint8_t kNoSpiMode = 0xFFU;
constexpr uint32_t kProbeSpiHz = 1000000UL;
} // namespace LSM6DSO32

namespace H3LIS331DL
{
constexpr uint8_t kExpectedWhoAmI = 0x32U;
} // namespace H3LIS331DL

namespace MPL3115A2
{
constexpr uint16_t kConversionTimeoutMs = 50U;
constexpr uint8_t kFastBarometerCtrlReg1 = 0x00U; // BAR mode, OS1.
constexpr float kHpaToPa = 100.0f;
constexpr float kMinDatasheetPressurePa = 20000.0f;
constexpr float kMaxDatasheetPressurePa = 110000.0f;
} // namespace MPL3115A2

namespace Mock
{
struct FlightProfile
{
    float launchTimeS;
    float burnoutTimeS;
    float apogeeTimeS;
    float apogeeAltitudeM;
    float descentRateMps;
};

constexpr uint8_t kDefaultScenarioId = 0U;
constexpr uint32_t kBarometerPeriodMs = Sensors::kBarometerTaskPeriodMs;
constexpr uint32_t kBarometerDropoutStartMs = 8200UL;
constexpr uint32_t kBarometerDropoutEndMs = 8800UL;
constexpr uint32_t kPadFalseAccelStartMs = 300UL;
constexpr uint32_t kPadFalseAccelEndMs = 330UL;

constexpr float kNoiseAmplitudeM = 0.8f;
constexpr float kGustNoiseAmplitudeM = 0.6f;
constexpr float kGustPulseAmplitudeM = -6.0f;
constexpr float kGustPulseLeadS = 1.2f;
constexpr float kGustPulseSigmaS = 0.16f;

constexpr float kAccelXG = 0.02f;
constexpr float kAccelYG = 0.01f;
constexpr float kPrelaunchAccelG = 0.15f;
constexpr float kMotorBurnAccelG = 3.8f;
constexpr float kCoastAccelG = 0.55f;
constexpr float kPadFalseAccelG = 2.5f;

constexpr double kBaseLatitudeDeg = 37.1234567;
constexpr double kBaseLongitudeDeg = 127.1234567;
constexpr double kGpsBaseAltitudeM = 50.0;
constexpr double kGpsDriftDegPerS = 0.000001;
constexpr double kAscentSpeedMps = 35.0;
constexpr double kCourseDeg = 85.2;
constexpr double kHdop = 1.2;
constexpr uint8_t kSatellites = 9U;
constexpr uint32_t kLocationAgeMs = 100UL;
constexpr uint16_t kBatteryBaseMv = 12000U;
constexpr uint16_t kBatterySawtoothMv = 500U;
constexpr uint32_t kBatterySawtoothPeriodMs = 10000UL;

constexpr FlightProfile kNominalProfile = {1.0f, 2.55f, 10.5f, 400.0f, 25.0f};
constexpr FlightProfile kLowApogeeProfile = {1.0f, 2.45f, 9.8f, 250.0f, 22.0f};
} // namespace Mock
} // namespace NuraConstants
