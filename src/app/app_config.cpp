#include "app/app_config.h"

#include <SPI.h>

#include "board_pinmap.h"

// EEPROM이나 SPI Flash 등을 사용하지 않고 콘픽을 하드코딩
// 해두는 단계이기 때문에 전역 Private Namespace에서 값을 리턴하는
// 식으로 클래스를 구성했다.
// TODO: Write an interface for EEPROM config store.

namespace
{
    constexpr unsigned long kSerialBaudRate = 115200UL;
    constexpr uint8_t kStatusIndicatorPin = BoardPinMap::StatusIndicator::pin;
    constexpr uint16_t kFaultBlinkIntervalMs = 1000U;

    constexpr uint8_t kImuCsPin = BoardPinMap::LSM6DSO32::csPin;
    constexpr uint8_t kImuReadFailureThreshold = 3U;
    constexpr uint8_t kImuMaxRecoveryAttempts = 5U;
    constexpr uint32_t kImuRecoveryIntervalMs = 1000U;
    constexpr uint32_t kImuTaskPeriodMs = 10U;

    constexpr uint8_t kMagI2cAddress = BoardPinMap::LIS3MDL::i2cAddress;
    constexpr uint8_t kMagReadFailureThreshold = 3U;
    constexpr uint8_t kMagMaxRecoveryAttempts = 5U;
    constexpr uint32_t kMagRecoveryIntervalMs = 1000U;
    constexpr uint32_t kMagTaskPeriodMs = 20U;

    constexpr uint32_t kBarometerTaskPeriodMs = 50U;
    constexpr uint32_t kBarometerRecoveryIntervalMs = 1000U;
    constexpr uint32_t kGnssTaskPeriodMs = 50U;
    constexpr uint16_t kGnssPollByteBudget = 128U;
    constexpr uint32_t kGnssMaxFixAgeMs = 2000U;

    constexpr uint32_t kWatchdogTaskPeriodMs = 50U;
    constexpr uint32_t kFlightStateTaskPeriodMs = 100U;
    constexpr uint32_t kLoggerTaskPeriodMs = 20U;
    constexpr uint32_t kTelemetryTaskPeriodMs = 20U;
    constexpr uint32_t kTelemetryFastPeriodMs = 200U;
    constexpr uint32_t kTelemetryGpsPeriodMs = 1000U;
    constexpr uint32_t kTelemetrySensorFreshMs = 500U;

    constexpr uint8_t kLoggerDrainBudget = 4U;
    constexpr uint8_t kLoggerOutputFailThreshold = 3U;

#if defined(NURA_DEV_SX1278)
    constexpr long kLoraFrequencyHz = 433000000L;
    constexpr uint32_t kLoraSpiFrequencyHz = 125000UL;
    constexpr int kLoraTxPowerDbm = 10;
    constexpr uint8_t kLoraInitAttempts = 5U;
    constexpr uint8_t kLoraSpiMode = SPI_MODE1;
    constexpr bool kLoraProbeSpiMode = true;
#else
    constexpr long kLoraFrequencyHz = 920900000L;
    constexpr uint32_t kLoraSpiFrequencyHz = 8000000UL;
    constexpr int kLoraTxPowerDbm = 17;
    constexpr uint8_t kLoraInitAttempts = 1U;
    constexpr uint8_t kLoraSpiMode = SPI_MODE0;
    constexpr bool kLoraProbeSpiMode = false;
#endif
    constexpr int kLoraSpreadingFactor = 7;
    constexpr long kLoraSignalBandwidthHz = 125000L;
    constexpr int kLoraCodingRateDenominator = 5;
    constexpr long kLoraPreambleLength = 8L;
    constexpr int kLoraSyncWord = 0x12;
}

unsigned long DefaultAppConfig::serialBaudRate() const
{
    return kSerialBaudRate;
}

uint8_t DefaultAppConfig::statusIndicatorPin() const
{
    return kStatusIndicatorPin;
}

uint16_t DefaultAppConfig::faultBlinkIntervalMs() const
{
    return kFaultBlinkIntervalMs;
}

uint8_t DefaultAppConfig::imuCsPin() const
{
    return kImuCsPin;
}

uint8_t DefaultAppConfig::imuReadFailureThreshold() const
{
    return kImuReadFailureThreshold;
}

uint8_t DefaultAppConfig::imuMaxRecoveryAttempts() const
{
    return kImuMaxRecoveryAttempts;
}

uint32_t DefaultAppConfig::imuRecoveryIntervalMs() const
{
    return kImuRecoveryIntervalMs;
}

uint32_t DefaultAppConfig::imuTaskPeriodMs() const
{
    return kImuTaskPeriodMs;
}

uint8_t DefaultAppConfig::magI2cAddress() const
{
    return kMagI2cAddress;
}

uint8_t DefaultAppConfig::magReadFailureThreshold() const
{
    return kMagReadFailureThreshold;
}

uint8_t DefaultAppConfig::magMaxRecoveryAttempts() const
{
    return kMagMaxRecoveryAttempts;
}

uint32_t DefaultAppConfig::magRecoveryIntervalMs() const
{
    return kMagRecoveryIntervalMs;
}

uint32_t DefaultAppConfig::magTaskPeriodMs() const
{
    return kMagTaskPeriodMs;
}

uint32_t DefaultAppConfig::barometerTaskPeriodMs() const
{
    return kBarometerTaskPeriodMs;
}

uint32_t DefaultAppConfig::barometerRecoveryIntervalMs() const
{
    return kBarometerRecoveryIntervalMs;
}

uint32_t DefaultAppConfig::gnssTaskPeriodMs() const
{
    return kGnssTaskPeriodMs;
}

uint16_t DefaultAppConfig::gnssPollByteBudget() const
{
    return kGnssPollByteBudget;
}

uint32_t DefaultAppConfig::gnssMaxFixAgeMs() const
{
    return kGnssMaxFixAgeMs;
}

uint32_t DefaultAppConfig::watchdogTaskPeriodMs() const
{
    return kWatchdogTaskPeriodMs;
}

uint32_t DefaultAppConfig::flightStateTaskPeriodMs() const
{
    return kFlightStateTaskPeriodMs;
}

uint32_t DefaultAppConfig::loggerTaskPeriodMs() const
{
    return kLoggerTaskPeriodMs;
}

uint32_t DefaultAppConfig::telemetryTaskPeriodMs() const
{
    return kTelemetryTaskPeriodMs;
}

uint32_t DefaultAppConfig::telemetryFastPeriodMs() const
{
    return kTelemetryFastPeriodMs;
}

uint32_t DefaultAppConfig::telemetryGpsPeriodMs() const
{
    return kTelemetryGpsPeriodMs;
}

uint32_t DefaultAppConfig::telemetrySensorFreshMs() const
{
    return kTelemetrySensorFreshMs;
}

uint8_t DefaultAppConfig::loggerDrainBudget() const
{
    return kLoggerDrainBudget;
}

uint8_t DefaultAppConfig::loggerOutputFailThreshold() const
{
    return kLoggerOutputFailThreshold;
}

long DefaultAppConfig::loraFrequencyHz() const
{
    return kLoraFrequencyHz;
}

uint32_t DefaultAppConfig::loraSpiFrequencyHz() const
{
    return kLoraSpiFrequencyHz;
}

int DefaultAppConfig::loraTxPowerDbm() const
{
    return kLoraTxPowerDbm;
}

int DefaultAppConfig::loraSpreadingFactor() const
{
    return kLoraSpreadingFactor;
}

long DefaultAppConfig::loraSignalBandwidthHz() const
{
    return kLoraSignalBandwidthHz;
}

int DefaultAppConfig::loraCodingRateDenominator() const
{
    return kLoraCodingRateDenominator;
}

long DefaultAppConfig::loraPreambleLength() const
{
    return kLoraPreambleLength;
}

int DefaultAppConfig::loraSyncWord() const
{
    return kLoraSyncWord;
}

uint8_t DefaultAppConfig::loraInitAttempts() const
{
    return kLoraInitAttempts;
}

uint8_t DefaultAppConfig::loraSpiMode() const
{
    return kLoraSpiMode;
}

bool DefaultAppConfig::loraProbeSpiMode() const
{
    return kLoraProbeSpiMode;
}
