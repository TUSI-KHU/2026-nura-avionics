#include "app/app_config.h"

#include "board_pinmap.h"
#include "nura_constants.h"

// EEPROM이나 SPI Flash 등을 사용하지 않고 컴파일 타임 상수를 리턴하는
// 단계이다. 동작 파라미터는 nura_constants.h, 물리 핀맵은 board_pinmap.h를 따른다.
// TODO: Write an interface for EEPROM config store.

unsigned long DefaultAppConfig::serialBaudRate() const
{
    return NuraConstants::App::kSerialBaudRate;
}

uint8_t DefaultAppConfig::statusIndicatorPin() const
{
    return BoardPinMap::StatusIndicator::pin;
}

uint16_t DefaultAppConfig::faultBlinkIntervalMs() const
{
    return NuraConstants::App::kFaultBlinkIntervalMs;
}

uint8_t DefaultAppConfig::imuCsPin() const
{
    return BoardPinMap::LSM6DSO32::csPin;
}

uint8_t DefaultAppConfig::imuReadFailureThreshold() const
{
    return NuraConstants::Sensors::kImuReadFailureThreshold;
}

uint8_t DefaultAppConfig::imuMaxRecoveryAttempts() const
{
    return NuraConstants::Sensors::kImuMaxRecoveryAttempts;
}

uint32_t DefaultAppConfig::imuRecoveryIntervalMs() const
{
    return NuraConstants::Sensors::kImuRecoveryIntervalMs;
}

uint32_t DefaultAppConfig::imuTaskPeriodMs() const
{
    return NuraConstants::Sensors::kImuTaskPeriodMs;
}

uint32_t DefaultAppConfig::magnetometerTaskPeriodMs() const
{
    return NuraConstants::Sensors::kMagnetometerTaskPeriodMs;
}

uint32_t DefaultAppConfig::barometerTaskPeriodMs() const
{
    return NuraConstants::Sensors::kBarometerTaskPeriodMs;
}

uint32_t DefaultAppConfig::barometerRecoveryIntervalMs() const
{
    return NuraConstants::Sensors::kBarometerRecoveryIntervalMs;
}

uint32_t DefaultAppConfig::gnssTaskPeriodMs() const
{
    return NuraConstants::Sensors::kGnssTaskPeriodMs;
}

uint16_t DefaultAppConfig::gnssPollByteBudget() const
{
    return NuraConstants::Sensors::kGnssPollByteBudget;
}

uint32_t DefaultAppConfig::gnssMaxFixAgeMs() const
{
    return NuraConstants::Sensors::kGnssMaxFixAgeMs;
}

uint32_t DefaultAppConfig::watchdogTaskPeriodMs() const
{
    return NuraConstants::Tasks::kWatchdogTaskPeriodMs;
}

uint32_t DefaultAppConfig::flightStateTaskPeriodMs() const
{
    return NuraConstants::Tasks::kFlightStateTaskPeriodMs;
}

uint32_t DefaultAppConfig::loggerTaskPeriodMs() const
{
    return NuraConstants::Tasks::kLoggerTaskPeriodMs;
}

uint32_t DefaultAppConfig::telemetryTaskPeriodMs() const
{
    return NuraConstants::Tasks::kTelemetryTaskPeriodMs;
}

uint32_t DefaultAppConfig::telemetryFastPeriodMs() const
{
    return NuraConstants::Telemetry::kFastPeriodMs;
}

uint32_t DefaultAppConfig::telemetryGpsPeriodMs() const
{
    return NuraConstants::Telemetry::kGpsPeriodMs;
}

uint32_t DefaultAppConfig::telemetrySensorFreshMs() const
{
    return NuraConstants::Telemetry::kSensorFreshMs;
}

uint8_t DefaultAppConfig::loggerDrainBudget() const
{
    return NuraConstants::Logger::kDrainBudget;
}

uint8_t DefaultAppConfig::loggerOutputFailThreshold() const
{
    return NuraConstants::Logger::kOutputFailThreshold;
}

long DefaultAppConfig::loraFrequencyHz() const
{
#if defined(NURA_DEV_SX1278)
    return NuraConstants::LoRa::kDevFrequencyHz;
#else
    return NuraConstants::LoRa::kFlightFrequencyHz;
#endif
}

uint32_t DefaultAppConfig::loraSpiFrequencyHz() const
{
#if defined(NURA_DEV_SX1278)
    return NuraConstants::LoRa::kDevSpiFrequencyHz;
#else
    return NuraConstants::LoRa::kFlightSpiFrequencyHz;
#endif
}

int DefaultAppConfig::loraTxPowerDbm() const
{
#if defined(NURA_DEV_SX1278)
    return NuraConstants::LoRa::kDevTxPowerDbm;
#else
    return NuraConstants::LoRa::kFlightTxPowerDbm;
#endif
}

int DefaultAppConfig::loraSpreadingFactor() const
{
    return NuraConstants::LoRa::kSpreadingFactor;
}

long DefaultAppConfig::loraSignalBandwidthHz() const
{
    return NuraConstants::LoRa::kSignalBandwidthHz;
}

int DefaultAppConfig::loraCodingRateDenominator() const
{
    return NuraConstants::LoRa::kCodingRateDenominator;
}

long DefaultAppConfig::loraPreambleLength() const
{
    return NuraConstants::LoRa::kPreambleLength;
}

int DefaultAppConfig::loraSyncWord() const
{
    return NuraConstants::LoRa::kSyncWord;
}

uint8_t DefaultAppConfig::loraInitAttempts() const
{
#if defined(NURA_DEV_SX1278)
    return NuraConstants::LoRa::kDevInitAttempts;
#else
    return NuraConstants::LoRa::kFlightInitAttempts;
#endif
}

uint8_t DefaultAppConfig::loraSpiMode() const
{
#if defined(NURA_DEV_SX1278)
    return NuraConstants::LoRa::kDevSpiMode;
#else
    return NuraConstants::LoRa::kFlightSpiMode;
#endif
}

bool DefaultAppConfig::loraProbeSpiMode() const
{
#if defined(NURA_DEV_SX1278)
    return NuraConstants::LoRa::kDevProbeSpiMode;
#else
    return NuraConstants::LoRa::kFlightProbeSpiMode;
#endif
}
