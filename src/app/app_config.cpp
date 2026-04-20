#include "app/app_config.h"

// EEPROM이나 SPI Flash 등을 사용하지 않고 콘픽을 하드코딩
// 해두는 단계이기 때문에 전역 Private Namespace에서 값을 리턴하는
// 식으로 클래스를 구성했다.
// TODO: Write an interface for EEPROM config store.

namespace
{
    constexpr unsigned long kSerialBaudRate = 115200UL;
    constexpr uint8_t kStatusIndicatorPin = LED_BUILTIN;
    constexpr uint16_t kFaultBlinkIntervalMs = 1000U;

    constexpr uint8_t kImuI2cAddress = 0x68U;
    constexpr uint8_t kImuReadFailureThreshold = 3U;
    constexpr uint8_t kImuMaxRecoveryAttempts = 5U;
    constexpr uint32_t kImuRecoveryIntervalMs = 1000U;
    constexpr uint32_t kImuTaskPeriodMs = 10U;

    constexpr uint32_t kWatchdogTaskPeriodMs = 50U;
    constexpr uint32_t kFlightStateTaskPeriodMs = 100U;
    constexpr uint32_t kLoggerTaskPeriodMs = 20U;

    constexpr uint8_t kLoggerDrainBudget = 4U;
    constexpr uint8_t kLoggerOutputFailThreshold = 3U;
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

uint8_t DefaultAppConfig::imuI2cAddress() const
{
    return kImuI2cAddress;
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

uint8_t DefaultAppConfig::loggerDrainBudget() const
{
    return kLoggerDrainBudget;
}

uint8_t DefaultAppConfig::loggerOutputFailThreshold() const
{
    return kLoggerOutputFailThreshold;
}
