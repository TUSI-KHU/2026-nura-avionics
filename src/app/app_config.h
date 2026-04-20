#pragma once

#include <stdint.h>

#include <Arduino.h>

class IAppConfig
{
public:
    virtual ~IAppConfig() = default;

    virtual unsigned long serialBaudRate() const = 0;
    virtual uint8_t statusIndicatorPin() const = 0;
    virtual uint16_t faultBlinkIntervalMs() const = 0;

    virtual uint8_t imuI2cAddress() const = 0;
    virtual uint8_t imuReadFailureThreshold() const = 0;
    virtual uint8_t imuMaxRecoveryAttempts() const = 0;
    virtual uint32_t imuRecoveryIntervalMs() const = 0;
    virtual uint32_t imuTaskPeriodMs() const = 0;

    virtual uint32_t watchdogTaskPeriodMs() const = 0;
    virtual uint32_t flightStateTaskPeriodMs() const = 0;
    virtual uint32_t loggerTaskPeriodMs() const = 0;

    virtual uint8_t loggerDrainBudget() const = 0;
    virtual uint8_t loggerOutputFailThreshold() const = 0;
};

class DefaultAppConfig : public IAppConfig
{
public:
    unsigned long serialBaudRate() const override;
    uint8_t statusIndicatorPin() const override;
    uint16_t faultBlinkIntervalMs() const override;

    uint8_t imuI2cAddress() const override;
    uint8_t imuReadFailureThreshold() const override;
    uint8_t imuMaxRecoveryAttempts() const override;
    uint32_t imuRecoveryIntervalMs() const override;
    uint32_t imuTaskPeriodMs() const override;

    uint32_t watchdogTaskPeriodMs() const override;
    uint32_t flightStateTaskPeriodMs() const override;
    uint32_t loggerTaskPeriodMs() const override;

    uint8_t loggerDrainBudget() const override;
    uint8_t loggerOutputFailThreshold() const override;
};
