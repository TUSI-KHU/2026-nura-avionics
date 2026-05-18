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

    virtual uint8_t imuCsPin() const = 0;
    virtual uint8_t imuReadFailureThreshold() const = 0;
    virtual uint8_t imuMaxRecoveryAttempts() const = 0;
    virtual uint32_t imuRecoveryIntervalMs() const = 0;
    virtual uint32_t imuTaskPeriodMs() const = 0;

    virtual uint8_t magI2cAddress() const = 0;
    virtual uint8_t magReadFailureThreshold() const = 0;
    virtual uint8_t magMaxRecoveryAttempts() const = 0;
    virtual uint32_t magRecoveryIntervalMs() const = 0;
    virtual uint32_t magTaskPeriodMs() const = 0;

    virtual uint32_t barometerTaskPeriodMs() const = 0;
    virtual uint32_t barometerRecoveryIntervalMs() const = 0;
    virtual uint32_t gnssTaskPeriodMs() const = 0;
    virtual uint16_t gnssPollByteBudget() const = 0;
    virtual uint32_t gnssMaxFixAgeMs() const = 0;

    virtual uint32_t watchdogTaskPeriodMs() const = 0;
    virtual uint32_t flightStateTaskPeriodMs() const = 0;
    virtual uint32_t loggerTaskPeriodMs() const = 0;
    virtual uint32_t telemetryTaskPeriodMs() const = 0;
    virtual uint32_t telemetryFastPeriodMs() const = 0;
    virtual uint32_t telemetryGpsPeriodMs() const = 0;
    virtual uint32_t telemetrySensorFreshMs() const = 0;

    virtual uint8_t loggerDrainBudget() const = 0;
    virtual uint8_t loggerOutputFailThreshold() const = 0;

    virtual long loraFrequencyHz() const = 0;
    virtual uint32_t loraSpiFrequencyHz() const = 0;
    virtual int loraTxPowerDbm() const = 0;
    virtual int loraSpreadingFactor() const = 0;
    virtual long loraSignalBandwidthHz() const = 0;
    virtual int loraCodingRateDenominator() const = 0;
    virtual long loraPreambleLength() const = 0;
    virtual int loraSyncWord() const = 0;
    virtual uint8_t loraInitAttempts() const = 0;
    virtual uint8_t loraSpiMode() const = 0;
    virtual bool loraProbeSpiMode() const = 0;
};

class DefaultAppConfig : public IAppConfig
{
public:
    unsigned long serialBaudRate() const override;
    uint8_t statusIndicatorPin() const override;
    uint16_t faultBlinkIntervalMs() const override;

    uint8_t imuCsPin() const override;
    uint8_t imuReadFailureThreshold() const override;
    uint8_t imuMaxRecoveryAttempts() const override;
    uint32_t imuRecoveryIntervalMs() const override;
    uint32_t imuTaskPeriodMs() const override;

    uint8_t magI2cAddress() const override;
    uint8_t magReadFailureThreshold() const override;
    uint8_t magMaxRecoveryAttempts() const override;
    uint32_t magRecoveryIntervalMs() const override;
    uint32_t magTaskPeriodMs() const override;

    uint32_t barometerTaskPeriodMs() const override;
    uint32_t barometerRecoveryIntervalMs() const override;
    uint32_t gnssTaskPeriodMs() const override;
    uint16_t gnssPollByteBudget() const override;
    uint32_t gnssMaxFixAgeMs() const override;

    uint32_t watchdogTaskPeriodMs() const override;
    uint32_t flightStateTaskPeriodMs() const override;
    uint32_t loggerTaskPeriodMs() const override;
    uint32_t telemetryTaskPeriodMs() const override;
    uint32_t telemetryFastPeriodMs() const override;
    uint32_t telemetryGpsPeriodMs() const override;
    uint32_t telemetrySensorFreshMs() const override;

    uint8_t loggerDrainBudget() const override;
    uint8_t loggerOutputFailThreshold() const override;

    long loraFrequencyHz() const override;
    uint32_t loraSpiFrequencyHz() const override;
    int loraTxPowerDbm() const override;
    int loraSpreadingFactor() const override;
    long loraSignalBandwidthHz() const override;
    int loraCodingRateDenominator() const override;
    long loraPreambleLength() const override;
    int loraSyncWord() const override;
    uint8_t loraInitAttempts() const override;
    uint8_t loraSpiMode() const override;
    bool loraProbeSpiMode() const override;
};
