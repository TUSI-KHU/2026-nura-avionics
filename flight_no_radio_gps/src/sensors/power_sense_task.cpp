#include "power_sense_task.h"

#include "board_pinmap.h"
#include "nura_constants.h"

PowerSenseTask::PowerSenseTask(BatteryVoltageHAL &batteryVoltage,
                               TelemetryState &telemetryState,
                               Logger &logger)
    : batteryVoltage_(batteryVoltage),
      telemetryState_(telemetryState),
      logger_(logger)
{
}

const char *PowerSenseTask::name() const
{
    return "power";
}

bool PowerSenseTask::init()
{
    publishInvalid(0UL);
    initialized_ = batteryVoltage_.begin(BoardPinMap::PowerSense::voltagePin,
                                         NuraConstants::Sensors::kPowerSenseAdcReferenceMv,
                                         NuraConstants::Sensors::kPowerSenseAdcResolutionBits,
                                         NuraConstants::Sensors::kPowerSenseDividerRatioNumerator,
                                         NuraConstants::Sensors::kPowerSenseDividerRatioDenominator,
                                         NuraConstants::Sensors::kPowerSenseMinValidBatteryMv,
                                         NuraConstants::Sensors::kPowerSenseMaxValidBatteryMv);

    if (initialized_)
    {
        LOGI(logger_, 0U, "power", "battery voltage sense initialized");
    }
    else
    {
        LOGW(logger_, 0U, "power", "battery voltage sense init failed");
    }

    return true;
}

bool PowerSenseTask::tick(uint32_t nowMs)
{
    if (!initialized_)
    {
        return true;
    }

    BatteryVoltageReading reading;
    if (!batteryVoltage_.read(reading, nowMs) || !reading.valid)
    {
        if (lastValid_)
        {
            LOGW(logger_, nowMs, "power", "battery voltage invalid");
        }
        publishInvalid(nowMs);
        return true;
    }

    PowerTelemetryData &power = telemetryState_.power;
    power.valid = true;
    power.batteryMv = reading.batteryMv;
    power.lastUpdatedMs = reading.sampleMs;
    lastValid_ = true;
    return true;
}

uint32_t PowerSenseTask::periodMs() const
{
    return NuraConstants::Sensors::kPowerSenseTaskPeriodMs;
}

void PowerSenseTask::publishInvalid(uint32_t nowMs)
{
    telemetryState_.power.valid = false;
    telemetryState_.power.batteryMv = 0U;
    telemetryState_.power.lastUpdatedMs = nowMs;
    lastValid_ = false;
}
