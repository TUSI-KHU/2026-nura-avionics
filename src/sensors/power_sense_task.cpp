#include "power_sense_task.h"

#include "board_pinmap.h"
#include "nura_constants.h"

PowerSenseTask::PowerSenseTask(IBatteryVoltage &batteryVoltage,
                               PowerState &powerState,
                               Logger &logger,
                               uint8_t sensePin)
    : batteryVoltage_(batteryVoltage),
      powerState_(powerState),
      logger_(logger),
      sensePin_(sensePin)
{
}

const char *PowerSenseTask::name() const
{
    return "power";
}

bool PowerSenseTask::init()
{
    publishInvalid(0UL);
    initialized_ = false;
    for (uint8_t attempt = 0U; attempt < NuraConstants::Sensors::kSensorInitRetryAttempts; ++attempt)
    {
        initialized_ = batteryVoltage_.begin(sensePin_,
                                              NuraConstants::Sensors::kPowerSenseAdcReferenceMv,
                                              NuraConstants::Sensors::kPowerSenseAdcResolutionBits,
                                              NuraConstants::Sensors::kPowerSenseDividerRatioNumerator,
                                              NuraConstants::Sensors::kPowerSenseDividerRatioDenominator,
                                              NuraConstants::Sensors::kPowerSenseMinValidBatteryMv,
                                              NuraConstants::Sensors::kPowerSenseMaxValidBatteryMv);
        if (initialized_)
        {
            break;
        }
        if ((attempt + 1U) < NuraConstants::Sensors::kSensorInitRetryAttempts)
        {
            delay(NuraConstants::Sensors::kSensorInitRetryDelayMs);
        }
    }

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

    PowerState &power = powerState_;
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
    powerState_.valid = false;
    powerState_.batteryMv = 0U;
    powerState_.lastUpdatedMs = nowMs;
    lastValid_ = false;
}
