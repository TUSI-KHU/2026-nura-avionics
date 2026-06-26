#pragma once

#include <stdint.h>

#include "core/logger/logger.h"
#include "core/tasks.h"
#include "hal/battery_voltage_hal.h"
#include "state/telemetry_state.h"

class PowerSenseTask : public Task
{
public:
    PowerSenseTask(BatteryVoltageHAL &batteryVoltage, TelemetryState &telemetryState, Logger &logger);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    void publishInvalid(uint32_t nowMs);

    BatteryVoltageHAL &batteryVoltage_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    bool initialized_ = false;
    bool lastValid_ = false;
};
