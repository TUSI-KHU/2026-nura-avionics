#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/tasks.h"
#include "hal/mpl3115a2_hal.h"
#include "state/telemetry_state.h"

class BarometerTask : public Task
{
public:
    BarometerTask(MPL3115A2HAL &barometer, TelemetryState &telemetryState, Logger &logger, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    bool initialize(uint32_t nowMs);
    void clearReading(uint32_t nowMs);

    MPL3115A2HAL &barometer_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
    bool initialized_ = false;
    uint32_t lastInitAttemptMs_ = 0;
};
