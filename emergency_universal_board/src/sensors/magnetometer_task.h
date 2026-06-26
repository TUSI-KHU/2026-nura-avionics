#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "hal/lis3mdl_hal.h"
#include "state/magnetometer_state.h"
#include "state/telemetry_state.h"

class MagnetometerTask : public RecoverableTask
{
public:
    MagnetometerTask(LIS3MDLHAL &magnetometer,
                     MagnetometerState &magnetometerState,
                     TelemetryState &telemetryState,
                     Logger &logger,
                     const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

    bool recover(uint32_t nowMs) override;

private:
    bool initialize(uint32_t nowMs);
    void resetState(uint32_t nowMs);
    void updateState(const Lis3mdlReading &sample);

    LIS3MDLHAL &magnetometer_;
    MagnetometerState &magnetometerState_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
};
