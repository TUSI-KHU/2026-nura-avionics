#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "state/mag_state.h"
#include "hal/lis3mdl_hal.h"

class MagTask : public RecoverableTask
{
public:
    MagTask(LIS3MDLHAL &mag, MagState &magState, Logger &logger, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

    bool recover(uint32_t nowMs) override;

private:
    bool initializeDevice(uint32_t logTs);

    LIS3MDLHAL &mag_;
    MagState &magState_;
    Logger &logger_;
    const IAppConfig &config_;
};
