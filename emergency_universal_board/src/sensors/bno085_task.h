#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "hal/bno085_hal.h"
#include "state/high_g_imu_state.h"
#include "state/imu_state.h"
#include "state/telemetry_state.h"

class BNO085Task : public RecoverableTask
{
public:
    BNO085Task(BNO085HAL &imu,
               ImuState &imuState,
               HighGImuState &highGImuState,
               TelemetryState &telemetryState,
               Logger &logger,
               const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;
    bool recover(uint32_t nowMs) override;

private:
    bool initializeDevice(uint32_t logTs);
    void publishSample(const Bno085Reading &sample);
    void logSample(uint32_t nowMs);

    BNO085HAL &imu_;
    ImuState &imuState_;
    HighGImuState &highGImuState_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
    uint32_t lastSampleLogMs_ = 0U;
};
