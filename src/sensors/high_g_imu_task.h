#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "hal/h3lis331dl_hal.h"
#include "state/high_g_imu_state.h"

class HighGImuTask : public RecoverableTask
{
public:
    HighGImuTask(H3LIS331DLHAL &imu,
                 HighGImuState &imuState,
                 Logger &logger,
                 const IAppConfig &config,
                 uint8_t csPin = 10U,
                 H3LIS331DLRange range = H3LIS331DLRange::RANGE_100G);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

    bool recover(uint32_t nowMs) override;

private:
    bool initializeDevice(uint32_t logTs);
    void resetState();
    void updateState(const H3LIS331DLReading &sample);
    void logSample(uint32_t nowMs);

    H3LIS331DLHAL &imu_;
    HighGImuState &imuState_;
    Logger &logger_;
    const IAppConfig &config_;
    uint8_t csPin_;
    H3LIS331DLRange range_;
    uint32_t lastSampleLogMs_ = 0U;
};
