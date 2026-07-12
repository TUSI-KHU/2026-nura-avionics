#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "hal/h3lis331dl_hal.h"
#include "state/high_g_imu_state.h"
#include "state/telemetry_state.h"

class HighGImuTask : public RecoverableTask
{
public:
    HighGImuTask(H3LIS331DLHAL &imu,
                 HighGImuState &imuState,
                 TelemetryState &telemetryState,
                 Logger &logger,
                 const IAppConfig &config,
                 uint8_t csPin,
                 H3LIS331DLRange range);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

    bool recover(uint32_t nowMs) override;

private:
    bool initializeDevice(uint32_t logTs);
    bool calibrateStationary(uint32_t logTs);
    void resetState();
    void updateState(const H3LIS331DLReading &sample);
    void applyCalibrationAndFilter(const H3LIS331DLReading &sample,
                                   float &xG,
                                   float &yG,
                                   float &zG);
    void logSample(uint32_t nowMs);

    H3LIS331DLHAL &imu_;
    HighGImuState &imuState_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
    uint8_t csPin_;
    H3LIS331DLRange range_;
    uint32_t lastSampleLogMs_ = 0U;
    float offsetXG_ = 0.0f;
    float offsetYG_ = 0.0f;
    float offsetZG_ = 0.0f;
    float filteredXG_ = 0.0f;
    float filteredYG_ = 0.0f;
    float filteredZG_ = 0.0f;
    bool calibrationValid_ = false;
    bool filterReady_ = false;
};
