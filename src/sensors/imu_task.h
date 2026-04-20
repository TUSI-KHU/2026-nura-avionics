#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "state/imu_state.h"
#include "hal/mpu6050_hal.h"

class IMUTask : public RecoverableTask
{
public:
    // MPU6050를 주기적으로 읽고 recoverable 정책을 적용하는 센서 태스크다
    IMUTask(MPU6050HAL &imu, ImuState &imuState, Logger &logger, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

    bool recover(uint32_t nowMs) override;

private:
    bool initializeDevice(uint32_t logTs);

    MPU6050HAL &imu_;
    ImuState &imuState_;
    Logger &logger_;
    const IAppConfig &config_;
};
