#pragma once

#include <stdint.h>

#include "core/contexts.h"
#include "core/recoverable_task/recoverable_task.h"
#include "hal/mpu6050_hal.h"

class IMUTask : public RecoverableTask
{
public:
    explicit IMUTask(MPU6050HAL &imu);

    const char *name() const override;
    bool init(SystemContext &ctx) override;
    bool tick(SystemContext &ctx, uint32_t nowMs) override;
    uint32_t periodMs() const override;

    bool recover(uint32_t nowMs) override;

private:
    MPU6050HAL &imu_;
};
