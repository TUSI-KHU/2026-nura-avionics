#pragma once

#include <stdint.h>

#include "core/contexts.h"
#include "core/tasks.h"
#include "hal/mpu6050_hal.h"

class IMUTask : public Task
{
public:
    explicit IMUTask(MPU6050HAL &imu);

    const char *name() const override;
    bool init(SystemContext &ctx) override;
    bool tick(SystemContext &ctx, uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    bool isSampleValid(const Mpu6050Reading &sample) const;

    MPU6050HAL &imu_;
    uint32_t nextReconnectAttemptMs_ = 0U;

    static const uint32_t kReconnectIntervalMs = 500U;
};
