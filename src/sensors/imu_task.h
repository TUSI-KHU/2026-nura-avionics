#pragma once

#include <stdint.h>

#include "core/contexts.h"
#include "core/recoverable_device/recoverable_device.h"
#include "core/tasks.h"
#include "hal/mpu6050_hal.h"

class IMUTask : public Task, public RecoverableDevice
{
public:
    explicit IMUTask(MPU6050HAL &imu);

    const char *name() const override;
    bool init(SystemContext &ctx) override;
    bool tick(SystemContext &ctx, uint32_t nowMs) override;
    uint32_t periodMs() const override;

    const char *deviceName() const override;
    DeviceHealthInfo &health(SystemContext &ctx) const override;
    bool recover(SystemContext &ctx, uint32_t nowMs) override;

private:
    MPU6050HAL &imu_;
};
