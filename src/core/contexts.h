#pragma once
#include <stdint.h>

#include "core/recoverable_device/recoverable_device.h"
#include "logger/logger.h"
#include "states.h"

struct ImuData
{
    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    float gyroXDps = 0.0f;
    float gyroYDps = 0.0f;
    float gyroZDps = 0.0f;

    uint32_t lastUpdatedMs = 0;
};

struct HealthFlags
{
    bool imuOk = false;
    bool logOk = true;
    bool schedulerOk = true;

    DeviceHealthInfo imuDevice;
};

enum class AbortReason : uint8_t
{
    NONE,
    IMU_FAILED
};

struct AbortStatus
{
    bool active = false;
    AbortReason reason = AbortReason::NONE;
    uint32_t raisedMs = 0;
};

struct SystemContext
{
    State state = State::BOOT;
    uint32_t stateEnteredMs = 0;

    ImuData imu;

    HealthFlags health;
    AbortStatus abort;
    Logger logger;
};
