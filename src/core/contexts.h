#pragma once
#include <stdint.h>
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
};

struct SystemContext
{
    State state = State::BOOT;
    uint32_t stateEnteredMs = 0;

    ImuData imu;

    HealthFlags health;
    Logger logger;
};
