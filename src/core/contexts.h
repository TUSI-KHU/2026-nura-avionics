#pragma once
#include <stdint.h>
#include "states.h"

struct ImuData {
    float accelZMps2 = 0.0f;
    float gyroZDps = 0.0f;
    uint32_t lastUpdatedMs = 0;
};

struct HealthFlags {
    bool imuOk = false;
    bool baroOk = false;
};

struct SystemContext {
    State state = State::BOOT;
    uint32_t stateEnteredMs = 0;

    ImuData imu;

    HealthFlags health;
};
