#pragma once

#include <stdint.h>

struct HighGImuState
{
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    uint8_t whoAmI = 0;
    bool connected = false;
    bool hasNewData = false;

    uint32_t lastUpdatedMs = 0;
};
