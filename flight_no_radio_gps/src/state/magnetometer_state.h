#pragma once

#include <stdint.h>

struct MagnetometerState
{
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    float magXuT = 0.0f;
    float magYuT = 0.0f;
    float magZuT = 0.0f;

    bool connected = false;
    bool hasNewData = false;

    uint32_t lastUpdatedMs = 0;
};
