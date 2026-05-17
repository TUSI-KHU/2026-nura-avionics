#pragma once

#include <stdint.h>

struct MagData
{
    float magXuT = 0.0f;
    float magYuT = 0.0f;
    float magZuT = 0.0f;

    float magXGauss = 0.0f;
    float magYGauss = 0.0f;
    float magZGauss = 0.0f;

    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    uint32_t lastUpdatedMs = 0;
};

struct MagState
{
    MagData data;
};
