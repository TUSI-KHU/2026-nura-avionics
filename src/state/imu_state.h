#pragma once

#include <stdint.h>

struct ImuData
{
    // 마지막으로 정상 수집된 IMU 샘플을 시스템 전역에 공유한다.
    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    float gyroXDps = 0.0f;
    float gyroYDps = 0.0f;
    float gyroZDps = 0.0f;

    uint32_t lastUpdatedMs = 0;
};

struct ImuState
{
    ImuData data;
};
