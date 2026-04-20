#pragma once
#include <stdint.h>

#include "core/recoverable_task/recoverable_task.h"
#include "logger/logger.h"
#include "states.h"

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

enum class AbortReason : uint8_t
{
    NONE,
    CRITICAL_SENSOR_FAILURE
};

struct AbortStatus
{
    // watchdog가 올린 중단 상태를 FSM이 참조한다.
    bool active = false;
    AbortReason reason = AbortReason::NONE;
    uint32_t raisedMs = 0;
};

struct SystemContext
{
    // 태스크 사이를 오가는 컨텍스트
    State state = State::BOOT;
    uint32_t stateEnteredMs = 0;

    ImuData imu;

    AbortStatus abort;
    Logger logger;
};
