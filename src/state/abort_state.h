#pragma once

#include <stdint.h>

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

struct AbortState
{
    AbortStatus status;
};
