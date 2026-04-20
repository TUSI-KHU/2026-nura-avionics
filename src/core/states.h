#pragma once
#include <stdint.h>

enum class State : uint8_t
{
    // 비행 상태 머신의 상태 정의
    BOOT,
    IDLE,
    ARMED,
    LAUNCH,
    DESCENT,
    GROUND,
    SAFE
};

inline const char *stateName(State state)
{
    // 로그 출력을 위한 사람이 읽기 쉬운 상태 이름 변환 함수다.
    switch (state)
    {
    case State::BOOT:
        return "BOOT";
    case State::IDLE:
        return "IDLE";
    case State::ARMED:
        return "ARMED";
    case State::LAUNCH:
        return "LAUNCH";
    case State::DESCENT:
        return "DESCENT";
    case State::GROUND:
        return "GROUND";
    case State::SAFE:
        return "SAFE";
    default:
        return "UNKNOWN";
    }
}
