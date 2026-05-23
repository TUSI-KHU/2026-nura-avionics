#pragma once
#include <stdint.h>

enum class State : uint8_t
{
    INIT,
    SAFE,
    ARMED,
    LAUNCH,
    COAST,
    APOGEE,
    DROGUE,
    DEPLOY,
    GROUND,
    FAULT
};

inline const char *stateName(State state)
{
    // 로그 출력을 위한 사람이 읽기 쉬운 상태 이름 변환 함수다.
    switch (state)
    {
    case State::INIT:
        return "INIT";
    case State::SAFE:
        return "SAFE";
    case State::ARMED:
        return "ARMED";
    case State::LAUNCH:
        return "LAUNCH";
    case State::COAST:
        return "COAST";
    case State::APOGEE:
        return "APOGEE";
    case State::DROGUE:
        return "DROGUE";
    case State::DEPLOY:
        return "DEPLOY";
    case State::GROUND:
        return "GROUND";
    case State::FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}
