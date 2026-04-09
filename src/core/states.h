#pragma once
#include <stdint.h>

enum class State : uint8_t 
{
    BOOT,
    IDLE,
    ARMED,
    LAUNCH,
    DESCENT,
    GROUND,
    SAFE
};

inline const char* stateName(State state) {
    switch (state) {
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
