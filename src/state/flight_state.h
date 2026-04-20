#pragma once

#include <stdint.h>

#include "core/states.h"

struct FlightState
{
    State state = State::BOOT;
    uint32_t stateEnteredMs = 0;
};
