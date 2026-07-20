#pragma once

#include <stdint.h>

struct PowerState
{
    bool valid = false;
    uint16_t batteryMv = 0U;
    uint32_t lastUpdatedMs = 0U;
};
