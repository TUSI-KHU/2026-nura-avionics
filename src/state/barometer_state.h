#pragma once

#include <stdint.h>

enum BarometerFaultFlags : uint16_t
{
    BARO_FAULT_NONE = 0U,
    BARO_FAULT_READ_FAIL = 1U << 0,
    BARO_FAULT_STALE = 1U << 1,
    BARO_FAULT_BAD_VALUE = 1U << 2,
    BARO_FAULT_STUCK = 1U << 3,
};

struct BarometerState
{
    bool valid = false;
    bool referenceValid = false;
    bool fault = false;
    uint16_t faultFlags = BARO_FAULT_NONE;
    uint8_t consecutiveReadFailCount = 0U;
    uint8_t consecutiveBadValueCount = 0U;
    uint8_t totalBadValueCount = 0U;
    float pressurePa = 0.0f;
    float referencePressurePa = 0.0f;
    float rawAltitudeM = 0.0f;
    float altitudeM = 0.0f;
    uint32_t lastUpdatedMs = 0U;
};
