#pragma once

#include <stdint.h>

struct GpsData
{
    bool hasFix = false;
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeM = 0.0;
    double speedMps = 0.0;
    double courseDeg = 0.0;
    double hdop = 0.0;
    uint32_t satellites = 0;
    uint32_t locationAgeMs = 0;
    uint32_t charsProcessed = 0;
    uint32_t passedChecksum = 0;
    uint32_t failedChecksum = 0;
    uint32_t lastUpdatedMs = 0;
};

struct GpsState
{
    GpsData data;
};
