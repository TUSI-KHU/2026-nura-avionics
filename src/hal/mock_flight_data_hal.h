#pragma once

#include <stdint.h>

struct MockFlightDataReading
{
    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;
    float gyroXDps = 0.0f;
    float gyroYDps = 0.0f;
    float gyroZDps = 0.0f;
    float pressurePa = 101325.0f;
    double latitudeDeg = 37.1234567;
    double longitudeDeg = 127.1234567;
    double altitudeM = 50.0;
    double speedMps = 12.0;
    double courseDeg = 85.2;
    double hdop = 1.2;
    uint8_t satellites = 9U;
    uint16_t batteryMv = 12000U;
    uint32_t sampleMs = 0;
};

class MockFlightDataHAL
{
public:
    bool begin();
    bool read(MockFlightDataReading &out, uint32_t nowMs);
};
