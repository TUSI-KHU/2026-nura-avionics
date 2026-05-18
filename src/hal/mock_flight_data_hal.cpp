#include "mock_flight_data_hal.h"

bool MockFlightDataHAL::begin()
{
    return true;
}

bool MockFlightDataHAL::read(MockFlightDataReading &out, uint32_t nowMs)
{
    const uint32_t phase = (nowMs / 100UL) % 120UL;
    const float phaseF = static_cast<float>(phase);

    out.accelXMps2 = (static_cast<float>(phase % 20UL) - 10.0f) * 0.0980665f;
    out.accelYMps2 = (5.0f - static_cast<float>(phase % 10UL)) * 0.0980665f;
    out.accelZMps2 = 9.80665f + (static_cast<float>(phase % 18UL) * 0.0980665f);
    out.gyroXDps = (static_cast<float>(phase % 30UL) - 15.0f) * 0.1f;
    out.gyroYDps = (static_cast<float>(phase % 26UL) - 13.0f) * 0.1f;
    out.gyroZDps = (static_cast<float>(phase % 22UL) - 11.0f) * 0.1f;
    out.pressurePa = 101325.0f - (phaseF * 8.0f);
    out.latitudeDeg = 37.1234567 + (static_cast<double>((nowMs / 1000UL) % 100UL) * 0.0000001);
    out.longitudeDeg = 127.1234567 + (static_cast<double>((nowMs / 1000UL) % 100UL) * 0.0000001);
    out.altitudeM = 50.0 + static_cast<double>(phase % 30UL);
    out.speedMps = 12.0;
    out.courseDeg = 85.2;
    out.hdop = 1.2;
    out.satellites = 9U;
    out.batteryMv = static_cast<uint16_t>(12000U - static_cast<uint16_t>((nowMs / 10000UL) % 500UL));
    out.sampleMs = nowMs;
    return true;
}
