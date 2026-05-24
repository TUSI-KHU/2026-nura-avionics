#pragma once

#include <stdint.h>

#include "nura_constants.h"

enum class MockFlightScenarioId : uint8_t
{
    NOMINAL = 0U,
    LOW_APOGEE = 1U,
    BARO_NOISE = 2U,
    BARO_GUST = 3U,
    BARO_DROPOUT = 4U,
    PAD_FALSE_ACCEL = 5U
};

struct MockFlightDataReading
{
    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;
    float highAccelXG = 0.0f;
    float highAccelYG = 0.0f;
    float highAccelZG = 0.0f;
    float gyroXDps = 0.0f;
    float gyroYDps = 0.0f;
    float gyroZDps = 0.0f;
    bool attitudeValid = true;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;
    bool tiltValid = true;
    float tiltAngleDeg = 0.0f;
    float pressurePa = 101325.0f;
    float rawAltitudeM = 0.0f;
    float filteredAltitudeM = 0.0f;
    bool barometerValid = true;
    bool barometerUpdated = false;
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
    void setScenario(MockFlightScenarioId scenario);
    MockFlightScenarioId scenario() const;
    const char *scenarioName() const;

private:
    using ScenarioParams = NuraConstants::Mock::FlightProfile;

    static ScenarioParams paramsFor(MockFlightScenarioId scenario);
    static const char *nameFor(MockFlightScenarioId scenario);
    static float altitudeFor(const ScenarioParams &params, float timeS);
    static float pressureForAltitude(float altitudeM);
    static float deterministicNoiseM(uint32_t nowMs, float amplitudeM);
    float filteredAltitude(float rawAltitudeM);

    MockFlightScenarioId scenario_ = MockFlightScenarioId::NOMINAL;
    float altitudeWindowM_[NuraConstants::Sensors::kBarometerMedianWindowSamples] = {0.0f, 0.0f, 0.0f};
    uint8_t altitudeWindowHead_ = 0U;
    uint8_t altitudeWindowCount_ = 0U;
    float filteredAltitudeM_ = 0.0f;
    bool filterReady_ = false;
};
