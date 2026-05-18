#pragma once

#include <stdint.h>

#include <Adafruit_MPL3115A2.h>
#include <Wire.h>

struct Mpl3115a2Reading
{
    float pressurePa = 0.0f;
    float pressureHpa = 0.0f;
    float temperatureC = 0.0f;
    float relativeAltitudeM = 0.0f;
    uint32_t sampleMs = 0;
};

class MPL3115A2HAL
{
public:
    bool begin(TwoWire &wire = Wire1,
               uint16_t conversionTimeoutMs = 700U,
               float seaLevelPressureHpa = 1013.25f);
    bool read(Mpl3115a2Reading &out, uint32_t nowMs);
    void setSeaLevelPressureHpa(float seaLevelPressureHpa);
    bool calibrateGroundBaseline(uint16_t sampleCount = 64U,
                                 uint16_t sampleDelayMs = 20U,
                                 float knownGroundAltitudeM = 0.0f);
    void clearGroundBaseline();
    bool groundBaselineValid() const;

private:
    bool waitForConversion();
    static bool validPressure(float pressurePa);
    static float pressureToAltitudeM(float pressurePa, float referencePressurePa);
    static float pressureToSeaLevelPressureHpa(float pressureHpa, float altitudeM);

    Adafruit_MPL3115A2 sensor_;
    uint16_t conversionTimeoutMs_ = 700U;
    float seaLevelPressureHpa_ = 1013.25f;
    float groundPressurePa_ = 0.0f;
    bool initialized_ = false;
    bool groundBaselineValid_ = false;
};
