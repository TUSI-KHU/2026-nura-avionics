#pragma once

#include <stdint.h>

#include <MS5611.h>
#include <Wire.h>

struct Ms5611Reading
{
    float pressurePa = 0.0f;
    float pressureMbar = 0.0f;
    float temperatureC = 0.0f;
    float altitudeM = 0.0f;
    float relativeAltitudeM = 0.0f;
    uint32_t sampleMs = 0;
    uint32_t deviceId = 0;
};

class MS5611HAL
{
public:
    bool begin(uint8_t i2cAddress = MS5611_DEFAULT_ADDRESS,
               TwoWire &wire = Wire1,
               osr_t oversampling = OSR_HIGH,
               float seaLevelPressureMbar = 1013.25f);
    bool read(Ms5611Reading &out, uint32_t nowMs);

    uint16_t promWord(uint8_t index);
    uint16_t promCrc();
    bool promCrcValid() const;

    void setSeaLevelPressureMbar(float seaLevelPressureMbar);
    bool calibrateGroundBaseline(uint16_t sampleCount = 64U,
                                 uint16_t sampleDelayMs = 20U,
                                 float knownGroundAltitudeM = 0.0f);
    void clearGroundBaseline();
    bool groundBaselineValid() const;

private:
    bool promLooksValid();
    static uint8_t calculatePromCrc4(const uint16_t prom[8]);
    static bool validPressure(float pressurePa);
    static float pressureToAltitudeM(float pressurePa, float referencePressurePa);
    static float pressureToSeaLevelPressureMbar(float pressureMbar, float altitudeM);

    MS5611 sensor_;
    osr_t oversampling_ = OSR_HIGH;
    float seaLevelPressureMbar_ = 1013.25f;
    float groundPressurePa_ = 0.0f;
    uint16_t prom_[8] = {0};
    bool initialized_ = false;
    bool promCrcValid_ = false;
    bool groundBaselineValid_ = false;
};
