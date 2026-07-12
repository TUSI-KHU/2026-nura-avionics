#pragma once

#include <stdint.h>

struct BatteryVoltageReading
{
    bool valid = false;
    uint16_t rawAdc = 0U;
    uint16_t senseMv = 0U;
    uint16_t batteryMv = 0U;
    uint32_t sampleMs = 0UL;
};

class BatteryVoltageHAL
{
public:
    bool begin(uint8_t pin,
               uint16_t adcReferenceMv,
               uint8_t adcResolutionBits,
               uint16_t dividerRatioNumerator,
               uint16_t dividerRatioDenominator,
               uint16_t minValidBatteryMv,
               uint16_t maxValidBatteryMv);
    bool read(BatteryVoltageReading &out, uint32_t nowMs) const;

private:
    static uint16_t scaleRawToSenseMv(uint16_t raw, uint16_t adcMax, uint16_t adcReferenceMv);
    static uint16_t scaleRawToBatteryMv(uint16_t raw,
                                        uint16_t adcMax,
                                        uint16_t adcReferenceMv,
                                        uint16_t dividerRatioNumerator,
                                        uint16_t dividerRatioDenominator);

    uint8_t pin_ = 0U;
    uint16_t adcMax_ = 0U;
    uint16_t adcReferenceMv_ = 0U;
    uint16_t dividerRatioNumerator_ = 0U;
    uint16_t dividerRatioDenominator_ = 0U;
    uint16_t minValidBatteryMv_ = 0U;
    uint16_t maxValidBatteryMv_ = 0U;
    bool initialized_ = false;
};
