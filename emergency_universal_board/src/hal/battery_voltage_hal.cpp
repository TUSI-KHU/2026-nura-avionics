#include "battery_voltage_hal.h"

#include <Arduino.h>

bool BatteryVoltageHAL::begin(uint8_t pin,
                              uint16_t adcReferenceMv,
                              uint8_t adcResolutionBits,
                              uint16_t dividerRatioNumerator,
                              uint16_t dividerRatioDenominator,
                              uint16_t minValidBatteryMv,
                              uint16_t maxValidBatteryMv)
{
    if (adcReferenceMv == 0U ||
        adcResolutionBits == 0U ||
        adcResolutionBits > 16U ||
        dividerRatioNumerator == 0U ||
        dividerRatioDenominator == 0U ||
        minValidBatteryMv > maxValidBatteryMv)
    {
        initialized_ = false;
        return false;
    }

    pin_ = pin;
    adcReferenceMv_ = adcReferenceMv;
    adcMax_ = static_cast<uint16_t>((1UL << adcResolutionBits) - 1UL);
    dividerRatioNumerator_ = dividerRatioNumerator;
    dividerRatioDenominator_ = dividerRatioDenominator;
    minValidBatteryMv_ = minValidBatteryMv;
    maxValidBatteryMv_ = maxValidBatteryMv;

    pinMode(pin_, INPUT);
    analogReadResolution(adcResolutionBits);
    initialized_ = true;
    return true;
}

bool BatteryVoltageHAL::read(BatteryVoltageReading &out, uint32_t nowMs) const
{
    out = BatteryVoltageReading{};
    out.sampleMs = nowMs;

    if (!initialized_ || adcMax_ == 0U)
    {
        return false;
    }

    const int raw = analogRead(pin_);
    if (raw < 0)
    {
        return false;
    }

    out.rawAdc = raw > static_cast<int>(adcMax_) ? adcMax_ : static_cast<uint16_t>(raw);
    out.senseMv = scaleRawToSenseMv(out.rawAdc, adcMax_, adcReferenceMv_);
    out.batteryMv = scaleRawToBatteryMv(out.rawAdc,
                                         adcMax_,
                                         adcReferenceMv_,
                                         dividerRatioNumerator_,
                                         dividerRatioDenominator_);
    out.valid = out.batteryMv >= minValidBatteryMv_ && out.batteryMv <= maxValidBatteryMv_;
    return true;
}

uint16_t BatteryVoltageHAL::scaleRawToSenseMv(uint16_t raw, uint16_t adcMax, uint16_t adcReferenceMv)
{
    if (adcMax == 0U)
    {
        return 0U;
    }

    const uint32_t numerator = static_cast<uint32_t>(raw) * static_cast<uint32_t>(adcReferenceMv);
    return static_cast<uint16_t>((numerator + (static_cast<uint32_t>(adcMax) / 2UL)) /
                                 static_cast<uint32_t>(adcMax));
}

uint16_t BatteryVoltageHAL::scaleRawToBatteryMv(uint16_t raw,
                                                uint16_t adcMax,
                                                uint16_t adcReferenceMv,
                                                uint16_t dividerRatioNumerator,
                                                uint16_t dividerRatioDenominator)
{
    if (adcMax == 0U || dividerRatioDenominator == 0U)
    {
        return 0U;
    }

    const uint64_t numerator = static_cast<uint64_t>(raw) * static_cast<uint64_t>(adcReferenceMv) *
                               static_cast<uint64_t>(dividerRatioNumerator);
    const uint64_t denominator = static_cast<uint64_t>(adcMax) *
                                 static_cast<uint64_t>(dividerRatioDenominator);
    const uint64_t scaled = (numerator + (denominator / 2ULL)) / denominator;
    return scaled > 65535UL ? 65535U : static_cast<uint16_t>(scaled);
}
