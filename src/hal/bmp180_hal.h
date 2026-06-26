#pragma once

#include <stdint.h>

#include <Wire.h>

#include "nura_constants.h"

struct Bmp180Reading
{
    float pressurePa = 0.0f;
    float pressureHpa = 0.0f;
    float temperatureC = 0.0f;
    uint32_t sampleMs = 0;
};

enum class Bmp180PollResult : uint8_t
{
    PENDING,
    READY,
    ERROR
};

class BMP180HAL
{
public:
    bool begin(TwoWire &wire = Wire,
               uint8_t i2cAddress = NuraConstants::BMP180::kI2cAddress,
               uint16_t conversionTimeoutMs = NuraConstants::BMP180::kConversionTimeoutMs);
    Bmp180PollResult poll(Bmp180Reading &out, uint32_t nowMs);
    bool read(Bmp180Reading &out, uint32_t nowMs);

private:
    enum class Conversion : uint8_t
    {
        NONE,
        TEMPERATURE,
        PRESSURE
    };

    bool readCalibration();
    bool startTemperature(uint32_t nowMs);
    bool startPressure(uint32_t nowMs);
    bool readTemperature();
    bool readPressure(Bmp180Reading &out, uint32_t nowMs);
    bool conversionReady(uint32_t nowMs) const;
    bool readU8(uint8_t reg, uint8_t &value) const;
    bool readS16(uint8_t reg, int16_t &value) const;
    bool readU16(uint8_t reg, uint16_t &value) const;
    bool readRawTemperature(int32_t &value) const;
    bool readRawPressure(int32_t &value) const;
    bool writeU8(uint8_t reg, uint8_t value) const;
    static bool validPressure(float pressurePa);

    TwoWire *wire_ = nullptr;
    uint8_t address_ = NuraConstants::BMP180::kI2cAddress;
    uint16_t conversionTimeoutMs_ = NuraConstants::BMP180::kConversionTimeoutMs;
    bool initialized_ = false;
    Conversion pending_ = Conversion::NONE;
    uint32_t conversionStartMs_ = 0UL;
    uint8_t pressureSamplesSinceTemp_ = 0U;
    int32_t b5_ = 0;
    float lastTemperatureC_ = 0.0f;

    int16_t ac1_ = 0;
    int16_t ac2_ = 0;
    int16_t ac3_ = 0;
    uint16_t ac4_ = 0;
    uint16_t ac5_ = 0;
    uint16_t ac6_ = 0;
    int16_t b1_ = 0;
    int16_t b2_ = 0;
    int16_t mb_ = 0;
    int16_t mc_ = 0;
    int16_t md_ = 0;
};
