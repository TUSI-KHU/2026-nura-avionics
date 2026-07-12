#pragma once

#include <stdint.h>

#include <Adafruit_BMP3XX.h>
#include <Wire.h>

struct Bmp390Reading
{
    float pressurePa = 0.0f;
    float pressureHpa = 0.0f;
    float temperatureC = 0.0f;
    uint32_t sampleMs = 0;
};

enum class Bmp390PollResult : uint8_t
{
    PENDING,
    READY,
    ERROR
};

class BMP390HAL
{
public:
    bool begin(TwoWire &wire = Wire, uint8_t i2cAddress = 0x77U);
    Bmp390PollResult poll(Bmp390Reading &out, uint32_t nowMs);
    bool read(Bmp390Reading &out, uint32_t nowMs);

private:
    Adafruit_BMP3XX sensor_;
    bool initialized_ = false;
};
