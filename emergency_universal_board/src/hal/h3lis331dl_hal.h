#pragma once

#include <stdint.h>
#include <stddef.h>

#include <Arduino.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

enum class H3LIS331DLRange : uint8_t
{
    RANGE_100G,
    RANGE_200G,
    RANGE_400G
};

struct H3LIS331DLReading
{
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    uint8_t whoAmI = 0;
    uint32_t sampleMs = 0;
};

class H3LIS331DLHAL
{
public:
    bool begin(uint8_t csPin,
               SPIClass &spi = SPI,
               H3LIS331DLRange range = H3LIS331DLRange::RANGE_100G);
    bool read(H3LIS331DLReading &out, uint32_t nowMs);
    uint8_t readWhoAmI();

private:
    static bool validEvent(const sensors_event_t &event);
    static h3lis331dl_range_t toAdafruitRange(H3LIS331DLRange range);

    Adafruit_H3LIS331 sensor_;
    uint8_t whoAmI_ = 0;
    bool initialized_ = false;
};
