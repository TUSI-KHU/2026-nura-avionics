#pragma once

#include <stdint.h>

#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct H3lis331dlReading
{
    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    uint32_t sampleMs = 0;
};

struct H3lis331dlCalibration
{
    float accelXGOffset = 0.0f;
    float accelYGOffset = 0.0f;
    float accelZGOffset = 0.0f;
};

class H3LIS331DLHAL
{
public:
    bool begin(uint8_t i2cAddress = LIS331_DEFAULT_ADDRESS,
               TwoWire &wire = Wire,
               h3lis331dl_range_t range = H3LIS331_RANGE_200_G,
               lis331_data_rate_t dataRate = LIS331_DATARATE_1000_HZ);
    bool read(H3lis331dlReading &out, uint32_t nowMs);
    void setCalibration(const H3lis331dlCalibration &calibration);
    void clearCalibration();
    bool calibrateStationary(uint16_t sampleCount = 128U,
                             uint16_t sampleDelayMs = 2U,
                             float expectedAccelXG = 0.0f,
                             float expectedAccelYG = 0.0f,
                             float expectedAccelZG = 1.0f);
    const H3lis331dlCalibration &calibration() const;

private:
    static bool validEvent(const sensors_event_t &event);

    Adafruit_H3LIS331 sensor_;
    H3lis331dlCalibration calibration_;
    bool initialized_ = false;
};
