#pragma once

#include <stdint.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct Lis3mdlReading
{
    float magXuT = 0.0f;
    float magYuT = 0.0f;
    float magZuT = 0.0f;

    float magXGauss = 0.0f;
    float magYGauss = 0.0f;
    float magZGauss = 0.0f;

    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    uint32_t sampleMs = 0;
};

struct Lis3mdlCalibration
{
    float hardIronXuT = 0.0f;
    float hardIronYuT = 0.0f;
    float hardIronZuT = 0.0f;

    float scaleX = 1.0f;
    float scaleY = 1.0f;
    float scaleZ = 1.0f;
};

class LIS3MDLHAL
{
public:
    bool begin(uint8_t i2cAddress = LIS3MDL_I2CADDR_DEFAULT,
               TwoWire &wire = Wire1,
               lis3mdl_range_t range = LIS3MDL_RANGE_16_GAUSS,
               lis3mdl_dataRate_t dataRate = LIS3MDL_DATARATE_155_HZ,
               lis3mdl_performancemode_t performanceMode = LIS3MDL_ULTRAHIGHMODE);
    bool read(Lis3mdlReading &out, uint32_t nowMs);
    void setCalibration(const Lis3mdlCalibration &calibration);
    void clearCalibration();
    bool calibrateHardIron(uint32_t durationMs = 15000UL,
                           uint16_t sampleDelayMs = 20U);
    const Lis3mdlCalibration &calibration() const;

private:
    static bool validEvent(const sensors_event_t &event);

    Adafruit_LIS3MDL sensor_;
    Lis3mdlCalibration calibration_;
    bool initialized_ = false;
};
