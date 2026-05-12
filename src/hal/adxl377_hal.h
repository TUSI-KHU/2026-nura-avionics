#pragma once

#include <stdint.h>

#include <Arduino.h>

struct Adxl377Reading
{
    // ADXL377 is analog-output, so keep raw ADC and converted values together.
    uint16_t rawX = 0;
    uint16_t rawY = 0;
    uint16_t rawZ = 0;

    float voltageX = 0.0f;
    float voltageY = 0.0f;
    float voltageZ = 0.0f;

    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    uint32_t sampleMs = 0;
};

struct Adxl377Calibration
{
    float zeroGVoltageX = 0.0f;
    float zeroGVoltageY = 0.0f;
    float zeroGVoltageZ = 0.0f;
    float sensitivityMvPerG = 0.0f;
};

class ADXL377HAL
{
public:
    bool begin(uint8_t xPin,
               uint8_t yPin,
               uint8_t zPin,
               float referenceVoltage = 3.3f,
               uint8_t adcResolutionBits = 12U,
               float zeroGVoltage = 0.0f,
               float sensitivityMvPerG = 0.0f);
    bool read(Adxl377Reading &out, uint32_t nowMs) const;
    void setCalibration(const Adxl377Calibration &calibration);
    void clearCalibration();
    bool calibrateStationary(uint16_t sampleCount = 256U,
                             uint16_t sampleDelayMs = 2U,
                             float expectedAccelXG = 0.0f,
                             float expectedAccelYG = 0.0f,
                             float expectedAccelZG = 1.0f);
    Adxl377Calibration calibration() const;

private:
    float rawToVoltage(uint16_t raw) const;
    float voltageToG(float voltage, float zeroGVoltage) const;

    uint8_t xPin_ = 0U;
    uint8_t yPin_ = 0U;
    uint8_t zPin_ = 0U;
    uint16_t adcMax_ = 0U;
    float referenceVoltage_ = 3.3f;
    float zeroGVoltage_ = 1.65f;
    float zeroGVoltageX_ = 1.65f;
    float zeroGVoltageY_ = 1.65f;
    float zeroGVoltageZ_ = 1.65f;
    float sensitivityVPerG_ = 0.00715f;
    bool initialized_ = false;
};
