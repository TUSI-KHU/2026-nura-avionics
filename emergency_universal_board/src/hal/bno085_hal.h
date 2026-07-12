#pragma once

#include <stdint.h>

#include <Adafruit_BNO08x.h>
#include <Wire.h>

struct Bno085Reading
{
    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;
    float gyroXDps = 0.0f;
    float gyroYDps = 0.0f;
    float gyroZDps = 0.0f;
    bool attitudeValid = false;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;
    uint32_t sampleMs = 0;
};

class BNO085HAL
{
public:
    explicit BNO085HAL(int8_t resetPin = -1);

    bool begin(TwoWire &wire = Wire, uint8_t i2cAddress = 0x4AU);
    bool read(Bno085Reading &out, uint32_t nowMs);

private:
    bool enableReports();
    void publishQuaternion(float real, float i, float j, float k, Bno085Reading &out) const;

    Adafruit_BNO08x sensor_;
    bool initialized_ = false;
    bool haveAccel_ = false;
    bool haveGyro_ = false;
    Bno085Reading latest_;
};
