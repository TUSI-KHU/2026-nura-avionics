#include "hal/bmp390_hal.h"

#include <Arduino.h>
#include <math.h>

bool BMP390HAL::begin(TwoWire &wire, uint8_t i2cAddress)
{
    initialized_ = sensor_.begin_I2C(i2cAddress, &wire);
    if (!initialized_)
    {
        return false;
    }

    sensor_.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    sensor_.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    sensor_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    sensor_.setOutputDataRate(BMP3_ODR_50_HZ);
    return true;
}

Bmp390PollResult BMP390HAL::poll(Bmp390Reading &out, uint32_t nowMs)
{
    return read(out, nowMs) ? Bmp390PollResult::READY : Bmp390PollResult::ERROR;
}

bool BMP390HAL::read(Bmp390Reading &out, uint32_t nowMs)
{
    if (!initialized_ || !sensor_.performReading())
    {
        return false;
    }

    if (!isfinite(sensor_.pressure) || sensor_.pressure <= 0.0f)
    {
        return false;
    }

    out.pressurePa = sensor_.pressure;
    out.pressureHpa = sensor_.pressure / 100.0f;
    out.temperatureC = sensor_.temperature;
    out.sampleMs = nowMs;
    return true;
}
