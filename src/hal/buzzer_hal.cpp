#include "hal/buzzer_hal.h"

#include <Arduino.h>

BuzzerHAL::BuzzerHAL(uint8_t pin)
    : pin_(pin)
{
}

bool BuzzerHAL::begin()
{
    pinMode(pin_, OUTPUT);
    initialized_ = true;
    silence();
    return true;
}

void BuzzerHAL::playTone(uint16_t frequencyHz)
{
    if (!initialized_ || frequencyHz == 0U)
    {
        silence();
        return;
    }

    tone(pin_, frequencyHz);
}

void BuzzerHAL::silence()
{
    if (!initialized_)
    {
        return;
    }

    noTone(pin_);
    digitalWrite(pin_, LOW);
}
