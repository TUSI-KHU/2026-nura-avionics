#pragma once

#include <Arduino.h>

class DigitalOutputHAL
{
public:
    explicit DigitalOutputHAL(uint8_t pin = LED_BUILTIN)
        : pin_(pin)
    {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, LOW);
    }

    void write(bool high) const
    {
        digitalWrite(pin_, high ? HIGH : LOW);
    }

private:
    uint8_t pin_;
};
