#pragma once

#include <Arduino.h>

class DigitalOutputHAL
{
public:
    // 단순 출력 핀 HAL이므로 생성 시 즉시 pinMode까지 마친다.
    explicit DigitalOutputHAL(uint8_t pin = LED_BUILTIN)
        : pin_(pin)
    {
        pinMode(pin_, OUTPUT);
        digitalWrite(pin_, LOW);
    }

    void write(bool high) const
    {
        // bool 값을 Arduino HIGH/LOW로 변환해 출력한다.
        digitalWrite(pin_, high ? HIGH : LOW);
    }

private:
    uint8_t pin_;
};
