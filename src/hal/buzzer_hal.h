#pragma once

#include <stdint.h>

#include "hal/buzzer_output.h"

class BuzzerHAL final : public IBuzzerOutput
{
public:
    explicit BuzzerHAL(uint8_t pin);

    bool begin() override;
    void playTone(uint16_t frequencyHz) override;
    void silence() override;

private:
    uint8_t pin_;
    bool initialized_ = false;
};
