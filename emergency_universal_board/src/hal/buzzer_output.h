#pragma once

#include <stdint.h>

class IBuzzerOutput
{
public:
    virtual ~IBuzzerOutput() = default;
    virtual bool begin() = 0;
    virtual void playTone(uint16_t frequencyHz) = 0;
    virtual void silence() = 0;
};

class NullBuzzerOutput final : public IBuzzerOutput
{
public:
    bool begin() override { return true; }
    void playTone(uint16_t frequencyHz) override { (void)frequencyHz; }
    void silence() override {}
};
