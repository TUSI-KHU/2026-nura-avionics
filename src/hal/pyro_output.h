#pragma once

#include <stdint.h>

enum class PyroChannel : uint8_t
{
    DROGUE = 0U,
    MAIN = 1U,
};

class IPyroOutput
{
public:
    virtual ~IPyroOutput() = default;

    virtual bool begin() = 0;
    virtual void allOff() = 0;
    virtual void setDrogue(bool enabled) = 0;
    virtual void setMain(bool enabled) = 0;
};

class NullPyroOutput final : public IPyroOutput
{
public:
    bool begin() override { return true; }
    void allOff() override {}
    void setDrogue(bool enabled) override { (void)enabled; }
    void setMain(bool enabled) override { (void)enabled; }
};
