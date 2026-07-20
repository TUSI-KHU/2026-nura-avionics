#pragma once

#include <stddef.h>
#include <stdint.h>

class IFlightLogStorage
{
public:
    virtual ~IFlightLogStorage() = default;
    virtual bool begin() = 0;
    virtual bool canAppend(uint16_t length) const = 0;
    virtual bool append(const uint8_t *data, uint16_t length) = 0;
    virtual bool service(uint32_t nowMs) = 0;
    virtual bool requestFlush() = 0;
    virtual bool idle() const = 0;
    virtual void stop() = 0;
    virtual bool healthy() const = 0;
};

class NullFlightLogStorage : public IFlightLogStorage
{
public:
    bool begin() override { return false; }
    bool canAppend(uint16_t length) const override
    {
        (void)length;
        return false;
    }
    bool append(const uint8_t *data, uint16_t length) override
    {
        (void)data;
        (void)length;
        return false;
    }
    bool service(uint32_t nowMs) override
    {
        (void)nowMs;
        return false;
    }
    bool requestFlush() override { return false; }
    bool idle() const override { return true; }
    void stop() override {}
    bool healthy() const override { return false; }
};
