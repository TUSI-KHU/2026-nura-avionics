#pragma once

#include <stddef.h>
#include <stdint.h>

class IFlightLogStorage
{
public:
    virtual ~IFlightLogStorage() = default;
    virtual bool begin() = 0;
    virtual bool append(const uint8_t *data, uint16_t length) = 0;
    virtual bool flush() = 0;
    virtual void stop() = 0;
    virtual bool healthy() const = 0;
};

class NullFlightLogStorage : public IFlightLogStorage
{
public:
    bool begin() override { return false; }
    bool append(const uint8_t *data, uint16_t length) override
    {
        (void)data;
        (void)length;
        return false;
    }
    bool flush() override { return false; }
    void stop() override {}
    bool healthy() const override { return false; }
};
