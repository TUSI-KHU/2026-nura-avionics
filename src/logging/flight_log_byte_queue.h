#pragma once

#include <stddef.h>
#include <stdint.h>

#include "nura_constants.h"

class FlightLogByteQueue
{
public:
    bool push(const uint8_t *data, uint16_t length);
    uint16_t peek(uint8_t *out, uint16_t maxLength) const;
    bool consume(uint16_t length);
    void clear();

    uint16_t used() const;
    uint16_t free() const;
    bool empty() const;

private:
    static constexpr uint16_t kCapacity = NuraConstants::Logger::kFlightLogStorageQueueBytes;

    uint8_t buffer_[kCapacity] = {};
    uint16_t head_ = 0U;
    uint16_t tail_ = 0U;
    uint16_t used_ = 0U;
};
