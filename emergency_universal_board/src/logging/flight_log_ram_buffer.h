#pragma once

#include <stddef.h>
#include <stdint.h>

#include "nura_constants.h"

class FlightLogRamBuffer
{
public:
    bool push(const uint8_t *data, uint16_t length);
    bool pop(uint8_t *out, uint16_t outCapacity, uint16_t &length);
    bool empty() const;
    uint16_t used() const;
    uint16_t capacity() const;
    uint16_t recordCount() const;
    uint32_t droppedRecords() const;
    void clear();

private:
    static constexpr uint16_t kCapacity = NuraConstants::Logger::kFlightLogRamBufferBytes;
    static constexpr uint16_t kLengthBytes = 2U;

    void writeByte(uint8_t value);
    uint8_t readByte(uint16_t index) const;
    void advance(uint16_t &index, uint16_t amount) const;
    uint16_t peekLength() const;
    bool dropOldest();
    uint16_t freeBytes() const;

    uint8_t buffer_[kCapacity] = {};
    uint16_t head_ = 0U;
    uint16_t tail_ = 0U;
    uint16_t used_ = 0U;
    uint16_t records_ = 0U;
    uint32_t droppedRecords_ = 0U;
};
