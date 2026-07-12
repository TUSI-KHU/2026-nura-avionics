#include "flight_log_ram_buffer.h"

bool FlightLogRamBuffer::push(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length == 0U || (length + kLengthBytes) > kCapacity)
    {
        return false;
    }

    const uint16_t needed = static_cast<uint16_t>(length + kLengthBytes);
    while (freeBytes() < needed)
    {
        if (!dropOldest())
        {
            return false;
        }
    }

    writeByte(static_cast<uint8_t>(length & 0xFFU));
    writeByte(static_cast<uint8_t>((length >> 8U) & 0xFFU));
    for (uint16_t i = 0U; i < length; ++i)
    {
        writeByte(data[i]);
    }

    used_ = static_cast<uint16_t>(used_ + needed);
    ++records_;
    return true;
}

bool FlightLogRamBuffer::pop(uint8_t *out, uint16_t outCapacity, uint16_t &length)
{
    length = 0U;
    if (out == nullptr || records_ == 0U || used_ < kLengthBytes)
    {
        return false;
    }

    const uint16_t recordLength = peekLength();
    const uint16_t totalLength = static_cast<uint16_t>(recordLength + kLengthBytes);
    if (recordLength > outCapacity || totalLength > used_)
    {
        return false;
    }

    advance(tail_, kLengthBytes);
    for (uint16_t i = 0U; i < recordLength; ++i)
    {
        out[i] = readByte(tail_);
        advance(tail_, 1U);
    }

    used_ = static_cast<uint16_t>(used_ - totalLength);
    --records_;
    length = recordLength;
    return true;
}

bool FlightLogRamBuffer::empty() const
{
    return records_ == 0U;
}

uint16_t FlightLogRamBuffer::used() const
{
    return used_;
}

uint16_t FlightLogRamBuffer::capacity() const
{
    return kCapacity;
}

uint16_t FlightLogRamBuffer::recordCount() const
{
    return records_;
}

uint32_t FlightLogRamBuffer::droppedRecords() const
{
    return droppedRecords_;
}

void FlightLogRamBuffer::clear()
{
    head_ = 0U;
    tail_ = 0U;
    used_ = 0U;
    records_ = 0U;
    droppedRecords_ = 0U;
}

void FlightLogRamBuffer::writeByte(uint8_t value)
{
    buffer_[head_] = value;
    advance(head_, 1U);
}

uint8_t FlightLogRamBuffer::readByte(uint16_t index) const
{
    return buffer_[index % kCapacity];
}

void FlightLogRamBuffer::advance(uint16_t &index, uint16_t amount) const
{
    index = static_cast<uint16_t>((index + amount) % kCapacity);
}

uint16_t FlightLogRamBuffer::peekLength() const
{
    const uint8_t low = readByte(tail_);
    const uint8_t high = readByte(static_cast<uint16_t>((tail_ + 1U) % kCapacity));
    return static_cast<uint16_t>(low | (static_cast<uint16_t>(high) << 8U));
}

bool FlightLogRamBuffer::dropOldest()
{
    if (records_ == 0U || used_ < kLengthBytes)
    {
        return false;
    }

    const uint16_t recordLength = peekLength();
    const uint16_t totalLength = static_cast<uint16_t>(recordLength + kLengthBytes);
    if (totalLength > used_)
    {
        clear();
        return false;
    }

    advance(tail_, totalLength);
    used_ = static_cast<uint16_t>(used_ - totalLength);
    --records_;
    ++droppedRecords_;
    return true;
}

uint16_t FlightLogRamBuffer::freeBytes() const
{
    return static_cast<uint16_t>(kCapacity - used_);
}
