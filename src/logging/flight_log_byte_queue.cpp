#include "flight_log_byte_queue.h"

#include <string.h>

bool FlightLogByteQueue::push(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length == 0U || length > free())
    {
        return false;
    }

    const uint16_t first = static_cast<uint16_t>(
        length < (kCapacity - head_) ? length : (kCapacity - head_));
    memcpy(buffer_ + head_, data, first);
    const uint16_t second = static_cast<uint16_t>(length - first);
    if (second > 0U)
    {
        memcpy(buffer_, data + first, second);
    }

    head_ = static_cast<uint16_t>((head_ + length) % kCapacity);
    used_ = static_cast<uint16_t>(used_ + length);
    return true;
}

uint16_t FlightLogByteQueue::peek(uint8_t *out, uint16_t maxLength) const
{
    if (out == nullptr || maxLength == 0U || used_ == 0U)
    {
        return 0U;
    }

    const uint16_t length = used_ < maxLength ? used_ : maxLength;
    const uint16_t first = static_cast<uint16_t>(
        length < (kCapacity - tail_) ? length : (kCapacity - tail_));
    memcpy(out, buffer_ + tail_, first);
    const uint16_t second = static_cast<uint16_t>(length - first);
    if (second > 0U)
    {
        memcpy(out + first, buffer_, second);
    }
    return length;
}

bool FlightLogByteQueue::consume(uint16_t length)
{
    if (length > used_)
    {
        return false;
    }
    tail_ = static_cast<uint16_t>((tail_ + length) % kCapacity);
    used_ = static_cast<uint16_t>(used_ - length);
    return true;
}

void FlightLogByteQueue::clear()
{
    head_ = 0U;
    tail_ = 0U;
    used_ = 0U;
}

uint16_t FlightLogByteQueue::used() const
{
    return used_;
}

uint16_t FlightLogByteQueue::free() const
{
    return static_cast<uint16_t>(kCapacity - used_);
}

bool FlightLogByteQueue::empty() const
{
    return used_ == 0U;
}
