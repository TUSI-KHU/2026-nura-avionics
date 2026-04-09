#include "logger.h"

Logger::Logger()
    : head_(0), tail_(0), count_(0), dropped_(0)
{
}

bool Logger::push(const LogEntry &e)
{
    if (!isLevelEnabled(e.level))
    {
        return true;
    }

    if (count_ >= kMaxBufferSize)
    {
        ++dropped_;
        return false;
    }

    buffer_[head_] = e;
    head_ = nextIndex(head_);
    ++count_;
    return true;
}

bool Logger::push(
    uint32_t ts,
    LogLevel level,
    const char *src,
    const char *msg)
{
    LogEntry e;
    e.ts = ts;
    e.level = level;
    e.src = src;
    e.msg = msg;

    return push(e);
}

bool Logger::pop(LogEntry &out)
{
    if (count_ == 0)
    {
        return false;
    }

    out = buffer_[tail_];
    tail_ = nextIndex(tail_);
    --count_;
    return true;
}

bool Logger::empty() const
{
    return count_ == 0;
}

uint8_t Logger::size() const
{
    return count_;
}

uint32_t Logger::droppedCount() const
{
    return dropped_;
}

bool Logger::isLevelEnabled(LogLevel level)
{
    return static_cast<uint8_t>(level) <= static_cast<uint8_t>(LOG_LEVEL);
}

void Logger::clear()
{
    head_ = 0;
    tail_ = 0;
    count_ = 0;
    dropped_ = 0;
}

uint8_t Logger::nextIndex(uint8_t index)
{
    ++index;
    if (index >= kMaxBufferSize)
    {
        index = 0;
    }
    return index;
}
