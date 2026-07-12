#pragma once

#include <cstdint>

namespace nura::core
{

class IMonotonicClock
{
public:
    virtual ~IMonotonicClock() = default;
    virtual uint64_t nowUs() const = 0;
};

} // namespace nura::core
