#pragma once
#include <stdint.h>

// 모든 주기성 작업이 따라야 하는 공통 인터페이스다.
class Task
{
public:
    virtual ~Task() = default;
    virtual const char *name() const = 0;

    virtual bool init() = 0;
    virtual bool tick(uint32_t nowMs) = 0;

    // Desired interval
    virtual uint32_t periodMs() const = 0;
};
