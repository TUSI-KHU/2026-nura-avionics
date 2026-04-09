#pragma once
#include <stdint.h>

struct SystemContext;

class Task {
public:
    virtual ~Task() = default;
    virtual const char* name() const = 0;

    virtual bool init(SystemContext& ctx) = 0;
    virtual bool tick(SystemContext& ctx, uint32_t nowMs) = 0;

    // Desired interval
    virtual uint32_t periodMs() const = 0;
};
