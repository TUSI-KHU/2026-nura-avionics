#pragma once

#include "core/contexts.h"
#include "core/tasks.h"

class FlightStateMachineTask : public Task {
public:
    const char* name() const;
    bool init(SystemContext& ctx);
    bool tick(SystemContext& ctx, uint32_t nowMs);
    uint32_t periodMs() const;

private:
    void transitionTo(SystemContext& ctx, State next, uint32_t nowMs);
    uint32_t nextBootPrintMs_ = 0;
};
