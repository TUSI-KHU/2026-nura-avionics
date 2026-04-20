#pragma once

#include "core/contexts.h"
#include "core/tasks.h"

class FlightStateMachineTask : public Task
{
public:
    // 전체 시스템 상태 전이를 담당하는 상태 머신 태스크다.
    const char *name() const;
    bool init(SystemContext &ctx);
    bool tick(SystemContext &ctx, uint32_t nowMs);
    uint32_t periodMs() const;

private:
    void transitionTo(SystemContext &ctx, State next, uint32_t nowMs);
};
