#include "fsm_task.h"

const char *FlightStateMachineTask::name() const
{
    return "fsm";
}

bool FlightStateMachineTask::init(SystemContext &ctx)
{
    ctx.state = State::BOOT;
    ctx.stateEnteredMs = 0;
    nextBootPrintMs_ = 0;
    return true;
}

bool FlightStateMachineTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    const bool healthy = ctx.health.imuOk;

    if (!healthy && ctx.state != State::BOOT)
    {
        transitionTo(ctx, State::SAFE, nowMs);
        return true;
    }

    switch (ctx.state)
    {
    case State::BOOT:
        LOGD(ctx.logger, nowMs, "fsm", "boot state heartbeat");
        break;

    case State::IDLE:
        break;

    case State::ARMED:
        break;

    case State::LAUNCH:
        break;

    case State::DESCENT:
        break;

    case State::GROUND:
        break;

    case State::SAFE:
        break;

    default:
        transitionTo(ctx, State::SAFE, nowMs);
        break;
    }

    return true;
}

uint32_t FlightStateMachineTask::periodMs() const
{
    return 100;
}

void FlightStateMachineTask::transitionTo(SystemContext &ctx, State next, uint32_t nowMs)
{
    if (ctx.state == next)
    {
        return;
    }

    ctx.state = next;
    ctx.stateEnteredMs = nowMs;
}
