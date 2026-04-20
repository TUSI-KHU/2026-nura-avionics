#include "fsm_task.h"
#include "core/fault.h"

const char *FlightStateMachineTask::name() const
{
    return "fsm";
}

bool FlightStateMachineTask::init(SystemContext &ctx)
{
    ctx.state = State::BOOT;
    ctx.stateEnteredMs = 0;
    return true;
}

bool FlightStateMachineTask::tick(SystemContext &ctx, uint32_t nowMs)
{
    // 현재는 abort 활성 여부만으로 시스템 health를 단순 판단한다.
    const bool healthy = !ctx.abort.active;

    if (ctx.abort.active && ctx.state != State::SAFE)
    {
        transitionTo(ctx, State::SAFE, nowMs);
        return true;
    }

    if (!healthy && ctx.state != State::BOOT)
    {
        transitionTo(ctx, State::SAFE, nowMs);
        return true;
    }

    switch (ctx.state)
    {
    case State::BOOT:
        // 부팅 단계에서는 최소 health 조건을 만족해야 IDLE로 진입한다.
        if (healthy)
        {
            transitionTo(ctx, State::IDLE, nowMs);
        }
        else
        {
            hang();
        }
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
    return 100U;
}

void FlightStateMachineTask::transitionTo(SystemContext &ctx, State next, uint32_t nowMs)
{
    // 동일 상태 재전이는 무시하고, 실제 전이 시에만 로그를 남긴다.
    if (ctx.state == next)
    {
        return;
    }

    LOGI(ctx.logger, nowMs, "fsm", stateName(next));

    ctx.state = next;
    ctx.stateEnteredMs = nowMs;
}
