#include "fsm_task.h"

FlightStateMachineTask::FlightStateMachineTask(FlightState &flightState,
                                               AbortState &abortState,
                                               Logger &logger,
                                               const IAppConfig &config,
                                               IPanicHandler &panicHandler)
    : flightState_(flightState),
      abortState_(abortState),
      logger_(logger),
      config_(config),
      panicHandler_(panicHandler) {}

const char *FlightStateMachineTask::name() const
{
    return "fsm";
}

bool FlightStateMachineTask::init()
{
    flightState_.state = State::BOOT;
    flightState_.stateEnteredMs = 0;
    return true;
}

bool FlightStateMachineTask::tick(uint32_t nowMs)
{
    // 현재는 abort 활성 여부만으로 시스템 health를 단순 판단한다.
    const bool healthy = !abortState_.status.active;

    if (abortState_.status.active && flightState_.state != State::SAFE)
    {
        transitionTo(State::SAFE, nowMs);
        return true;
    }

    if (!healthy && flightState_.state != State::BOOT)
    {
        transitionTo(State::SAFE, nowMs);
        return true;
    }

    switch (flightState_.state)
    {
    case State::BOOT:
        // 부팅 단계에서는 최소 health 조건을 만족해야 IDLE로 진입한다.
        if (healthy)
        {
            transitionTo(State::IDLE, nowMs);
        }
        else
        {
            panicHandler_.panic();
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
        transitionTo(State::SAFE, nowMs);
        break;
    }

    return true;
}

uint32_t FlightStateMachineTask::periodMs() const
{
    return config_.flightStateTaskPeriodMs();
}

void FlightStateMachineTask::transitionTo(State next, uint32_t nowMs)
{
    // 동일 상태 재전이는 무시하고, 실제 전이 시에만 로그를 남긴다.
    if (flightState_.state == next)
    {
        return;
    }

    LOGI(logger_, nowMs, "fsm", stateName(next));

    flightState_.state = next;
    flightState_.stateEnteredMs = nowMs;
}
