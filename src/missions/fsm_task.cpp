#include "fsm_task.h"
#include <Arduino.h>

const char* FlightStateMachineTask::name() const {
    return "fsm";
}

bool FlightStateMachineTask::init(SystemContext& ctx) {
    ctx.state = State::BOOT;
    ctx.stateEnteredMs = 0;
    nextBootPrintMs_ = 0;
    return true;
}

bool FlightStateMachineTask::tick(SystemContext& ctx, uint32_t nowMs) {
    const bool healthy = ctx.health.imuOk && ctx.health.baroOk;

    if (!healthy && ctx.state != State::BOOT) {
        transitionTo(ctx, State::SAFE, nowMs);
        return true;
    }

    switch (ctx.state) {
        case State::BOOT:
            // Serial.print(F("[FSM] BOOT test: nowMs="));
            // Serial.println(nowMs);
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

uint32_t FlightStateMachineTask::periodMs() const {
    return 1000;
}

void FlightStateMachineTask::transitionTo(SystemContext& ctx, State next, uint32_t nowMs) {
    if (ctx.state == next) {
        return;
    }

    ctx.state = next;
    ctx.stateEnteredMs = nowMs;
}
