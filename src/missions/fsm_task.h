#pragma once

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/tasks.h"
#include "state/abort_state.h"
#include "state/flight_state.h"
#include "hal/panic_handler.h"

class FlightStateMachineTask : public Task
{
public:
    // 전체 시스템 상태 전이를 담당하는 상태 머신 태스크다.
    FlightStateMachineTask(FlightState &flightState,
                           AbortState &abortState,
                           Logger &logger,
                           const IAppConfig &config,
                           IPanicHandler &panicHandler);

    const char *name() const;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    void transitionTo(State next, uint32_t nowMs);

    FlightState &flightState_;
    AbortState &abortState_;
    Logger &logger_;
    const IAppConfig &config_;
    IPanicHandler &panicHandler_;
};
