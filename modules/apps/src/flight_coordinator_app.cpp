#include "nura/apps/flight_coordinator_app.h"

namespace nura::apps
{
namespace c = nura::contracts;
namespace core = nura::core;

c::AppId FlightCoordinatorApp::id() const
{
    return c::AppId::FLIGHT_COORDINATOR;
}

core::AppRunResult FlightCoordinatorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const uint64_t started_us = clock_.nowUs();
    c::FlightInputs inputs{};
    if (!bus_.readFlightInputs(inputs, id(), cycle_id, started_us))
    {
        return core::AppRunResult::NO_INPUT;
    }

    nura::flight::CoordinatorOutput output{};
    if (!initialized_)
    {
        output = coordinator_.initialize(inputs, now_ms, clock_.nowUs());
        initialized_ = true;
    }
    else
    {
        c::CommandRequest command{};
        const c::CommandRequest *command_ptr =
            bus_.popCommand(command, id(), cycle_id, clock_.nowUs()) ? &command
                                                                    : nullptr;
        output = coordinator_.step(inputs, command_ptr, now_ms, clock_.nowUs(),
                                   cycle_id);
    }

    const uint64_t completed_us = clock_.nowUs();
    for (size_t i = 0U; i < output.transition_count; ++i)
    {
        const c::TransitionEvent &transition = output.transitions[i];
        c::TraceRecord record{};
        record.cycle_id = cycle_id;
        record.correlation_id = transition.sequence;
        record.timestamp_us = completed_us;
        record.app = transition.requested_by;
        record.peer = id();
        record.event = c::TraceEvent::TRANSITION_COMMIT;
        record.state = transition.previous;
        record.detail = static_cast<uint32_t>(transition.current);
        (void)trace_map_.tryRecord(record);
    }

    return publish(output, cycle_id, completed_us);
}

core::AppRunResult FlightCoordinatorApp::publish(
    const nura::flight::CoordinatorOutput &output, uint32_t cycle_id,
    uint64_t now_us)
{
    bool degraded = output.output_overflow;
    if (!bus_.publishFlightStatus(output.status, cycle_id))
    {
        return core::AppRunResult::FAILED;
    }
    if (!bus_.publishAppEnable(output.app_enable, cycle_id))
    {
        degraded = true;
    }

    for (size_t i = 0U; i < output.actuation_count; ++i)
    {
        if (!bus_.pushActuation(output.actuation[i], cycle_id, now_us))
        {
            return core::AppRunResult::FAILED;
        }
    }
    for (size_t i = 0U; i < output.transition_count; ++i)
    {
        if (!bus_.pushTransition(output.transitions[i], cycle_id, now_us))
        {
            degraded = true;
        }
    }
    for (size_t i = 0U; i < output.decision_count; ++i)
    {
        if (!bus_.pushDecision(output.decisions[i], cycle_id, now_us))
        {
            degraded = true;
        }
    }
    return degraded ? core::AppRunResult::DEGRADED : core::AppRunResult::OK;
}

} // namespace nura::apps
