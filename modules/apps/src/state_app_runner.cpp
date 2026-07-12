#include "nura/apps/state_app_runner.h"

#include <climits>

namespace nura::apps
{
namespace c = nura::contracts;
namespace f = nura::flight;

f::StateAppOutput TracingStateAppRunner::enter(
    f::IStateApp &app, const c::FlightStatus &status,
    const c::FlightInputs &inputs, uint32_t now_ms, uint32_t cycle_id)
{
    return run(app, status, inputs, now_ms, cycle_id, true);
}

f::StateAppOutput TracingStateAppRunner::step(
    f::IStateApp &app, const c::FlightStatus &status,
    const c::FlightInputs &inputs, uint32_t now_ms, uint32_t cycle_id)
{
    return run(app, status, inputs, now_ms, cycle_id, false);
}

f::StateAppOutput TracingStateAppRunner::run(
    f::IStateApp &app, const c::FlightStatus &status,
    const c::FlightInputs &inputs, uint32_t now_ms, uint32_t cycle_id,
    bool entering)
{
    const uint64_t started_us = clock_.nowUs();
    record(c::TraceEvent::STATE_APP_BEGIN, app.id(), status.state, cycle_id,
           started_us, 0U, entering ? 1U : 0U);
    f::StateAppOutput output = entering ? app.onEnter(status, inputs, now_ms)
                                        : app.step(status, inputs, now_ms);
    const uint64_t completed_us = clock_.nowUs();
    const uint64_t elapsed = completed_us >= started_us
                                 ? completed_us - started_us
                                 : 0U;
    const uint32_t duration = elapsed > UINT32_MAX
                                  ? UINT32_MAX
                                  : static_cast<uint32_t>(elapsed);
    record(c::TraceEvent::STATE_APP_END, app.id(), status.state, cycle_id,
           completed_us, duration, entering ? 1U : 0U);

    if (output.transition.requested)
    {
        record(c::TraceEvent::TRANSITION_REQUEST,
               output.transition.requested_by, status.state, cycle_id,
               completed_us, 0U,
               static_cast<uint32_t>(output.transition.next));
    }
    return output;
}

void TracingStateAppRunner::record(c::TraceEvent event, c::AppId app,
                                   c::FlightState state, uint32_t cycle_id,
                                   uint64_t timestamp_us,
                                   uint32_t duration_us, uint32_t detail)
{
    c::TraceRecord record{};
    record.cycle_id = cycle_id;
    record.correlation_id = cycle_id;
    record.timestamp_us = timestamp_us;
    record.duration_us = duration_us;
    record.app = app;
    record.peer = c::AppId::FLIGHT_COORDINATOR;
    record.event = event;
    record.state = state;
    record.detail = detail;
    (void)trace_map_.tryRecord(record);
}

} // namespace nura::apps
