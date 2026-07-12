#include "nura/apps/event_recorder_app.h"

#include "nura/config/runtime_profile.h"

namespace nura::apps
{
namespace c = nura::contracts;
namespace core = nura::core;
namespace p = nura::platform;

c::AppId EventRecorderApp::id() const { return c::AppId::EVENT_RECORDER; }

core::AppRunResult EventRecorderApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    (void)now_ms;
    bool handled = false;
    bool degraded = false;
    for (uint8_t i = 0U;
         i < nura::config::kRuntimeProfile.event_drain_limit; ++i)
    {
        bool consumed = false;
        c::TransitionEvent transition{};
        if (bus_.popTransition(transition, id(), cycle_id, clock_.nowUs()))
        {
            consumed = true;
            handled = true;
            degraded = sink_.tryWrite(transition) != p::PortResult::OK || degraded;
        }

        c::DecisionTrace decision{};
        if (bus_.popDecision(decision, id(), cycle_id, clock_.nowUs()))
        {
            consumed = true;
            handled = true;
            degraded = sink_.tryWrite(decision) != p::PortResult::OK || degraded;
        }

        c::SensorFaultEvent fault{};
        if (bus_.popSensorFault(fault, id(), cycle_id, clock_.nowUs()))
        {
            consumed = true;
            handled = true;
            degraded = sink_.tryWrite(fault) != p::PortResult::OK || degraded;
        }
        if (!consumed) break;
    }
    if (!handled) return core::AppRunResult::NO_INPUT;
    return degraded ? core::AppRunResult::DEGRADED : core::AppRunResult::OK;
}

} // namespace nura::apps
