#include "nura/apps/recovery_actuation_app.h"

#include "nura/config/runtime_profile.h"

namespace nura::apps
{
namespace c = nura::contracts;
namespace core = nura::core;
namespace p = nura::platform;

c::AppId RecoveryActuationApp::id() const
{
    return c::AppId::RECOVERY_ACTUATION;
}

core::AppRunResult RecoveryActuationApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    (void)now_ms;
    if (!initialized_)
    {
        if (output_.initialize() != p::PortResult::OK ||
            output_.allOff() != p::PortResult::OK)
        {
            return core::AppRunResult::FAILED;
        }
        initialized_ = true;
    }

    have_status_ = bus_.readFlightStatus(status_, id(), cycle_id,
                                         clock_.nowUs()) || have_status_;
    bool handled = false;
    bool degraded = false;
    for (uint8_t i = 0U;
         i < nura::config::kRuntimeProfile.recovery_drain_limit; ++i)
    {
        c::ActuationIntent intent{};
        if (!bus_.popActuation(intent, id(), cycle_id, clock_.nowUs()))
        {
            break;
        }
        handled = true;

        if (!authorized(intent))
        {
            traceResult(intent, cycle_id, clock_.nowUs(), -2);
            degraded = true;
            continue;
        }

        p::PortResult result = p::PortResult::FAULT;
        if (intent.operation == c::ActuationOperation::ALL_OFF)
        {
            result = output_.allOff();
        }
        else
        {
            result = output_.setChannel(intent.channel, intent.enabled);
        }
        traceResult(intent, cycle_id, clock_.nowUs(), static_cast<int32_t>(result));
        if (result != p::PortResult::OK)
        {
            (void)output_.allOff();
            return core::AppRunResult::FAILED;
        }
    }

    if (!handled)
    {
        return core::AppRunResult::NO_INPUT;
    }
    return degraded ? core::AppRunResult::DEGRADED : core::AppRunResult::OK;
}

bool RecoveryActuationApp::authorized(const c::ActuationIntent &intent) const
{
    if (intent.operation == c::ActuationOperation::ALL_OFF || !intent.enabled)
    {
        return true;
    }
    if (!have_status_ || status_.state != intent.authorized_state ||
        status_.transition_sequence != intent.transition_sequence)
    {
        return false;
    }
    if (intent.channel == c::RecoveryChannel::DROGUE_PRIMARY)
    {
        return status_.state == c::FlightState::APOGEE;
    }
    if (intent.channel == c::RecoveryChannel::MAIN_PRIMARY)
    {
        return status_.state == c::FlightState::DEPLOY;
    }
    return false;
}

void RecoveryActuationApp::traceResult(const c::ActuationIntent &intent,
                                       uint32_t cycle_id, uint64_t now_us,
                                       int32_t result)
{
    c::TraceRecord record{};
    record.cycle_id = cycle_id;
    record.correlation_id = intent.sequence;
    record.timestamp_us = now_us;
    record.app = id();
    record.peer = intent.source;
    record.topic = c::TopicId::ACTUATION_INTENT_QUEUE;
    record.event = c::TraceEvent::ACTUATION_RESULT;
    record.state = status_.state;
    record.result = result;
    record.detail = static_cast<uint32_t>(intent.channel) |
                    (intent.enabled ? (1U << 8U) : 0U);
    (void)trace_map_.tryRecord(record);
}

} // namespace nura::apps
