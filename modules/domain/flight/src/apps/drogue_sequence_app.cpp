#include "nura/flight/apps/drogue_sequence_app.h"

#include "nura/flight/flight_policy.h"

namespace nura::flight
{
namespace c = nura::contracts;

namespace
{
c::ActuationIntent drogueIntent(bool enabled, uint32_t now_ms, c::AppId source)
{
    c::ActuationIntent intent{};
    intent.timestamp_ms = now_ms;
    intent.operation = c::ActuationOperation::SET_CHANNEL;
    intent.channel = c::RecoveryChannel::DROGUE_PRIMARY;
    intent.enabled = enabled;
    intent.authorized_state = c::FlightState::APOGEE;
    intent.source = source;
    return intent;
}
} // namespace

c::AppId DrogueSequenceApp::id() const { return c::AppId::DROGUE_SEQUENCE; }
c::FlightState DrogueSequenceApp::state() const { return c::FlightState::APOGEE; }

StateAppOutput DrogueSequenceApp::onEnter(const c::FlightStatus &status,
                                          const c::FlightInputs &inputs,
                                          uint32_t now_ms)
{
    (void)status;
    (void)inputs;
    primary_off_ = false;
    backup_on_ = false;
    backup_off_ = false;

    StateAppOutput output{};
    (void)output.addActuation(drogueIntent(true, now_ms, id()));
    return output;
}

StateAppOutput DrogueSequenceApp::step(const c::FlightStatus &status,
                                       const c::FlightInputs &inputs,
                                       uint32_t now_ms)
{
    (void)inputs;
    StateAppOutput output{};
    const uint32_t elapsed_ms = now_ms - status.apogee_ms;

    if (!primary_off_ && elapsed_ms >= timing_.pyro_fire_duration_ms)
    {
        (void)output.addActuation(drogueIntent(false, now_ms, id()));
        primary_off_ = true;
    }

    if (!backup_on_ && elapsed_ms >= timing_.drogue_backup_delay_ms)
    {
        (void)output.addActuation(drogueIntent(true, now_ms, id()));
        backup_on_ = true;
    }

    if (backup_on_ && !backup_off_ &&
        elapsed_ms >= timing_.drogue_backup_delay_ms +
                          timing_.pyro_fire_duration_ms)
    {
        (void)output.addActuation(drogueIntent(false, now_ms, id()));
        backup_off_ = true;
        output.drogue_sequence_complete = true;
        output.requestTransition(c::FlightState::DROGUE, now_ms, id());
    }
    return output;
}

} // namespace nura::flight
