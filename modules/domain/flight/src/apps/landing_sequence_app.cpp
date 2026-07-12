#include "nura/flight/apps/landing_sequence_app.h"

#include <cmath>

namespace nura::flight
{
namespace c = nura::contracts;

namespace
{
c::ActuationIntent mainIntent(bool enabled, uint32_t now_ms, c::AppId source)
{
    c::ActuationIntent intent{};
    intent.timestamp_ms = now_ms;
    intent.operation = c::ActuationOperation::SET_CHANNEL;
    intent.channel = c::RecoveryChannel::MAIN_PRIMARY;
    intent.enabled = enabled;
    intent.authorized_state = c::FlightState::DEPLOY;
    intent.source = source;
    return intent;
}

bool barometerValid(const c::BarometerSample &barometer)
{
    const uint32_t required = c::SAMPLE_STATUS_VALID |
                              c::SAMPLE_STATUS_REFERENCE_VALID;
    return (barometer.header.status_flags & required) == required &&
           (barometer.header.status_flags & c::SAMPLE_STATUS_FAULT) == 0U &&
           barometer.fault_flags == c::BAROMETER_FAULT_NONE &&
           barometer.header.sample_time_ms != 0U &&
           std::isfinite(barometer.filtered_altitude_m);
}
} // namespace

c::AppId LandingSequenceApp::id() const { return c::AppId::LANDING_SEQUENCE; }
c::FlightState LandingSequenceApp::state() const { return c::FlightState::DEPLOY; }

StateAppOutput LandingSequenceApp::onEnter(const c::FlightStatus &status,
                                           const c::FlightInputs &inputs,
                                           uint32_t now_ms)
{
    (void)status;
    (void)inputs;
    main_off_ = false;
    resetSamples();

    StateAppOutput output{};
    (void)output.addActuation(mainIntent(true, now_ms, id()));
    return output;
}

StateAppOutput LandingSequenceApp::step(const c::FlightStatus &status,
                                        const c::FlightInputs &inputs,
                                        uint32_t now_ms)
{
    StateAppOutput output{};
    const uint32_t elapsed_ms = now_ms - status.deploy_ms;
    if (!main_off_ && elapsed_ms >= timing_.pyro_fire_duration_ms)
    {
        (void)output.addActuation(mainIntent(false, now_ms, id()));
        main_off_ = true;
        output.main_sequence_complete = true;
    }

    if (!main_off_ || !consumeBarometer(inputs.barometer))
    {
        return output;
    }

    const bool is_stable = stable();
    (void)output.addDecision(makeDecision(
        status.state, c::DecisionKind::LANDING,
        is_stable ? c::DecisionResult::ACCEPT : c::DecisionResult::OBSERVE,
        is_stable ? c::DECISION_REASON_CONFIRMATION_MET
                  : c::DECISION_REASON_THRESHOLD_NOT_MET,
        inputs.barometer.header.sample_time_ms,
        inputs.barometer.filtered_altitude_m,
        FlightPolicy::kLandingStableAltitudeRangeM,
        static_cast<float>(count_),
        static_cast<float>(FlightPolicy::kLandingStableWindowSamples),
        count_, 0U));
    if (is_stable)
    {
        output.requestTransition(c::FlightState::GROUND,
                                 inputs.barometer.header.sample_time_ms, id());
    }
    return output;
}

bool LandingSequenceApp::consumeBarometer(const c::BarometerSample &barometer)
{
    if (!barometerValid(barometer) ||
        barometer.header.sample_time_ms == last_barometer_time_ms_)
    {
        return false;
    }

    if (last_barometer_time_ms_ != 0U &&
        (barometer.header.sample_time_ms - last_barometer_time_ms_) >
            FlightPolicy::kLandingMaxBarometerSampleGapMs)
    {
        resetSamples();
    }
    last_barometer_time_ms_ = barometer.header.sample_time_ms;
    pushSample(barometer.header.sample_time_ms,
               barometer.filtered_altitude_m);
    return true;
}

void LandingSequenceApp::pushSample(uint32_t sample_time_ms, float altitude_m)
{
    samples_[head_] = {sample_time_ms, altitude_m};
    head_ = static_cast<uint8_t>((head_ + 1U) % samples_.size());
    if (count_ < samples_.size())
    {
        ++count_;
    }
}

bool LandingSequenceApp::stable() const
{
    if (count_ < samples_.size())
    {
        return false;
    }

    float minimum = samples_[0].altitude_m;
    float maximum = samples_[0].altitude_m;
    for (size_t i = 1U; i < samples_.size(); ++i)
    {
        if (samples_[i].altitude_m < minimum)
        {
            minimum = samples_[i].altitude_m;
        }
        if (samples_[i].altitude_m > maximum)
        {
            maximum = samples_[i].altitude_m;
        }
    }
    return (maximum - minimum) <= FlightPolicy::kLandingStableAltitudeRangeM;
}

void LandingSequenceApp::resetSamples()
{
    head_ = 0U;
    count_ = 0U;
    last_barometer_time_ms_ = 0U;
}

} // namespace nura::flight
