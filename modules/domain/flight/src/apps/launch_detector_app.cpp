#include "nura/flight/apps/launch_detector_app.h"

#include "nura/flight/flight_policy.h"

namespace nura::flight
{
namespace c = nura::contracts;

c::AppId LaunchDetectorApp::id() const { return c::AppId::LAUNCH_DETECTOR; }
c::FlightState LaunchDetectorApp::state() const { return c::FlightState::ARMED; }

StateAppOutput LaunchDetectorApp::onEnter(const c::FlightStatus &status,
                                          const c::FlightInputs &inputs,
                                          uint32_t now_ms)
{
    (void)status;
    (void)inputs;
    (void)now_ms;
    selector_.reset();
    confirmation_count_ = 0U;
    return {};
}

StateAppOutput LaunchDetectorApp::step(const c::FlightStatus &status,
                                       const c::FlightInputs &inputs,
                                       uint32_t now_ms)
{
    StateAppOutput output{};
    SelectedAcceleration sample{};
    if (!selector_.consume(inputs, now_ms, sample))
    {
        return output;
    }

    if (sample.norm_g >= FlightPolicy::kLaunchAccelThresholdG)
    {
        ++confirmation_count_;
    }
    else
    {
        confirmation_count_ = 0U;
    }

    uint16_t reason = sample.source == AccelerationSource::LOW_G
                          ? c::DECISION_REASON_PRIMARY_SENSOR
                          : c::DECISION_REASON_FALLBACK_SENSOR;
    reason = static_cast<uint16_t>(reason |
        (sample.norm_g >= FlightPolicy::kLaunchAccelThresholdG
             ? c::DECISION_REASON_THRESHOLD_MET
             : c::DECISION_REASON_THRESHOLD_NOT_MET));
    const bool accepted = confirmation_count_ >= FlightPolicy::kLaunchConfirmSamples;
    if (accepted)
    {
        reason = static_cast<uint16_t>(reason | c::DECISION_REASON_CONFIRMATION_MET);
    }

    (void)output.addDecision(makeDecision(
        status.state, c::DecisionKind::LAUNCH_ACCEL,
        accepted ? c::DecisionResult::ACCEPT : c::DecisionResult::OBSERVE,
        reason, sample.sample_time_ms, sample.norm_g,
        FlightPolicy::kLaunchAccelThresholdG, 0.0f, 0.0f,
        confirmation_count_, static_cast<uint8_t>(sample.source)));

    if (accepted)
    {
        output.requestTransition(c::FlightState::LAUNCH, sample.sample_time_ms, id());
    }
    return output;
}

} // namespace nura::flight
