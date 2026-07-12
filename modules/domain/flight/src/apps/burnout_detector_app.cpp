#include "nura/flight/apps/burnout_detector_app.h"

#include "nura/flight/flight_policy.h"

namespace nura::flight
{
namespace c = nura::contracts;

c::AppId BurnoutDetectorApp::id() const { return c::AppId::BURNOUT_DETECTOR; }
c::FlightState BurnoutDetectorApp::state() const { return c::FlightState::LAUNCH; }

StateAppOutput BurnoutDetectorApp::onEnter(const c::FlightStatus &status,
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

StateAppOutput BurnoutDetectorApp::step(const c::FlightStatus &status,
                                        const c::FlightInputs &inputs,
                                        uint32_t now_ms)
{
    StateAppOutput output{};
    SelectedAcceleration sample{};
    if (!selector_.consume(inputs, now_ms, sample))
    {
        return output;
    }

    if (sample.norm_g < FlightPolicy::kBurnoutAccelThresholdG)
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
        (sample.norm_g < FlightPolicy::kBurnoutAccelThresholdG
             ? c::DECISION_REASON_THRESHOLD_MET
             : c::DECISION_REASON_THRESHOLD_NOT_MET));
    const bool accepted = confirmation_count_ >= FlightPolicy::kBurnoutConfirmSamples;
    if (accepted)
    {
        reason = static_cast<uint16_t>(reason | c::DECISION_REASON_CONFIRMATION_MET);
    }

    (void)output.addDecision(makeDecision(
        status.state, c::DecisionKind::BURNOUT_ACCEL,
        accepted ? c::DecisionResult::ACCEPT : c::DecisionResult::OBSERVE,
        reason, sample.sample_time_ms, sample.norm_g,
        FlightPolicy::kBurnoutAccelThresholdG, 0.0f, 0.0f,
        confirmation_count_, static_cast<uint8_t>(sample.source)));

    if (accepted)
    {
        output.requestTransition(c::FlightState::COAST, sample.sample_time_ms, id());
    }
    return output;
}

} // namespace nura::flight
