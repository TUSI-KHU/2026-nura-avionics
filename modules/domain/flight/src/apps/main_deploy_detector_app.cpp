#include "nura/flight/apps/main_deploy_detector_app.h"

#include <cmath>

#include "nura/flight/flight_policy.h"

namespace nura::flight
{
namespace c = nura::contracts;

namespace
{
bool faulted(const c::BarometerSample &barometer)
{
    return (barometer.header.status_flags & c::SAMPLE_STATUS_FAULT) != 0U ||
           barometer.fault_flags != c::BAROMETER_FAULT_NONE;
}
} // namespace

c::AppId MainDeployDetectorApp::id() const
{
    return c::AppId::MAIN_DEPLOY_DETECTOR;
}

c::FlightState MainDeployDetectorApp::state() const
{
    return c::FlightState::DROGUE;
}

StateAppOutput MainDeployDetectorApp::onEnter(const c::FlightStatus &status,
                                              const c::FlightInputs &inputs,
                                              uint32_t now_ms)
{
    (void)status;
    (void)inputs;
    (void)now_ms;
    last_barometer_time_ms_ = 0U;
    resetStuckMonitor();
    return {};
}

StateAppOutput MainDeployDetectorApp::step(const c::FlightStatus &status,
                                           const c::FlightInputs &inputs,
                                           uint32_t now_ms)
{
    StateAppOutput output{};
    const c::BarometerSample &barometer = inputs.barometer;
    bool usable = !status.barometer_stuck_fault_latched &&
                  barometerUsable(barometer, now_ms);
    if (usable && barometer.header.sample_time_ms != last_barometer_time_ms_)
    {
        last_barometer_time_ms_ = barometer.header.sample_time_ms;
        trackStuck(barometer.header.sample_time_ms,
                   barometer.filtered_altitude_m, output);
        if (output.latch_barometer_stuck_fault)
        {
            usable = false;
        }
    }

    const uint32_t elapsed_ms = now_ms - status.drogue_ms;
    const bool altitude_reached =
        usable && barometer.filtered_altitude_m <= FlightPolicy::kMainDeployAltitudeM;
    const bool timeout_reached = elapsed_ms >= FlightPolicy::kMainTimeoutMs;
    (void)output.addDecision(makeDecision(
        status.state, c::DecisionKind::MAIN_DEPLOY,
        altitude_reached || timeout_reached ? c::DecisionResult::ACCEPT
                                            : c::DecisionResult::OBSERVE,
        altitude_reached
            ? c::DECISION_REASON_THRESHOLD_MET
            : (timeout_reached ? c::DECISION_REASON_TIMEOUT
                               : c::DECISION_REASON_THRESHOLD_NOT_MET),
        now_ms, barometer.filtered_altitude_m,
        FlightPolicy::kMainDeployAltitudeM, static_cast<float>(elapsed_ms),
        static_cast<float>(FlightPolicy::kMainTimeoutMs), 0U, 0U));

    if (altitude_reached || timeout_reached)
    {
        output.requestTransition(c::FlightState::DEPLOY, now_ms, id());
    }
    return output;
}

bool MainDeployDetectorApp::barometerUsable(
    const c::BarometerSample &barometer, uint32_t now_ms) const
{
    const uint32_t required = c::SAMPLE_STATUS_VALID |
                              c::SAMPLE_STATUS_REFERENCE_VALID;
    return (barometer.header.status_flags & required) == required &&
           !faulted(barometer) && barometer.header.sample_time_ms != 0U &&
           (now_ms - barometer.header.sample_time_ms) <=
               FlightPolicy::kApogeeMaxBarometerSampleGapMs;
}

void MainDeployDetectorApp::resetStuckMonitor()
{
    stuck_window_active_ = false;
    stuck_window_start_ms_ = 0U;
    last_stuck_sample_ms_ = 0U;
    stuck_min_altitude_m_ = 0.0f;
    stuck_max_altitude_m_ = 0.0f;
}

void MainDeployDetectorApp::trackStuck(uint32_t sample_time_ms, float altitude_m,
                                       StateAppOutput &output)
{
    if (!std::isfinite(altitude_m))
    {
        resetStuckMonitor();
        return;
    }

    if (!stuck_window_active_ ||
        (last_stuck_sample_ms_ != 0U &&
         (sample_time_ms - last_stuck_sample_ms_) >
             FlightPolicy::kApogeeMaxBarometerSampleGapMs))
    {
        stuck_window_active_ = true;
        stuck_window_start_ms_ = sample_time_ms;
        last_stuck_sample_ms_ = sample_time_ms;
        stuck_min_altitude_m_ = altitude_m;
        stuck_max_altitude_m_ = altitude_m;
        return;
    }

    if (altitude_m < stuck_min_altitude_m_)
    {
        stuck_min_altitude_m_ = altitude_m;
    }
    if (altitude_m > stuck_max_altitude_m_)
    {
        stuck_max_altitude_m_ = altitude_m;
    }
    last_stuck_sample_ms_ = sample_time_ms;

    if ((sample_time_ms - stuck_window_start_ms_) >=
        FlightPolicy::kBarometerStuckWindowMs)
    {
        if ((stuck_max_altitude_m_ - stuck_min_altitude_m_) <=
            FlightPolicy::kBarometerStuckRangeM)
        {
            output.latch_barometer_stuck_fault = true;
            return;
        }
        stuck_window_start_ms_ = sample_time_ms;
        stuck_min_altitude_m_ = altitude_m;
        stuck_max_altitude_m_ = altitude_m;
    }
}

} // namespace nura::flight
