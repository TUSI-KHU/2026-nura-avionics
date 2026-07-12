#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "nura/contracts/flight_topics.h"
#include "nura/flight/state_app.h"

namespace nura::flight
{

class LaunchDetectorApp;
class BurnoutDetectorApp;
class ApogeeDetectorApp;
class DrogueSequenceApp;
class MainDeployDetectorApp;
class LandingSequenceApp;

struct CoordinatorOutput
{
    static constexpr size_t kMaxDecisions = 6U;
    static constexpr size_t kMaxActuationIntents = 5U;
    static constexpr size_t kMaxTransitions = 2U;

    nura::contracts::FlightStatus status{};
    nura::contracts::AppEnableSet app_enable{};
    std::array<nura::contracts::DecisionTrace, kMaxDecisions> decisions{};
    std::array<nura::contracts::ActuationIntent, kMaxActuationIntents> actuation{};
    std::array<nura::contracts::TransitionEvent, kMaxTransitions> transitions{};
    size_t decision_count = 0U;
    size_t actuation_count = 0U;
    size_t transition_count = 0U;
    bool output_overflow = false;
};

class FlightCoordinator
{
public:
    FlightCoordinator(LaunchDetectorApp &launch,
                      BurnoutDetectorApp &burnout,
                      ApogeeDetectorApp &apogee,
                      DrogueSequenceApp &drogue,
                      MainDeployDetectorApp &main_deploy,
                      LandingSequenceApp &landing,
                      IStateAppRunner *state_app_runner = nullptr);

    CoordinatorOutput initialize(const nura::contracts::FlightInputs &inputs,
                                 uint32_t now_ms, uint64_t publish_time_us);
    CoordinatorOutput step(const nura::contracts::FlightInputs &inputs,
                           const nura::contracts::CommandRequest *command,
                           uint32_t now_ms, uint64_t publish_time_us,
                           uint32_t cycle_id = 0U);
    const nura::contracts::FlightStatus &status() const { return status_; }
    void setStateForReplay(nura::contracts::FlightState state, uint32_t entered_ms);

private:
    IStateApp *stateApp(nura::contracts::FlightState state);
    void mergeStateOutput(StateAppOutput &state_output, CoordinatorOutput &output);
    void transitionTo(nura::contracts::FlightState next, uint32_t timestamp_ms,
                      nura::contracts::AppId requested_by,
                      const nura::contracts::FlightInputs &inputs,
                      CoordinatorOutput &output);
    void enterState(nura::contracts::FlightState state,
                    const nura::contracts::FlightInputs &inputs,
                    uint32_t now_ms, CoordinatorOutput &output);
    void appendAllOff(uint32_t now_ms, CoordinatorOutput &output);
    void finalizeOutput(uint32_t now_ms, uint64_t publish_time_us,
                        CoordinatorOutput &output);
    void processCommand(const nura::contracts::CommandRequest &command,
                        uint32_t now_ms, const nura::contracts::FlightInputs &inputs,
                        CoordinatorOutput &output, bool &transitioned);

    std::array<IStateApp *, 6U> state_apps_{};
    IStateAppRunner *state_app_runner_ = nullptr;
    nura::contracts::FlightStatus status_{};
    uint32_t status_sequence_ = 0U;
    uint32_t app_enable_sequence_ = 0U;
    uint32_t actuation_sequence_ = 0U;
    uint32_t current_cycle_id_ = 0U;
};

} // namespace nura::flight
