#include "nura/flight/flight_coordinator.h"

#include "nura/config/runtime_profile.h"
#include "nura/flight/app_enable_policy.h"
#include "nura/flight/apps/apogee_detector_app.h"
#include "nura/flight/apps/burnout_detector_app.h"
#include "nura/flight/apps/drogue_sequence_app.h"
#include "nura/flight/apps/landing_sequence_app.h"
#include "nura/flight/apps/launch_detector_app.h"
#include "nura/flight/apps/main_deploy_detector_app.h"

namespace nura::flight
{
namespace c = nura::contracts;

FlightCoordinator::FlightCoordinator(LaunchDetectorApp &launch,
                                     BurnoutDetectorApp &burnout,
                                     ApogeeDetectorApp &apogee,
                                     DrogueSequenceApp &drogue,
                                     MainDeployDetectorApp &main_deploy,
                                     LandingSequenceApp &landing,
                                     IStateAppRunner *state_app_runner)
    : state_apps_{{&launch, &burnout, &apogee, &drogue, &main_deploy, &landing}},
      state_app_runner_(state_app_runner)
{
}

CoordinatorOutput FlightCoordinator::initialize(const c::FlightInputs &inputs,
                                                 uint32_t now_ms,
                                                 uint64_t publish_time_us)
{
    (void)inputs;
    status_ = {};
    status_.state = c::FlightState::INIT;
    status_.state_entered_ms = now_ms;
    status_sequence_ = 0U;
    app_enable_sequence_ = 0U;
    actuation_sequence_ = 0U;

    CoordinatorOutput output{};
    appendAllOff(now_ms, output);
    finalizeOutput(now_ms, publish_time_us, output);
    return output;
}

CoordinatorOutput FlightCoordinator::step(const c::FlightInputs &inputs,
                                           const c::CommandRequest *command,
                                           uint32_t now_ms,
                                           uint64_t publish_time_us,
                                           uint32_t cycle_id)
{
    current_cycle_id_ = cycle_id;
    CoordinatorOutput output{};
    if (inputs.abort_active && status_.state != c::FlightState::SAFE)
    {
        transitionTo(c::FlightState::SAFE, now_ms, c::AppId::FLIGHT_COORDINATOR,
                     inputs, output);
        finalizeOutput(now_ms, publish_time_us, output);
        return output;
    }

    bool transitioned = false;
    if (command != nullptr)
    {
        processCommand(*command, now_ms, inputs, output, transitioned);
        if (transitioned)
        {
            finalizeOutput(now_ms, publish_time_us, output);
            return output;
        }
    }

    switch (status_.state)
    {
    case c::FlightState::INIT:
        transitionTo(c::FlightState::SAFE, now_ms, c::AppId::FLIGHT_COORDINATOR,
                     inputs, output);
        break;
    case c::FlightState::SAFE:
        if (!inputs.abort_active)
        {
            transitionTo(c::FlightState::ARMED, now_ms,
                         c::AppId::FLIGHT_COORDINATOR, inputs, output);
        }
        break;
    case c::FlightState::ARMED:
    case c::FlightState::LAUNCH:
    case c::FlightState::COAST:
    case c::FlightState::APOGEE:
    case c::FlightState::DROGUE:
    case c::FlightState::DEPLOY:
    {
        IStateApp *app = stateApp(status_.state);
        if (app == nullptr)
        {
            transitionTo(c::FlightState::FAULT, now_ms,
                         c::AppId::FLIGHT_COORDINATOR, inputs, output);
            break;
        }

        StateAppOutput state_output = state_app_runner_ != nullptr
                                          ? state_app_runner_->step(
                                                *app, status_, inputs, now_ms,
                                                current_cycle_id_)
                                          : app->step(status_, inputs, now_ms);
        const TransitionRequest transition = state_output.transition;
        mergeStateOutput(state_output, output);
        if (transition.requested)
        {
            transitionTo(transition.next, transition.timestamp_ms,
                         transition.requested_by, inputs, output);
        }
        break;
    }
    case c::FlightState::GROUND:
    case c::FlightState::FAULT:
        break;
    default:
        transitionTo(c::FlightState::FAULT, now_ms,
                     c::AppId::FLIGHT_COORDINATOR, inputs, output);
        break;
    }

    finalizeOutput(now_ms, publish_time_us, output);
    return output;
}

IStateApp *FlightCoordinator::stateApp(c::FlightState state)
{
    for (IStateApp *app : state_apps_)
    {
        if (app != nullptr && app->state() == state &&
            nura::config::isApplicationEnabled(app->id(), state))
        {
            return app;
        }
    }
    return nullptr;
}

void FlightCoordinator::mergeStateOutput(StateAppOutput &state_output,
                                         CoordinatorOutput &output)
{
    if (state_output.latch_barometer_stuck_fault)
    {
        status_.barometer_stuck_fault_latched = true;
    }
    if (state_output.drogue_sequence_complete)
    {
        status_.drogue_sequence_complete = true;
    }
    if (state_output.main_sequence_complete)
    {
        status_.main_sequence_complete = true;
    }

    for (size_t i = 0U; i < state_output.decision_count; ++i)
    {
        if (output.decision_count >= output.decisions.size())
        {
            output.output_overflow = true;
            break;
        }
        c::DecisionTrace decision = state_output.decisions[i];
        decision.sequence = ++status_.decision_sequence;
        output.decisions[output.decision_count++] = decision;
    }

    for (size_t i = 0U; i < state_output.actuation_count; ++i)
    {
        if (output.actuation_count >= output.actuation.size())
        {
            output.output_overflow = true;
            break;
        }
        c::ActuationIntent intent = state_output.actuation[i];
        intent.sequence = ++actuation_sequence_;
        intent.transition_sequence = status_.transition_sequence;
        output.actuation[output.actuation_count++] = intent;
    }
}

void FlightCoordinator::transitionTo(c::FlightState next, uint32_t timestamp_ms,
                                     c::AppId requested_by,
                                     const c::FlightInputs &inputs,
                                     CoordinatorOutput &output)
{
    if (status_.state == next)
    {
        return;
    }

    const c::FlightState previous = status_.state;
    status_.state = next;
    status_.state_entered_ms = timestamp_ms;
    ++status_.transition_sequence;

    if (output.transition_count < output.transitions.size())
    {
        c::TransitionEvent event{};
        event.sequence = status_.transition_sequence;
        event.timestamp_ms = timestamp_ms;
        event.previous = previous;
        event.current = next;
        event.requested_by = requested_by;
        output.transitions[output.transition_count++] = event;
    }
    else
    {
        output.output_overflow = true;
    }

    enterState(next, inputs, timestamp_ms, output);
}

void FlightCoordinator::enterState(c::FlightState state,
                                   const c::FlightInputs &inputs,
                                   uint32_t now_ms,
                                   CoordinatorOutput &output)
{
    switch (state)
    {
    case c::FlightState::LAUNCH:
        status_.launch_ms = now_ms;
        break;
    case c::FlightState::COAST:
        status_.coast_ms = now_ms;
        break;
    case c::FlightState::APOGEE:
        status_.apogee_ms = now_ms;
        status_.drogue_sequence_complete = false;
        status_.recovery_deploy_started = true;
        break;
    case c::FlightState::DROGUE:
        status_.drogue_ms = now_ms;
        break;
    case c::FlightState::DEPLOY:
        status_.deploy_ms = now_ms;
        status_.main_sequence_complete = false;
        break;
    case c::FlightState::SAFE:
    case c::FlightState::GROUND:
    case c::FlightState::FAULT:
        appendAllOff(now_ms, output);
        break;
    case c::FlightState::INIT:
    default:
        break;
    }

    IStateApp *app = stateApp(state);
    if (app != nullptr)
    {
        StateAppOutput enter_output = state_app_runner_ != nullptr
                                          ? state_app_runner_->enter(
                                                *app, status_, inputs, now_ms,
                                                current_cycle_id_)
                                          : app->onEnter(status_, inputs, now_ms);
        mergeStateOutput(enter_output, output);
    }
}

void FlightCoordinator::appendAllOff(uint32_t now_ms, CoordinatorOutput &output)
{
    if (output.actuation_count >= output.actuation.size())
    {
        output.output_overflow = true;
        return;
    }
    c::ActuationIntent intent{};
    intent.sequence = ++actuation_sequence_;
    intent.timestamp_ms = now_ms;
    intent.transition_sequence = status_.transition_sequence;
    intent.operation = c::ActuationOperation::ALL_OFF;
    intent.authorized_state = status_.state;
    intent.source = c::AppId::FLIGHT_COORDINATOR;
    output.actuation[output.actuation_count++] = intent;
}

void FlightCoordinator::finalizeOutput(uint32_t now_ms, uint64_t publish_time_us,
                                       CoordinatorOutput &output)
{
    status_.header.schema_version = c::kContractSchemaVersion;
    status_.header.sequence = ++status_sequence_;
    status_.header.sample_time_ms = now_ms;
    status_.header.publish_time_us = publish_time_us;
    status_.header.status_flags = c::SAMPLE_STATUS_VALID;
    status_.header.producer = c::AppId::FLIGHT_COORDINATOR;
    output.status = status_;

    output.app_enable.header.schema_version = c::kContractSchemaVersion;
    output.app_enable.header.sequence = ++app_enable_sequence_;
    output.app_enable.header.sample_time_ms = now_ms;
    output.app_enable.header.publish_time_us = publish_time_us;
    output.app_enable.header.status_flags = c::SAMPLE_STATUS_VALID;
    output.app_enable.header.producer = c::AppId::FLIGHT_COORDINATOR;
    output.app_enable.state = status_.state;
    output.app_enable.enabled_mask = enabledApplications(status_.state);
}

void FlightCoordinator::processCommand(const c::CommandRequest &command,
                                       uint32_t now_ms,
                                       const c::FlightInputs &inputs,
                                       CoordinatorOutput &output,
                                       bool &transitioned)
{
    transitioned = false;
    if (command.type != c::CommandType::FORCE_RECOVERY_DEPLOY)
    {
        return;
    }

    StateAppOutput command_output{};
    const bool allowed = c::stateAllowsForceRecoveryDeploy(status_.state);
    (void)command_output.addDecision(makeDecision(
        status_.state, c::DecisionKind::FORCE_DEPLOY,
        allowed ? c::DecisionResult::ACCEPT : c::DecisionResult::REJECT,
        c::DECISION_REASON_FORCED, now_ms,
        static_cast<float>(command.command_sequence),
        static_cast<float>(status_.state_entered_ms), 0.0f, 0.0f, 0U, 0U));
    mergeStateOutput(command_output, output);
    if (!allowed)
    {
        return;
    }

    status_.force_recovery_executed = true;
    status_.force_recovery_executed_sequence = command.command_sequence;
    transitionTo(c::FlightState::APOGEE, now_ms, command.source, inputs, output);
    transitioned = true;
}

void FlightCoordinator::setStateForReplay(c::FlightState state,
                                          uint32_t entered_ms)
{
    status_.state = state;
    status_.state_entered_ms = entered_ms;
    switch (state)
    {
    case c::FlightState::LAUNCH: status_.launch_ms = entered_ms; break;
    case c::FlightState::COAST: status_.coast_ms = entered_ms; break;
    case c::FlightState::APOGEE: status_.apogee_ms = entered_ms; break;
    case c::FlightState::DROGUE: status_.drogue_ms = entered_ms; break;
    case c::FlightState::DEPLOY: status_.deploy_ms = entered_ms; break;
    default: break;
    }
    if (IStateApp *app = stateApp(state); app != nullptr)
    {
        const c::FlightInputs empty_inputs{};
        (void)app->onEnter(status_, empty_inputs, entered_ms);
    }
}

} // namespace nura::flight
