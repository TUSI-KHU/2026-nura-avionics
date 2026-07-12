#include <cstdio>

#include "nura/core/spsc_queue.h"
#include "nura/core/software_bus.h"
#include "nura/core/trace_map.h"
#include "nura/apps/recovery_actuation_app.h"
#include "nura/apps/flight_coordinator_app.h"
#include "nura/apps/state_app_runner.h"
#include "nura/config/runtime_profile.h"
#include "nura/flight/app_enable_policy.h"
#include "nura/flight/apps/apogee_detector_app.h"
#include "nura/flight/apps/burnout_detector_app.h"
#include "nura/flight/apps/drogue_sequence_app.h"
#include "nura/flight/apps/landing_sequence_app.h"
#include "nura/flight/apps/launch_detector_app.h"
#include "nura/flight/apps/main_deploy_detector_app.h"
#include "nura/flight/flight_coordinator.h"
#include "nura/platform/host/fake_platform.h"

namespace c = nura::contracts;
namespace core = nura::core;

namespace
{
int failures = 0;

class CountingApp final : public core::IExecutableApp
{
public:
    c::AppId id() const override { return c::AppId::TELEMETRY; }
    core::AppRunResult run(uint32_t, uint32_t) override
    {
        ++runs;
        return core::AppRunResult::OK;
    }
    uint32_t runs = 0U;
};

class TransitioningStateApp final : public nura::flight::IStateApp
{
public:
    c::AppId id() const override { return c::AppId::LAUNCH_DETECTOR; }
    c::FlightState state() const override { return c::FlightState::ARMED; }
    nura::flight::StateAppOutput onEnter(
        const c::FlightStatus &, const c::FlightInputs &, uint32_t) override
    {
        return {};
    }
    nura::flight::StateAppOutput step(
        const c::FlightStatus &, const c::FlightInputs &, uint32_t now_ms) override
    {
        nura::flight::StateAppOutput output{};
        output.requestTransition(c::FlightState::LAUNCH, now_ms, id());
        return output;
    }
};

void check(bool condition, const char *name)
{
    if (condition)
    {
        std::printf("PASS %s\n", name);
    }
    else
    {
        std::printf("FAIL %s\n", name);
        ++failures;
    }
}

void queueTest()
{
    core::SpscQueue<uint32_t, 4U> queue;
    check(queue.tryPush(1U) && queue.tryPush(2U) && queue.tryPush(3U),
          "spsc bounded fill");
    check(!queue.tryPush(4U) && queue.dropped() == 1U,
          "spsc explicit overflow");
    uint32_t value = 0U;
    check(queue.tryPop(value) && value == 1U && queue.tryPop(value) && value == 2U,
          "spsc preserves order");
}

void traceGapTest()
{
    core::TraceMap<4U> trace;
    for (uint32_t i = 0U; i < 6U; ++i)
    {
        c::TraceRecord record{};
        record.cycle_id = i;
        (void)trace.tryRecord(record);
    }
    c::TraceRecord output[4]{};
    uint32_t gap = 0U;
    const size_t count = trace.tryReadAfter(1U, output, 4U, gap);
    check(count == 4U && gap == 1U && trace.overwritten() == 2U,
          "tracemap reports overwrite gap");
}

void appPolicyTest()
{
    const uint64_t coast = nura::flight::enabledApplications(c::FlightState::COAST);
    const uint64_t armed = nura::flight::enabledApplications(c::FlightState::ARMED);
    check((coast & c::appBit(c::AppId::APOGEE_DETECTOR)) != 0U &&
              (coast & c::appBit(c::AppId::LAUNCH_DETECTOR)) == 0U,
          "coast enables only apogee state app");
    check((armed & c::appBit(c::AppId::LAUNCH_DETECTOR)) != 0U,
          "armed enables launch detector");
    check((coast & c::appBit(c::AppId::TELEMETRY)) == 0U &&
              (coast & c::appBit(c::AppId::FLIGHT_RECORDER)) == 0U &&
              (coast & c::appBit(c::AppId::ANNUNCIATION)) == 0U,
          "unimplemented applications are never advertised as enabled");

    const c::AppId expected_state_apps[] = {
        c::AppId::UNKNOWN,
        c::AppId::UNKNOWN,
        c::AppId::LAUNCH_DETECTOR,
        c::AppId::BURNOUT_DETECTOR,
        c::AppId::APOGEE_DETECTOR,
        c::AppId::DROGUE_SEQUENCE,
        c::AppId::MAIN_DEPLOY_DETECTOR,
        c::AppId::LANDING_SEQUENCE,
        c::AppId::UNKNOWN,
        c::AppId::UNKNOWN,
    };
    bool all_states_match = true;
    for (uint8_t raw_state = 0U; raw_state < 10U; ++raw_state)
    {
        const auto state = static_cast<c::FlightState>(raw_state);
        const uint64_t mask = nura::flight::enabledApplications(state);
        for (c::AppId candidate : {
                 c::AppId::LAUNCH_DETECTOR,
                 c::AppId::BURNOUT_DETECTOR,
                 c::AppId::APOGEE_DETECTOR,
                 c::AppId::DROGUE_SEQUENCE,
                 c::AppId::MAIN_DEPLOY_DETECTOR,
                 c::AppId::LANDING_SEQUENCE})
        {
            const bool enabled = (mask & c::appBit(candidate)) != 0U;
            all_states_match = all_states_match &&
                               (enabled ==
                                (candidate == expected_state_apps[raw_state]));
        }
    }
    check(all_states_match, "all FSM states have an exact state-app mapping");

    bool sensors_are_isolated = true;
    for (c::AppId sensor : {
             c::AppId::LOW_G_SENSOR, c::AppId::HIGH_G_SENSOR,
             c::AppId::BAROMETER_SENSOR, c::AppId::MAGNETOMETER_SENSOR,
             c::AppId::GNSS_SENSOR, c::AppId::POWER_SENSOR,
             c::AppId::SAFETY_INPUT})
    {
        const auto *descriptor = nura::config::appDescriptor(sensor);
        sensors_are_isolated = sensors_are_isolated && descriptor != nullptr &&
                               descriptor->independently_scheduled &&
                               descriptor->stack_bytes > 0U;
    }
    check(sensors_are_isolated,
          "every sensor owns an independent bounded execution domain");
}

void executionGateTest()
{
    nura::platform::host::ManualClock clock;
    core::SystemTraceMap trace;
    core::Executive executive(clock, trace);
    CountingApp app;
    const auto result = executive.runIfEnabled(
        app, 0U, 1U, 100U, c::FlightState::SAFE,
        nura::config::enabledApplicationMask(c::FlightState::SAFE));
    c::TraceRecord records[4]{};
    uint32_t gap = 0U;
    const size_t count = trace.tryReadAfter(0U, records, 4U, gap);
    check(result == core::AppRunResult::NO_INPUT && app.runs == 0U &&
              count == 1U &&
              records[0].event == c::TraceEvent::TASK_SKIPPED,
          "executive enforces catalog enable mask");
}

void exactStateTraceTest()
{
    nura::platform::host::ManualClock clock;
    core::SystemTraceMap trace;
    nura::apps::TracingStateAppRunner runner(clock, trace);
    TransitioningStateApp app;
    c::FlightStatus status{};
    status.state = c::FlightState::ARMED;
    c::FlightInputs inputs{};
    const auto output = runner.step(app, status, inputs, 100U, 7U);
    c::TraceRecord records[8]{};
    uint32_t gap = 0U;
    const size_t count = trace.tryReadAfter(0U, records, 8U, gap);
    check(output.transition.requested && count == 3U &&
              records[0].event == c::TraceEvent::STATE_APP_BEGIN &&
              records[1].event == c::TraceEvent::STATE_APP_END &&
              records[2].event == c::TraceEvent::TRANSITION_REQUEST &&
              records[1].duration_us == 1U,
          "state app trace brackets the exact call and request");
}

void transitionCommitTraceTest()
{
    nura::platform::host::ManualClock clock;
    core::SystemTraceMap trace;
    core::SoftwareBus bus(trace);
    nura::apps::TracingStateAppRunner runner(clock, trace);
    nura::flight::LaunchDetectorApp launch;
    nura::flight::BurnoutDetectorApp burnout;
    nura::flight::ApogeeDetectorApp apogee;
    nura::flight::DrogueSequenceApp drogue;
    nura::flight::MainDeployDetectorApp main_deploy;
    nura::flight::LandingSequenceApp landing;
    nura::flight::FlightCoordinator coordinator(
        launch, burnout, apogee, drogue, main_deploy, landing, &runner);
    nura::apps::FlightCoordinatorApp mission(coordinator, bus, clock, trace);
    c::FlightInputs inputs{};
    inputs.header.sequence = 1U;
    (void)bus.publishFlightInputs(inputs, 1U);
    (void)mission.run(0U, 1U);
    inputs.header.sequence = 2U;
    (void)bus.publishFlightInputs(inputs, 2U);
    (void)mission.run(10U, 2U);

    c::TraceRecord records[32]{};
    uint32_t gap = 0U;
    const size_t count = trace.tryReadAfter(0U, records, 32U, gap);
    bool saw_commit = false;
    for (size_t i = 0U; i < count; ++i)
    {
        saw_commit = saw_commit ||
                     records[i].event == c::TraceEvent::TRANSITION_COMMIT;
    }
    check(saw_commit, "committed FSM transitions have a distinct trace event");
}

void recoveryTimingProfileTest()
{
    const nura::flight::RecoveryTimingPolicy timing{500U, 2000U};
    nura::flight::DrogueSequenceApp drogue(timing);
    nura::flight::LandingSequenceApp landing(timing);
    c::FlightInputs inputs{};

    c::FlightStatus apogee{};
    apogee.state = c::FlightState::APOGEE;
    apogee.apogee_ms = 100U;
    (void)drogue.onEnter(apogee, inputs, 100U);
    const auto drogue_before = drogue.step(apogee, inputs, 599U);
    const auto drogue_at = drogue.step(apogee, inputs, 600U);

    c::FlightStatus deploy{};
    deploy.state = c::FlightState::DEPLOY;
    deploy.deploy_ms = 100U;
    (void)landing.onEnter(deploy, inputs, 100U);
    const auto main_before = landing.step(deploy, inputs, 599U);
    const auto main_at = landing.step(deploy, inputs, 600U);
    check(drogue_before.actuation_count == 0U &&
              drogue_at.actuation_count == 1U &&
              !drogue_at.actuation[0].enabled &&
              main_before.actuation_count == 0U &&
              main_at.actuation_count == 1U &&
              !main_at.actuation[0].enabled,
          "one recovery timing value controls drogue and main pulse length");
}

void coordinatorSafetyTest()
{
    nura::flight::LaunchDetectorApp launch;
    nura::flight::BurnoutDetectorApp burnout;
    nura::flight::ApogeeDetectorApp apogee;
    nura::flight::DrogueSequenceApp drogue;
    nura::flight::MainDeployDetectorApp main_deploy;
    nura::flight::LandingSequenceApp landing;
    nura::flight::FlightCoordinator coordinator(launch, burnout, apogee, drogue,
                                                main_deploy, landing);
    c::FlightInputs inputs{};
    (void)coordinator.initialize(inputs, 0U, 0U);
    (void)coordinator.step(inputs, nullptr, 10U, 10000U);
    (void)coordinator.step(inputs, nullptr, 20U, 20000U);

    c::CommandRequest rejected{};
    rejected.type = c::CommandType::FORCE_RECOVERY_DEPLOY;
    rejected.command_sequence = 7U;
    auto rejected_output = coordinator.step(inputs, &rejected, 30U, 30000U);
    check(rejected_output.status.state == c::FlightState::ARMED &&
              rejected_output.decision_count == 1U &&
              rejected_output.decisions[0].result == c::DecisionResult::REJECT,
          "force recovery is rejected outside launch/coast");

    coordinator.setStateForReplay(c::FlightState::COAST, 1000U);
    c::CommandRequest accepted = rejected;
    accepted.command_sequence = 8U;
    auto accepted_output = coordinator.step(inputs, &accepted, 2000U, 2000000U);
    check(accepted_output.status.state == c::FlightState::APOGEE &&
              accepted_output.status.force_recovery_executed &&
              accepted_output.actuation_count == 1U &&
              accepted_output.actuation[0].enabled,
          "force recovery enters guarded drogue sequence");

    inputs.abort_active = true;
    auto abort_output = coordinator.step(inputs, nullptr, 2010U, 2010000U);
    check(abort_output.status.state == c::FlightState::SAFE &&
              abort_output.actuation_count == 1U &&
              abort_output.actuation[0].operation == c::ActuationOperation::ALL_OFF,
          "abort forces safe and all-off intent");
}

void recoveryGuardTest()
{
    nura::platform::host::ManualClock clock;
    nura::platform::host::FakeFlightPlatform psp;
    core::SystemTraceMap trace;
    core::SoftwareBus bus(trace);
    nura::apps::RecoveryActuationApp recovery(psp, bus, clock, trace);

    c::FlightStatus safe{};
    safe.header.producer = c::AppId::FLIGHT_COORDINATOR;
    safe.header.sequence = 1U;
    safe.header.publish_time_us = 1U;
    safe.state = c::FlightState::SAFE;
    safe.transition_sequence = 2U;
    (void)bus.publishFlightStatus(safe, 1U);

    c::ActuationIntent invalid{};
    invalid.sequence = 1U;
    invalid.transition_sequence = 2U;
    invalid.operation = c::ActuationOperation::SET_CHANNEL;
    invalid.channel = c::RecoveryChannel::DROGUE_PRIMARY;
    invalid.enabled = true;
    invalid.authorized_state = c::FlightState::APOGEE;
    invalid.source = c::AppId::DROGUE_SEQUENCE;
    (void)bus.pushActuation(invalid, 1U, 2U);
    const auto result = recovery.run(0U, 1U);
    check(result == core::AppRunResult::DEGRADED && !psp.drogueEnabled(),
          "recovery adapter rejects stale or wrong-state ON intent");
}
} // namespace

int main()
{
    queueTest();
    traceGapTest();
    appPolicyTest();
    executionGateTest();
    exactStateTraceTest();
    transitionCommitTraceTest();
    recoveryTimingProfileTest();
    coordinatorSafetyTest();
    recoveryGuardTest();
    return failures == 0 ? 0 : 1;
}
