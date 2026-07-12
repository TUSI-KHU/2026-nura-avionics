#include <cstdio>
#include <filesystem>

#include "nura/apps/event_recorder_app.h"
#include "nura/apps/flight_coordinator_app.h"
#include "nura/apps/input_aggregator_app.h"
#include "nura/apps/recovery_actuation_app.h"
#include "nura/apps/sensor_apps.h"
#include "nura/apps/state_app_runner.h"
#include "nura/apps/supervisor_app.h"
#include "nura/apps/trace_exporter_app.h"
#include "nura/config/runtime_profile.h"
#include "nura/flight/apps/apogee_detector_app.h"
#include "nura/flight/apps/burnout_detector_app.h"
#include "nura/flight/apps/drogue_sequence_app.h"
#include "nura/flight/apps/landing_sequence_app.h"
#include "nura/flight/apps/launch_detector_app.h"
#include "nura/flight/apps/main_deploy_detector_app.h"
#include "nura/platform/host/fake_platform.h"

namespace c = nura::contracts;
namespace core = nura::core;
namespace apps = nura::apps;
namespace config = nura::config;
namespace flight = nura::flight;
namespace host = nura::platform::host;

int main(int argc, char **argv)
{
    const std::filesystem::path trace_path =
        argc > 1 ? argv[1] : "build/traces/flight_trace.csv";
    std::filesystem::create_directories(trace_path.parent_path());

    host::ManualClock clock;
    host::FakeFlightPlatform psp;
    host::CsvTraceSink trace_sink(trace_path.c_str());
    host::VectorEventSink event_sink;
    if (!trace_sink.valid())
    {
        std::fprintf(stderr, "cannot open TraceMap output: %s\n",
                     trace_path.c_str());
        return 2;
    }

    core::SystemTraceMap trace_map;
    core::SoftwareBus bus(trace_map);
    core::Executive executive(clock, trace_map);
    apps::TracingStateAppRunner state_app_runner(clock, trace_map);

    flight::LaunchDetectorApp launch;
    flight::BurnoutDetectorApp burnout;
    flight::ApogeeDetectorApp apogee;
    flight::DrogueSequenceApp drogue;
    flight::MainDeployDetectorApp main_deploy;
    flight::LandingSequenceApp landing;
    flight::FlightCoordinator coordinator(launch, burnout, apogee, drogue,
                                          main_deploy, landing,
                                          &state_app_runner);

    apps::LowGSensorApp low_g(psp, bus, clock);
    apps::HighGSensorApp high_g(psp, bus, clock);
    apps::BarometerSensorApp barometer(psp, bus, clock);
    apps::MagnetometerSensorApp magnetometer(psp, bus, clock);
    apps::GnssSensorApp gnss(psp, bus, clock);
    apps::PowerSensorApp power(psp, bus, clock);
    apps::SafetyInputApp safety(psp, bus, clock);
    apps::SupervisorApp supervisor(executive, bus, clock);
    apps::InputAggregatorApp input_aggregator(bus, clock);
    apps::FlightCoordinatorApp mission(coordinator, bus, clock, trace_map);
    apps::RecoveryActuationApp recovery(psp, bus, clock, trace_map);
    apps::EventRecorderApp event_recorder(bus, clock, event_sink);
    apps::TraceExporterApp trace_exporter(trace_map, trace_sink);

    c::FlightStatus status{};
    auto runScheduled = [&](core::IExecutableApp &app, uint32_t now_ms,
                            uint32_t cycle, c::FlightState state) {
        const config::AppDescriptor *descriptor =
            config::appDescriptor(app.id());
        if (descriptor == nullptr || descriptor->period_ms == 0U ||
            (now_ms % descriptor->period_ms) != 0U)
        {
            return;
        }
        (void)executive.runIfEnabled(
            app, now_ms, cycle, descriptor->deadline_us, state,
            config::enabledApplicationMask(state));
    };

    for (uint32_t now_ms = 0U, cycle = 1U; now_ms <= 55000U;
         now_ms += 2U, ++cycle)
    {
        clock.setNowMs(now_ms);
        runScheduled(low_g, now_ms, cycle, status.state);
        runScheduled(high_g, now_ms, cycle, status.state);
        runScheduled(barometer, now_ms, cycle, status.state);
        runScheduled(magnetometer, now_ms, cycle, status.state);
        runScheduled(gnss, now_ms, cycle, status.state);
        runScheduled(power, now_ms, cycle, status.state);
        runScheduled(safety, now_ms, cycle, status.state);
        if ((now_ms % config::appDescriptor(
                          c::AppId::FLIGHT_COORDINATOR)->period_ms) == 0U)
        {
            runScheduled(input_aggregator, now_ms, cycle, status.state);
            runScheduled(mission, now_ms, cycle, status.state);
        }
        runScheduled(recovery, now_ms, cycle, status.state);
        runScheduled(supervisor, now_ms, cycle, status.state);
        runScheduled(event_recorder, now_ms, cycle, status.state);
        runScheduled(trace_exporter, now_ms, cycle, status.state);
        (void)bus.readFlightStatus(status, c::AppId::SYSTEM, cycle, clock.nowUs());
    }

    for (uint8_t i = 0U; i < 32U; ++i)
    {
        if (trace_exporter.run(55000U, 5501U) == core::AppRunResult::NO_INPUT)
        {
            break;
        }
    }

    const auto metrics = bus.queueMetrics();
    const bool reached_ground = status.state == c::FlightState::GROUND;
    const bool outputs_off = !psp.drogueEnabled() && !psp.mainEnabled();
    const bool saw_all_transitions = event_sink.transitions().size() >= 8U;
    const bool saw_decisions = !event_sink.decisions().empty();
    const bool no_critical_drop = metrics.command_dropped == 0U &&
                                  metrics.actuation_dropped == 0U &&
                                  metrics.transition_dropped == 0U;
    const bool complete_trace = trace_exporter.exportGapCount() == 0U;

    std::printf("state=%u transitions=%zu decisions=%zu recovery_events=%zu "
                "trace_dropped=%u trace_overwritten=%u export_gap=%u\n",
                static_cast<unsigned>(status.state),
                event_sink.transitions().size(), event_sink.decisions().size(),
                psp.recoveryEvents().size(), trace_map.dropped(),
                trace_map.overwritten(), trace_exporter.exportGapCount());
    std::printf("TraceMap CSV: %s\n", trace_path.c_str());

    if (!reached_ground || !outputs_off || !saw_all_transitions ||
        !saw_decisions || !no_critical_drop || !complete_trace)
    {
        std::fprintf(stderr, "host flight acceptance failed\n");
        return 1;
    }
    return 0;
}
