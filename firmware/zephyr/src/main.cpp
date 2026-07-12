#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

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
#include "nura/flight/flight_policy.h"
#include "zephyr_psp.h"

namespace c = nura::contracts;
namespace core = nura::core;
namespace apps = nura::apps;
namespace config = nura::config;
namespace flight = nura::flight;
namespace zp = nura::platform::zephyr;

namespace
{
constexpr uint16_t stackBytes(c::AppId id)
{
    const config::AppDescriptor *descriptor = config::appDescriptor(id);
    return descriptor == nullptr ? 0U : descriptor->stack_bytes;
}

struct Runtime
{
    zp::ZephyrClock clock{};
    zp::ZephyrPsp psp{};
    zp::ZephyrTraceSink trace_sink{};
    zp::ZephyrEventSink event_sink{};
    core::SystemTraceMap trace_map{};
    core::SoftwareBus bus;
    core::Executive executive;
    apps::TracingStateAppRunner state_app_runner;

    flight::LaunchDetectorApp launch{};
    flight::BurnoutDetectorApp burnout{};
    flight::ApogeeDetectorApp apogee{};
    flight::DrogueSequenceApp drogue{};
    flight::MainDeployDetectorApp main_deploy{};
    flight::LandingSequenceApp landing{};
    flight::FlightCoordinator coordinator;

    apps::LowGSensorApp low_g;
    apps::HighGSensorApp high_g;
    apps::BarometerSensorApp barometer;
    apps::MagnetometerSensorApp magnetometer;
    apps::GnssSensorApp gnss;
    apps::PowerSensorApp power;
    apps::SafetyInputApp safety;
    apps::SupervisorApp supervisor;
    apps::InputAggregatorApp input_aggregator;
    apps::FlightCoordinatorApp mission;
    apps::RecoveryActuationApp recovery;
    apps::EventRecorderApp event_recorder;
    apps::TraceExporterApp trace_exporter;

    Runtime()
        : bus(trace_map),
          executive(clock, trace_map),
          state_app_runner(clock, trace_map),
          coordinator(launch, burnout, apogee, drogue, main_deploy, landing,
                      &state_app_runner),
          low_g(psp, bus, clock),
          high_g(psp, bus, clock),
          barometer(psp, bus, clock),
          magnetometer(psp, bus, clock),
          gnss(psp, bus, clock),
          power(psp, bus, clock),
          safety(psp, bus, clock),
          supervisor(executive, bus, clock),
          input_aggregator(bus, clock),
          mission(coordinator, bus, clock, trace_map),
          recovery(psp, bus, clock, trace_map),
          event_recorder(bus, clock, event_sink),
          trace_exporter(trace_map, trace_sink)
    {
    }
};

struct PeriodicAppContext
{
    Runtime *runtime = nullptr;
    core::IExecutableApp *app = nullptr;
    const config::AppDescriptor *descriptor = nullptr;
    uint32_t start_delay_ms = 0U;
    c::FlightState last_state = c::FlightState::INIT;
    uint64_t last_enabled_mask = config::enabledApplicationMask(c::FlightState::INIT);
};

K_THREAD_STACK_DEFINE(low_g_stack, stackBytes(c::AppId::LOW_G_SENSOR));
K_THREAD_STACK_DEFINE(high_g_stack, stackBytes(c::AppId::HIGH_G_SENSOR));
K_THREAD_STACK_DEFINE(barometer_stack, stackBytes(c::AppId::BAROMETER_SENSOR));
K_THREAD_STACK_DEFINE(magnetometer_stack,
                      stackBytes(c::AppId::MAGNETOMETER_SENSOR));
K_THREAD_STACK_DEFINE(gnss_stack, stackBytes(c::AppId::GNSS_SENSOR));
K_THREAD_STACK_DEFINE(power_stack, stackBytes(c::AppId::POWER_SENSOR));
K_THREAD_STACK_DEFINE(safety_stack, stackBytes(c::AppId::SAFETY_INPUT));
K_THREAD_STACK_DEFINE(mission_stack, stackBytes(c::AppId::FLIGHT_COORDINATOR));
K_THREAD_STACK_DEFINE(recovery_stack, stackBytes(c::AppId::RECOVERY_ACTUATION));
K_THREAD_STACK_DEFINE(supervisor_stack, stackBytes(c::AppId::SUPERVISOR));
K_THREAD_STACK_DEFINE(event_stack, stackBytes(c::AppId::EVENT_RECORDER));
K_THREAD_STACK_DEFINE(trace_stack, stackBytes(c::AppId::TRACE_EXPORTER));

k_thread low_g_thread;
k_thread high_g_thread;
k_thread barometer_thread;
k_thread magnetometer_thread;
k_thread gnss_thread;
k_thread power_thread;
k_thread safety_thread;
k_thread mission_thread;
k_thread recovery_thread;
k_thread supervisor_thread;
k_thread event_thread;
k_thread trace_thread;

PeriodicAppContext low_g_context;
PeriodicAppContext high_g_context;
PeriodicAppContext barometer_context;
PeriodicAppContext magnetometer_context;
PeriodicAppContext gnss_context;
PeriodicAppContext power_context;
PeriodicAppContext safety_context;
PeriodicAppContext recovery_context;
PeriodicAppContext supervisor_context;
PeriodicAppContext event_context;
PeriodicAppContext trace_context;

void updateSchedule(PeriodicAppContext &context, uint32_t cycle_id)
{
    c::AppEnableSet enabled{};
    if (context.runtime->bus.readAppEnable(
            enabled, context.app->id(), cycle_id,
            context.runtime->clock.nowUs()))
    {
        context.last_state = enabled.state;
        context.last_enabled_mask = enabled.enabled_mask;
    }
}

void periodicAppEntry(void *arg, void *, void *)
{
    auto &context = *static_cast<PeriodicAppContext *>(arg);
    if (context.start_delay_ms != 0U)
    {
        k_sleep(K_MSEC(context.start_delay_ms));
    }
    uint32_t cycle_id = 1U;
    while (true)
    {
        const uint32_t now_ms = static_cast<uint32_t>(k_uptime_get());
        updateSchedule(context, cycle_id);
        (void)context.runtime->executive.runIfEnabled(
            *context.app, now_ms, cycle_id, context.descriptor->deadline_us,
            context.last_state, context.last_enabled_mask);
        ++cycle_id;
        k_sleep(K_MSEC(context.descriptor->period_ms));
    }
}

void missionEntry(void *arg, void *, void *)
{
    auto &runtime = *static_cast<Runtime *>(arg);
    const config::AppDescriptor &aggregator =
        *config::appDescriptor(c::AppId::INPUT_AGGREGATOR);
    const config::AppDescriptor &mission =
        *config::appDescriptor(c::AppId::FLIGHT_COORDINATOR);
    uint32_t cycle_id = 1U;
    while (true)
    {
        const uint32_t now_ms = static_cast<uint32_t>(k_uptime_get());
        const c::FlightState state = runtime.coordinator.status().state;
        const uint64_t enabled_mask = config::enabledApplicationMask(state);
        (void)runtime.executive.runIfEnabled(
            runtime.input_aggregator, now_ms, cycle_id,
            aggregator.deadline_us, state, enabled_mask);
        (void)runtime.executive.runIfEnabled(
            runtime.mission, now_ms, cycle_id, mission.deadline_us,
            state, enabled_mask);
        ++cycle_id;
        k_sleep(K_MSEC(mission.period_ms));
    }
}

void initializeContext(PeriodicAppContext &context, Runtime &runtime,
                       core::IExecutableApp &app, uint32_t start_delay_ms)
{
    context.runtime = &runtime;
    context.app = &app;
    context.descriptor = config::appDescriptor(app.id());
    context.start_delay_ms = start_delay_ms;
}

void createPeriodicThread(k_thread &thread, k_thread_stack_t *stack,
                          size_t stack_size, PeriodicAppContext &context,
                          const char *name)
{
    k_thread_create(&thread, stack, stack_size, periodicAppEntry, &context,
                    nullptr, nullptr,
                    K_PRIO_PREEMPT(context.descriptor->priority), 0, K_NO_WAIT);
    k_thread_name_set(&thread, name);
}
} // namespace

int main()
{
    static Runtime runtime;
#ifdef CONFIG_FLASH_SIZE
    constexpr unsigned flash_kb = CONFIG_FLASH_SIZE;
#else
    constexpr unsigned flash_kb = 0U;
#endif
    printk("NURA Zephyr avionics fake_psp=%u trace=%u ram_kb=%u flash_kb=%u "
           "mission_rev=%u runtime_rev=%u pyro_ms=%u\n",
           IS_ENABLED(CONFIG_NURA_FAKE_PSP) ? 1U : 0U,
           static_cast<unsigned>(runtime.trace_map.capacity()),
           static_cast<unsigned>(CONFIG_SRAM_SIZE), flash_kb,
           static_cast<unsigned>(flight::FlightPolicy::kProfileRevision),
           static_cast<unsigned>(config::kRuntimeProfileRevision),
           static_cast<unsigned>(flight::FlightPolicy::kPyroFireDurationMs));

    const uint32_t settle_ms = CONFIG_NURA_BOARD_SETTLE_MS;
    initializeContext(low_g_context, runtime, runtime.low_g, settle_ms);
    initializeContext(high_g_context, runtime, runtime.high_g, settle_ms);
    initializeContext(barometer_context, runtime, runtime.barometer, settle_ms);
    initializeContext(magnetometer_context, runtime, runtime.magnetometer, settle_ms);
    initializeContext(gnss_context, runtime, runtime.gnss, settle_ms);
    initializeContext(power_context, runtime, runtime.power, settle_ms);
    initializeContext(safety_context, runtime, runtime.safety, settle_ms);
    initializeContext(recovery_context, runtime, runtime.recovery, 0U);
    initializeContext(supervisor_context, runtime, runtime.supervisor, settle_ms);
    initializeContext(event_context, runtime, runtime.event_recorder, 0U);
    initializeContext(trace_context, runtime, runtime.trace_exporter, 0U);

    createPeriodicThread(recovery_thread, recovery_stack,
                         K_THREAD_STACK_SIZEOF(recovery_stack), recovery_context,
                         "recovery");
    k_thread_create(&mission_thread, mission_stack,
                    K_THREAD_STACK_SIZEOF(mission_stack), missionEntry, &runtime,
                    nullptr, nullptr,
                    K_PRIO_PREEMPT(config::appDescriptor(
                        c::AppId::FLIGHT_COORDINATOR)->priority),
                    0, K_NO_WAIT);
    k_thread_name_set(&mission_thread, "mission");

    createPeriodicThread(low_g_thread, low_g_stack,
                         K_THREAD_STACK_SIZEOF(low_g_stack), low_g_context, "low_g");
    createPeriodicThread(high_g_thread, high_g_stack,
                         K_THREAD_STACK_SIZEOF(high_g_stack), high_g_context, "high_g");
    createPeriodicThread(barometer_thread, barometer_stack,
                         K_THREAD_STACK_SIZEOF(barometer_stack), barometer_context,
                         "barometer");
    createPeriodicThread(magnetometer_thread, magnetometer_stack,
                         K_THREAD_STACK_SIZEOF(magnetometer_stack),
                         magnetometer_context, "magnetometer");
    createPeriodicThread(gnss_thread, gnss_stack,
                         K_THREAD_STACK_SIZEOF(gnss_stack), gnss_context, "gnss");
    createPeriodicThread(power_thread, power_stack,
                         K_THREAD_STACK_SIZEOF(power_stack), power_context, "power");
    createPeriodicThread(safety_thread, safety_stack,
                         K_THREAD_STACK_SIZEOF(safety_stack), safety_context, "safety");
    createPeriodicThread(supervisor_thread, supervisor_stack,
                         K_THREAD_STACK_SIZEOF(supervisor_stack), supervisor_context,
                         "supervisor");
    createPeriodicThread(event_thread, event_stack,
                         K_THREAD_STACK_SIZEOF(event_stack), event_context, "events");
    createPeriodicThread(trace_thread, trace_stack,
                         K_THREAD_STACK_SIZEOF(trace_stack), trace_context, "trace");
    return 0;
}
