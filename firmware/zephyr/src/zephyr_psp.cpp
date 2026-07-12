#include "zephyr_psp.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "nura/flight/flight_policy.h"

namespace nura::platform::zephyr
{
namespace c = nura::contracts;

#if !CONFIG_NURA_FAKE_PSP
namespace
{
const gpio_dt_spec kDrogueA =
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(nura_drogue_a), gpios, {});
const gpio_dt_spec kDrogueB =
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(nura_drogue_b), gpios, {});
const gpio_dt_spec kMainA =
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(nura_main_a), gpios, {});
const gpio_dt_spec kMainB =
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(nura_main_b), gpios, {});

bool configureOutput(const gpio_dt_spec &output)
{
    return output.port != nullptr && gpio_is_ready_dt(&output) &&
           gpio_pin_configure_dt(&output, GPIO_OUTPUT_INACTIVE) == 0;
}

bool setOutput(const gpio_dt_spec &output, bool enabled)
{
    return gpio_pin_set_dt(&output, enabled ? 1 : 0) == 0;
}
} // namespace
#endif

PortResult ZephyrPsp::initialize()
{
#if CONFIG_NURA_FAKE_PSP
    recovery_ready_ = true;
    return PortResult::OK;
#else
    if (!recovery_ready_)
    {
        const PortResult result = configureRecoveryOutputs();
        if (result != PortResult::OK) return result;
    }
    return PortResult::OK;
#endif
}

PortResult ZephyrPsp::read(uint32_t now_ms, c::LowGImuSample &sample)
{
#if CONFIG_NURA_FAKE_PSP
    const float norm_g = now_ms >= 1000U && now_ms < 2500U
                             ? 3.0f
                             : (now_ms >= 2500U ? 0.5f : 1.0f);
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID |
                                 c::SAMPLE_STATUS_CONNECTED |
                                 c::SAMPLE_STATUS_NEW_DATA;
    sample.accel_z_mps2 = norm_g * nura::flight::FlightPolicy::kGravityMps2;
    sample.attitude_valid = true;
    sample.tilt_valid = true;
    return PortResult::OK;
#else
    (void)now_ms;
    (void)sample;
    return PortResult::UNAVAILABLE;
#endif
}

PortResult ZephyrPsp::read(uint32_t now_ms, c::HighGImuSample &sample)
{
#if CONFIG_NURA_FAKE_PSP
    const float norm_g = now_ms >= 1000U && now_ms < 2500U
                             ? 3.0f
                             : (now_ms >= 2500U ? 0.5f : 1.0f);
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID |
                                 c::SAMPLE_STATUS_CONNECTED |
                                 c::SAMPLE_STATUS_NEW_DATA;
    sample.accel_z_g = norm_g;
    return PortResult::OK;
#else
    (void)now_ms;
    (void)sample;
    return PortResult::UNAVAILABLE;
#endif
}

PortResult ZephyrPsp::read(uint32_t now_ms, c::BarometerSample &sample)
{
#if CONFIG_NURA_FAKE_PSP
    if ((now_ms % 50U) != 0U) return PortResult::NO_DATA;
    float altitude_m = 0.0f;
    if (now_ms > 1000U && now_ms <= 10500U)
    {
        const float ratio = static_cast<float>(now_ms - 1000U) / 9500.0f;
        altitude_m = 400.0f * ((2.0f * ratio) - (ratio * ratio));
    }
    else if (now_ms > 10500U)
    {
        altitude_m = 400.0f -
                     (static_cast<float>(now_ms - 10500U) / 100.0f);
        if (altitude_m < 0.0f) altitude_m = 0.0f;
    }
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID |
                                 c::SAMPLE_STATUS_REFERENCE_VALID |
                                 c::SAMPLE_STATUS_CONNECTED |
                                 c::SAMPLE_STATUS_NEW_DATA;
    sample.pressure_pa = 101325.0f;
    sample.reference_pressure_pa = 101325.0f;
    sample.raw_altitude_m = altitude_m;
    sample.filtered_altitude_m = altitude_m;
    return PortResult::OK;
#else
    (void)now_ms;
    (void)sample;
    return PortResult::UNAVAILABLE;
#endif
}

PortResult ZephyrPsp::read(uint32_t now_ms, c::MagnetometerSample &sample)
{
#if CONFIG_NURA_FAKE_PSP
    if ((now_ms % 100U) != 0U) return PortResult::NO_DATA;
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID | c::SAMPLE_STATUS_CONNECTED;
    return PortResult::OK;
#else
    (void)now_ms;
    (void)sample;
    return PortResult::UNAVAILABLE;
#endif
}

PortResult ZephyrPsp::read(uint32_t now_ms, c::GnssSample &sample)
{
#if CONFIG_NURA_FAKE_PSP
    if ((now_ms % 50U) != 0U) return PortResult::NO_DATA;
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID | c::SAMPLE_STATUS_CONNECTED;
    sample.has_fix = true;
    sample.satellites = 10U;
    return PortResult::OK;
#else
    (void)now_ms;
    (void)sample;
    return PortResult::UNAVAILABLE;
#endif
}

PortResult ZephyrPsp::read(uint32_t now_ms, c::PowerSample &sample)
{
#if CONFIG_NURA_FAKE_PSP
    if ((now_ms % 100U) != 0U) return PortResult::NO_DATA;
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID;
    sample.battery_mv = 12000U;
    return PortResult::OK;
#else
    (void)now_ms;
    (void)sample;
    return PortResult::UNAVAILABLE;
#endif
}

PortResult ZephyrPsp::read(uint32_t now_ms, c::SafetyStatus &status)
{
#if CONFIG_NURA_FAKE_PSP
    status.header.sample_time_ms = now_ms;
    status.header.status_flags = c::SAMPLE_STATUS_VALID;
    status.abort_active = false;
    return PortResult::OK;
#else
    (void)now_ms;
    (void)status;
    return PortResult::UNAVAILABLE;
#endif
}

PortResult ZephyrPsp::configureRecoveryOutputs()
{
#if CONFIG_NURA_FAKE_PSP
    recovery_ready_ = true;
    return PortResult::OK;
#else
    recovery_ready_ = configureOutput(kDrogueA) && configureOutput(kDrogueB) &&
                      configureOutput(kMainA) && configureOutput(kMainB);
    if (!recovery_ready_) return PortResult::UNAVAILABLE;
    return allOff();
#endif
}

PortResult ZephyrPsp::allOff()
{
#if CONFIG_NURA_FAKE_PSP
    return PortResult::OK;
#else
    if (!recovery_ready_) return PortResult::UNAVAILABLE;
    const bool ok = setOutput(kDrogueA, false) && setOutput(kDrogueB, false) &&
                    setOutput(kMainA, false) && setOutput(kMainB, false);
    return ok ? PortResult::OK : PortResult::FAULT;
#endif
}

PortResult ZephyrPsp::setChannel(c::RecoveryChannel channel, bool enabled)
{
#if CONFIG_NURA_FAKE_PSP
    printk("RECOVERY channel=%u enabled=%u\n",
           static_cast<unsigned>(channel), enabled ? 1U : 0U);
    return PortResult::OK;
#else
    if (!recovery_ready_) return PortResult::UNAVAILABLE;
    bool ok = false;
    if (channel == c::RecoveryChannel::DROGUE_PRIMARY)
    {
        ok = setOutput(kDrogueA, enabled) && setOutput(kDrogueB, enabled);
    }
    else if (channel == c::RecoveryChannel::MAIN_PRIMARY)
    {
        ok = setOutput(kMainA, enabled) && setOutput(kMainB, enabled);
    }
    if (!ok)
    {
        (void)allOff();
        return PortResult::FAULT;
    }
    return PortResult::OK;
#endif
}

PortResult ZephyrTraceSink::tryWrite(const c::TraceRecord &record)
{
#if CONFIG_NURA_TRACE_CONSOLE
    printk("TRACE,%u,%llu,%u,%s,%s,%u,%u,%d,%u,%u\n",
           record.sequence, static_cast<unsigned long long>(record.timestamp_us),
           record.cycle_id, c::traceEventName(record.event), c::appName(record.app),
           static_cast<unsigned>(record.topic), static_cast<unsigned>(record.state),
           record.result, record.duration_us, record.detail);
#else
    (void)record;
#endif
    return PortResult::OK;
}

PortResult ZephyrEventSink::tryWrite(const c::TransitionEvent &event)
{
    printk("FSM transition=%u from=%u to=%u at_ms=%u by=%s\n", event.sequence,
           static_cast<unsigned>(event.previous), static_cast<unsigned>(event.current),
           event.timestamp_ms, c::appName(event.requested_by));
    return PortResult::OK;
}

PortResult ZephyrEventSink::tryWrite(const c::DecisionTrace &event)
{
    if (event.result == c::DecisionResult::ACCEPT)
    {
        printk("FSM decision=%u kind=%u state=%u at_ms=%u\n", event.sequence,
               static_cast<unsigned>(event.kind), static_cast<unsigned>(event.state),
               event.timestamp_ms);
    }
    return PortResult::OK;
}

PortResult ZephyrEventSink::tryWrite(const c::SensorFaultEvent &event)
{
    printk("SENSOR fault topic=%u flags=%u at_ms=%u\n",
           static_cast<unsigned>(event.sensor_topic), event.fault_flags,
           event.timestamp_ms);
    return PortResult::OK;
}

} // namespace nura::platform::zephyr
