#include "nura/platform/host/fake_platform.h"

#include <algorithm>

#include "nura/config/runtime_profile.h"
#include "nura/flight/flight_policy.h"

namespace nura::platform::host
{
namespace c = nura::contracts;

namespace
{
constexpr uint32_t kLaunchMs = 1000U;
constexpr uint32_t kBurnoutMs = 2500U;
constexpr uint32_t kApogeeMs = 10500U;
constexpr float kApogeeAltitudeM = 400.0f;
constexpr float kDescentRateMps = 10.0f;
}

PortResult FakeFlightPlatform::read(uint32_t now_ms, c::LowGImuSample &sample)
{
    const float norm_g = now_ms >= kLaunchMs && now_ms < kBurnoutMs
                             ? 3.0f
                             : (now_ms >= kBurnoutMs ? 0.5f : 1.0f);
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID |
                                 c::SAMPLE_STATUS_CONNECTED |
                                 c::SAMPLE_STATUS_NEW_DATA;
    sample.accel_z_mps2 = norm_g * nura::flight::FlightPolicy::kGravityMps2;
    sample.tilt_valid = true;
    sample.attitude_valid = true;
    sample.tilt_angle_deg = 0.0f;
    return PortResult::OK;
}

PortResult FakeFlightPlatform::read(uint32_t now_ms, c::HighGImuSample &sample)
{
    const float norm_g = now_ms >= kLaunchMs && now_ms < kBurnoutMs
                             ? 3.0f
                             : (now_ms >= kBurnoutMs ? 0.5f : 1.0f);
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID |
                                 c::SAMPLE_STATUS_CONNECTED |
                                 c::SAMPLE_STATUS_NEW_DATA;
    sample.accel_z_g = norm_g;
    return PortResult::OK;
}

PortResult FakeFlightPlatform::read(uint32_t now_ms, c::BarometerSample &sample)
{
    if ((now_ms % nura::config::appDescriptor(
                       c::AppId::BAROMETER_SENSOR)->period_ms) != 0U)
    {
        return PortResult::NO_DATA;
    }
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID |
                                 c::SAMPLE_STATUS_REFERENCE_VALID |
                                 c::SAMPLE_STATUS_CONNECTED |
                                 c::SAMPLE_STATUS_NEW_DATA;
    sample.reference_pressure_pa = 101325.0f;
    sample.pressure_pa = 101325.0f;
    sample.raw_altitude_m = altitudeAt(now_ms);
    sample.filtered_altitude_m = sample.raw_altitude_m;
    return PortResult::OK;
}

PortResult FakeFlightPlatform::read(uint32_t now_ms, c::MagnetometerSample &sample)
{
    if ((now_ms % 100U) != 0U) return PortResult::NO_DATA;
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID | c::SAMPLE_STATUS_CONNECTED;
    sample.magnetic_x_ut = 20.0f;
    sample.magnetic_y_ut = 5.0f;
    sample.magnetic_z_ut = 40.0f;
    return PortResult::OK;
}

PortResult FakeFlightPlatform::read(uint32_t now_ms, c::GnssSample &sample)
{
    if ((now_ms % 200U) != 0U) return PortResult::NO_DATA;
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID | c::SAMPLE_STATUS_CONNECTED;
    sample.latitude_deg = 37.0;
    sample.longitude_deg = 127.0;
    sample.altitude_m = altitudeAt(now_ms);
    sample.satellites = 10U;
    sample.has_fix = true;
    return PortResult::OK;
}

PortResult FakeFlightPlatform::read(uint32_t now_ms, c::PowerSample &sample)
{
    if ((now_ms % 100U) != 0U) return PortResult::NO_DATA;
    sample.header.sample_time_ms = now_ms;
    sample.header.status_flags = c::SAMPLE_STATUS_VALID;
    sample.battery_mv = 7400U;
    return PortResult::OK;
}

PortResult FakeFlightPlatform::read(uint32_t now_ms, c::SafetyStatus &status)
{
    status.header.sample_time_ms = now_ms;
    status.header.status_flags = c::SAMPLE_STATUS_VALID;
    status.abort_active = abort_active_;
    return PortResult::OK;
}

PortResult FakeFlightPlatform::allOff()
{
    if (recovery_failure_) return PortResult::FAULT;
    drogue_enabled_ = false;
    main_enabled_ = false;
    record(c::ActuationOperation::ALL_OFF, c::RecoveryChannel::DROGUE_PRIMARY,
           false);
    return PortResult::OK;
}

PortResult FakeFlightPlatform::setChannel(c::RecoveryChannel channel, bool enabled)
{
    if (recovery_failure_) return PortResult::FAULT;
    if (channel == c::RecoveryChannel::DROGUE_PRIMARY)
    {
        drogue_enabled_ = enabled;
    }
    else if (channel == c::RecoveryChannel::MAIN_PRIMARY)
    {
        main_enabled_ = enabled;
    }
    else
    {
        return PortResult::FAULT;
    }
    record(c::ActuationOperation::SET_CHANNEL, channel, enabled);
    return PortResult::OK;
}

float FakeFlightPlatform::altitudeAt(uint32_t now_ms)
{
    if (now_ms <= kLaunchMs) return 0.0f;
    if (now_ms <= kApogeeMs)
    {
        const float ratio = static_cast<float>(now_ms - kLaunchMs) /
                            static_cast<float>(kApogeeMs - kLaunchMs);
        return kApogeeAltitudeM * ((2.0f * ratio) - (ratio * ratio));
    }
    const float descent_seconds = static_cast<float>(now_ms - kApogeeMs) / 1000.0f;
    return std::max(0.0f, kApogeeAltitudeM - (descent_seconds * kDescentRateMps));
}

void FakeFlightPlatform::record(c::ActuationOperation operation,
                                c::RecoveryChannel channel, bool enabled)
{
    recovery_events_.push_back({++recovery_order_, operation, channel, enabled});
}

CsvTraceSink::CsvTraceSink(const char *path)
{
    file_ = std::fopen(path, "w");
    if (file_ != nullptr)
    {
        std::fputs("sequence,timestamp_us,cycle_id,event,app,peer,topic,state,result,duration_us,correlation_id,detail\n",
                   file_);
    }
}

CsvTraceSink::~CsvTraceSink()
{
    if (file_ != nullptr) std::fclose(file_);
}

PortResult CsvTraceSink::tryWrite(const c::TraceRecord &record)
{
    if (file_ == nullptr) return PortResult::FAULT;
    const int written = std::fprintf(
        file_, "%u,%llu,%u,%s,%s,%s,%u,%u,%d,%u,%u,%u\n",
        record.sequence, static_cast<unsigned long long>(record.timestamp_us),
        record.cycle_id, c::traceEventName(record.event), c::appName(record.app),
        c::appName(record.peer), static_cast<unsigned>(record.topic),
        static_cast<unsigned>(record.state), record.result, record.duration_us,
        record.correlation_id, record.detail);
    return written > 0 ? PortResult::OK : PortResult::FAULT;
}

} // namespace nura::platform::host
