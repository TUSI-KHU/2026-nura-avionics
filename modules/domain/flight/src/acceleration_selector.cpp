#include "nura/flight/acceleration_selector.h"

#include <cmath>

#include "nura/contracts/common.h"
#include "nura/flight/flight_math.h"
#include "nura/flight/flight_policy.h"

namespace nura::flight
{

void AccelerationSelector::reset()
{
    last_sample_time_ms_ = 0U;
    last_source_ = AccelerationSource::NONE;
}

bool AccelerationSelector::consume(const nura::contracts::FlightInputs &inputs,
                                   uint32_t now_ms, SelectedAcceleration &sample)
{
    SelectedAcceleration candidate{};
    if (!lowG(inputs, now_ms, candidate) && !highG(inputs, now_ms, candidate))
    {
        return false;
    }

    if (candidate.source == last_source_ &&
        candidate.sample_time_ms == last_sample_time_ms_)
    {
        return false;
    }

    last_source_ = candidate.source;
    last_sample_time_ms_ = candidate.sample_time_ms;
    sample = candidate;
    return true;
}

bool AccelerationSelector::lowG(const nura::contracts::FlightInputs &inputs,
                                uint32_t now_ms, SelectedAcceleration &sample) const
{
    const auto &imu = inputs.low_g;
    if (imu.header.sample_time_ms == 0U ||
        (now_ms - imu.header.sample_time_ms) > FlightPolicy::kAccelFallbackMaxSampleAgeMs ||
        !finite3(imu.accel_x_mps2, imu.accel_y_mps2, imu.accel_z_mps2))
    {
        return false;
    }

    const float norm_g = accelNormGFromMps2(imu.accel_x_mps2, imu.accel_y_mps2,
                                            imu.accel_z_mps2);
    if (!std::isfinite(norm_g))
    {
        return false;
    }

    sample.source = AccelerationSource::LOW_G;
    sample.sample_time_ms = imu.header.sample_time_ms;
    sample.norm_g = norm_g;
    return true;
}

bool AccelerationSelector::highG(const nura::contracts::FlightInputs &inputs,
                                 uint32_t now_ms, SelectedAcceleration &sample) const
{
    const auto &imu = inputs.high_g;
    const uint32_t flags = imu.header.status_flags;
    if (!inputs.health.high_accel_ok ||
        (flags & nura::contracts::SAMPLE_STATUS_CONNECTED) == 0U ||
        (flags & nura::contracts::SAMPLE_STATUS_NEW_DATA) == 0U ||
        imu.header.sample_time_ms == 0U ||
        (now_ms - imu.header.sample_time_ms) > FlightPolicy::kAccelFallbackMaxSampleAgeMs ||
        !finite3(imu.accel_x_g, imu.accel_y_g, imu.accel_z_g))
    {
        return false;
    }

    const float norm_g = accelNormG(imu.accel_x_g, imu.accel_y_g, imu.accel_z_g);
    if (!std::isfinite(norm_g))
    {
        return false;
    }

    sample.source = AccelerationSource::HIGH_G;
    sample.sample_time_ms = imu.header.sample_time_ms;
    sample.norm_g = norm_g;
    return true;
}

} // namespace nura::flight
