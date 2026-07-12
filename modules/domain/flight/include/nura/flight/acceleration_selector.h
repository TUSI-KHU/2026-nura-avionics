#pragma once

#include <cstdint>

#include "nura/contracts/flight_topics.h"

namespace nura::flight
{

enum class AccelerationSource : uint8_t
{
    NONE = 0U,
    HIGH_G = 1U,
    LOW_G = 2U,
};

struct SelectedAcceleration
{
    AccelerationSource source = AccelerationSource::NONE;
    uint32_t sample_time_ms = 0U;
    float norm_g = 0.0f;
};

class AccelerationSelector
{
public:
    bool consume(const nura::contracts::FlightInputs &inputs, uint32_t now_ms,
                 SelectedAcceleration &sample);
    void reset();

private:
    bool lowG(const nura::contracts::FlightInputs &inputs, uint32_t now_ms,
              SelectedAcceleration &sample) const;
    bool highG(const nura::contracts::FlightInputs &inputs, uint32_t now_ms,
               SelectedAcceleration &sample) const;

    uint32_t last_sample_time_ms_ = 0U;
    AccelerationSource last_source_ = AccelerationSource::NONE;
};

} // namespace nura::flight
