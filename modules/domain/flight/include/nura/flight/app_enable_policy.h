#pragma once

#include "nura/config/runtime_profile.h"

namespace nura::flight
{

inline constexpr uint64_t globalApplicationMask()
{
    return nura::config::enabledApplicationMask(
        nura::contracts::FlightState::INIT);
}

inline constexpr uint64_t enabledApplications(nura::contracts::FlightState state)
{
    return nura::config::enabledApplicationMask(state);
}

} // namespace nura::flight
