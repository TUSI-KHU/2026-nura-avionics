#pragma once

#include <cmath>

#include "nura/flight/flight_policy.h"

namespace nura::flight
{

inline bool finite3(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline float accelNormG(float x_g, float y_g, float z_g)
{
    return std::sqrt((x_g * x_g) + (y_g * y_g) + (z_g * z_g));
}

inline float accelNormGFromMps2(float x_mps2, float y_mps2, float z_mps2)
{
    return std::sqrt((x_mps2 * x_mps2) + (y_mps2 * y_mps2) +
                     (z_mps2 * z_mps2)) /
           FlightPolicy::kGravityMps2;
}

} // namespace nura::flight
