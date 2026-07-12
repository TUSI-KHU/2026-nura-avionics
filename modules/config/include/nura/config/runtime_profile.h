#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "nura/contracts/common.h"
#include "nura/contracts/flight_topics.h"

namespace nura::config
{

enum class ExecutionDomain : uint8_t
{
    RECOVERY,
    MISSION,
    CRITICAL_SENSOR,
    OPTIONAL_SENSOR,
    SUPERVISOR,
    EVENT_RECORDER,
    TRACE_EXPORTER,
};

struct AppDescriptor
{
    nura::contracts::AppId id;
    ExecutionDomain domain;
    uint32_t period_ms;
    uint32_t deadline_us;
    uint16_t stack_bytes;
    int8_t priority;
    uint16_t allowed_states;
    bool independently_scheduled;
};

inline constexpr uint16_t stateBit(nura::contracts::FlightState state)
{
    return static_cast<uint16_t>(1U << static_cast<uint8_t>(state));
}

inline constexpr uint16_t kAllStatesMask =
    static_cast<uint16_t>((1U << 10U) - 1U);

inline constexpr uint16_t onlyState(nura::contracts::FlightState state)
{
    return stateBit(state);
}

// Single source of truth for every implemented application's scheduling and
// state-enable policy. State applications share the mission thread and are
// dispatched by FlightCoordinator; all other scheduled entries own a thread.
inline constexpr std::array<AppDescriptor, 19U> kAppCatalog{{
    {nura::contracts::AppId::LOW_G_SENSOR, ExecutionDomain::CRITICAL_SENSOR,
     10U, 1000U, 2048U, 2, kAllStatesMask, true},
    {nura::contracts::AppId::HIGH_G_SENSOR, ExecutionDomain::CRITICAL_SENSOR,
     10U, 1000U, 2048U, 2, kAllStatesMask, true},
    {nura::contracts::AppId::BAROMETER_SENSOR, ExecutionDomain::CRITICAL_SENSOR,
     50U, 1500U, 2048U, 2, kAllStatesMask, true},
    {nura::contracts::AppId::MAGNETOMETER_SENSOR, ExecutionDomain::OPTIONAL_SENSOR,
     100U, 1000U, 2048U, 3, kAllStatesMask, true},
    {nura::contracts::AppId::GNSS_SENSOR, ExecutionDomain::OPTIONAL_SENSOR,
     50U, 1000U, 2048U, 3, kAllStatesMask, true},
    {nura::contracts::AppId::POWER_SENSOR, ExecutionDomain::OPTIONAL_SENSOR,
     100U, 1000U, 2048U, 3, kAllStatesMask, true},
    {nura::contracts::AppId::SAFETY_INPUT, ExecutionDomain::CRITICAL_SENSOR,
     10U, 500U, 1536U, 2, kAllStatesMask, true},
    {nura::contracts::AppId::INPUT_AGGREGATOR, ExecutionDomain::MISSION,
     10U, 1500U, 0U, 1, kAllStatesMask, false},
    {nura::contracts::AppId::FLIGHT_COORDINATOR, ExecutionDomain::MISSION,
     10U, 3000U, 4096U, 1, kAllStatesMask, true},
    {nura::contracts::AppId::LAUNCH_DETECTOR, ExecutionDomain::MISSION,
     10U, 0U, 0U, 1, onlyState(nura::contracts::FlightState::ARMED), false},
    {nura::contracts::AppId::BURNOUT_DETECTOR, ExecutionDomain::MISSION,
     10U, 0U, 0U, 1, onlyState(nura::contracts::FlightState::LAUNCH), false},
    {nura::contracts::AppId::APOGEE_DETECTOR, ExecutionDomain::MISSION,
     10U, 0U, 0U, 1, onlyState(nura::contracts::FlightState::COAST), false},
    {nura::contracts::AppId::DROGUE_SEQUENCE, ExecutionDomain::MISSION,
     10U, 0U, 0U, 1, onlyState(nura::contracts::FlightState::APOGEE), false},
    {nura::contracts::AppId::MAIN_DEPLOY_DETECTOR, ExecutionDomain::MISSION,
     10U, 0U, 0U, 1, onlyState(nura::contracts::FlightState::DROGUE), false},
    {nura::contracts::AppId::LANDING_SEQUENCE, ExecutionDomain::MISSION,
     10U, 0U, 0U, 1, onlyState(nura::contracts::FlightState::DEPLOY), false},
    {nura::contracts::AppId::RECOVERY_ACTUATION, ExecutionDomain::RECOVERY,
     2U, 1000U, 3072U, 0, kAllStatesMask, true},
    {nura::contracts::AppId::EVENT_RECORDER, ExecutionDomain::EVENT_RECORDER,
     20U, 2000U, 3072U, 8, kAllStatesMask, true},
    {nura::contracts::AppId::SUPERVISOR, ExecutionDomain::SUPERVISOR,
     50U, 1000U, 3072U, 4, kAllStatesMask, true},
    {nura::contracts::AppId::TRACE_EXPORTER, ExecutionDomain::TRACE_EXPORTER,
     20U, 0U, 3072U, 9, kAllStatesMask, true},
}};

inline constexpr const AppDescriptor *appDescriptor(nura::contracts::AppId id)
{
    for (const AppDescriptor &descriptor : kAppCatalog)
    {
        if (descriptor.id == id)
        {
            return &descriptor;
        }
    }
    return nullptr;
}

inline constexpr bool isApplicationEnabled(nura::contracts::AppId id,
                                           nura::contracts::FlightState state)
{
    const AppDescriptor *descriptor = appDescriptor(id);
    return descriptor != nullptr &&
           (descriptor->allowed_states & stateBit(state)) != 0U;
}

inline constexpr uint64_t enabledApplicationMask(
    nura::contracts::FlightState state)
{
    uint64_t mask = 0U;
    for (const AppDescriptor &descriptor : kAppCatalog)
    {
        if (isApplicationEnabled(descriptor.id, state))
        {
            mask |= nura::contracts::appBit(descriptor.id);
        }
    }
    return mask;
}

inline constexpr bool appCatalogValid()
{
    for (size_t i = 0U; i < kAppCatalog.size(); ++i)
    {
        const AppDescriptor &descriptor = kAppCatalog[i];
        if (descriptor.independently_scheduled &&
            (descriptor.period_ms == 0U || descriptor.stack_bytes == 0U))
        {
            return false;
        }
        for (size_t j = i + 1U; j < kAppCatalog.size(); ++j)
        {
            if (descriptor.id == kAppCatalog[j].id)
            {
                return false;
            }
        }
    }
    return true;
}

struct RuntimeProfile
{
    uint32_t sensor_health_max_age_us = 250000U;
    uint32_t critical_health_max_age_us = 100000U;
    uint32_t watchdog_timeout_ms = 1000U;
    uint8_t recovery_drain_limit = 8U;
    uint8_t event_drain_limit = 16U;
};

inline constexpr RuntimeProfile kRuntimeProfile{};
inline constexpr uint32_t kRuntimeProfileRevision = 1U;

static_assert(appCatalogValid(),
              "App Catalog IDs and scheduled resources must be valid");
static_assert(appDescriptor(nura::contracts::AppId::RECOVERY_ACTUATION)->priority <
                  appDescriptor(nura::contracts::AppId::FLIGHT_COORDINATOR)->priority,
              "recovery output handling must outrank mission computation");

} // namespace nura::config
