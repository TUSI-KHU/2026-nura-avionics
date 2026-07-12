#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace nura::contracts
{

constexpr uint32_t kContractSchemaVersion = 2U;

enum class AppId : uint16_t
{
    UNKNOWN = 0U,
    LOW_G_SENSOR = 1U,
    HIGH_G_SENSOR = 2U,
    BAROMETER_SENSOR = 3U,
    MAGNETOMETER_SENSOR = 4U,
    GNSS_SENSOR = 5U,
    POWER_SENSOR = 6U,
    SAFETY_INPUT = 7U,
    INPUT_AGGREGATOR = 10U,
    FLIGHT_COORDINATOR = 11U,
    LAUNCH_DETECTOR = 12U,
    BURNOUT_DETECTOR = 13U,
    APOGEE_DETECTOR = 14U,
    DROGUE_SEQUENCE = 15U,
    MAIN_DEPLOY_DETECTOR = 16U,
    LANDING_SEQUENCE = 17U,
    RECOVERY_ACTUATION = 20U,
    ANNUNCIATION = 21U,
    TELEMETRY = 30U,
    FLIGHT_RECORDER = 31U,
    EVENT_RECORDER = 32U,
    SUPERVISOR = 33U,
    TRACE_EXPORTER = 34U,
    SYSTEM = 255U,
};

enum class TopicId : uint16_t
{
    NONE = 0U,
    LOW_G_LATEST = 1U,
    HIGH_G_LATEST = 2U,
    BAROMETER_LATEST = 3U,
    MAGNETOMETER_LATEST = 4U,
    GNSS_LATEST = 5U,
    POWER_LATEST = 6U,
    SENSOR_HEALTH_LATEST = 7U,
    SAFETY_STATUS_LATEST = 8U,
    FLIGHT_INPUTS_LATEST = 10U,
    FLIGHT_STATUS_LATEST = 11U,
    APP_ENABLE_LATEST = 12U,
    COMMAND_REQUEST_QUEUE = 20U,
    ACTUATION_INTENT_QUEUE = 21U,
    TRANSITION_EVENT_QUEUE = 22U,
    DECISION_EVENT_QUEUE = 23U,
    SENSOR_FAULT_QUEUE = 24U,
    TRACE_STREAM = 30U,
};

enum SampleStatusFlag : uint32_t
{
    SAMPLE_STATUS_NONE = 0U,
    SAMPLE_STATUS_VALID = 1U << 0,
    SAMPLE_STATUS_REFERENCE_VALID = 1U << 1,
    SAMPLE_STATUS_CONNECTED = 1U << 2,
    SAMPLE_STATUS_NEW_DATA = 1U << 3,
    SAMPLE_STATUS_DEGRADED = 1U << 4,
    SAMPLE_STATUS_FAULT = 1U << 5,
    SAMPLE_STATUS_STALE = 1U << 6,
};

struct SampleHeader
{
    uint32_t schema_version = kContractSchemaVersion;
    uint32_t sequence = 0U;
    uint32_t sample_time_ms = 0U;
    uint64_t publish_time_us = 0U;
    uint32_t status_flags = SAMPLE_STATUS_NONE;
    AppId producer = AppId::UNKNOWN;
};

template <typename T>
constexpr bool is_bus_payload_v = std::is_trivially_copyable<T>::value &&
                                  std::is_standard_layout<T>::value;

template <typename T, size_t MaxBytes>
constexpr bool payload_fits_v = is_bus_payload_v<T> && sizeof(T) <= MaxBytes;

inline constexpr uint64_t appBit(AppId id)
{
    const uint16_t value = static_cast<uint16_t>(id);
    return value < 64U ? (1ULL << value) : 0ULL;
}

} // namespace nura::contracts
