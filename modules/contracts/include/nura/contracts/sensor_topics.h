#pragma once

#include <cstdint>

#include "nura/contracts/common.h"

namespace nura::contracts
{

enum BarometerFaultFlag : uint16_t
{
    BAROMETER_FAULT_NONE = 0U,
    BAROMETER_FAULT_READ_FAIL = 1U << 0,
    BAROMETER_FAULT_STALE = 1U << 1,
    BAROMETER_FAULT_BAD_VALUE = 1U << 2,
    BAROMETER_FAULT_STUCK = 1U << 3,
};

struct LowGImuSample
{
    SampleHeader header{};
    float accel_x_mps2 = 0.0f;
    float accel_y_mps2 = 0.0f;
    float accel_z_mps2 = 0.0f;
    float gyro_x_dps = 0.0f;
    float gyro_y_dps = 0.0f;
    float gyro_z_dps = 0.0f;
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    float tilt_angle_deg = 0.0f;
    bool attitude_valid = false;
    bool tilt_valid = false;
};

struct HighGImuSample
{
    SampleHeader header{};
    float accel_x_g = 0.0f;
    float accel_y_g = 0.0f;
    float accel_z_g = 0.0f;
};

struct BarometerSample
{
    SampleHeader header{};
    float pressure_pa = 0.0f;
    float reference_pressure_pa = 0.0f;
    float raw_altitude_m = 0.0f;
    float filtered_altitude_m = 0.0f;
    uint16_t fault_flags = BAROMETER_FAULT_NONE;
    uint8_t consecutive_read_fail_count = 0U;
    uint8_t consecutive_bad_value_count = 0U;
    uint8_t total_bad_value_count = 0U;
};

struct MagnetometerSample
{
    SampleHeader header{};
    float magnetic_x_ut = 0.0f;
    float magnetic_y_ut = 0.0f;
    float magnetic_z_ut = 0.0f;
};

struct GnssSample
{
    SampleHeader header{};
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    double speed_mps = 0.0;
    double course_deg = 0.0;
    double hdop = 0.0;
    uint32_t location_age_ms = 0U;
    uint8_t satellites = 0U;
    bool has_fix = false;
};

struct PowerSample
{
    SampleHeader header{};
    uint16_t battery_mv = 0U;
};

struct SensorHealthSnapshot
{
    SampleHeader header{};
    bool low_g_ok = false;
    bool high_accel_ok = false;
    bool barometer_ok = false;
    bool magnetometer_ok = false;
    bool gnss_ok = false;
    bool power_ok = false;
    bool safety_input_ok = false;
    bool storage_ok = false;
    bool pyro_continuity_ok = false;
};

static_assert(payload_fits_v<LowGImuSample, 192U>, "low-g topic exceeds copy budget");
static_assert(payload_fits_v<HighGImuSample, 96U>, "high-g topic exceeds copy budget");
static_assert(payload_fits_v<BarometerSample, 128U>, "barometer topic exceeds copy budget");
static_assert(payload_fits_v<GnssSample, 160U>, "GNSS topic exceeds copy budget");

} // namespace nura::contracts
