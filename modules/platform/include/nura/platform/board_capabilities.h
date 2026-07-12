#pragma once

#include <cstddef>
#include <cstdint>

namespace nura::platform
{

struct BoardCapabilities
{
    const char *board_name = "unknown";
    uint32_t cpu_frequency_hz = 0U;
    size_t ram_bytes = 0U;
    size_t internal_flash_bytes = 0U;
    size_t external_flash_bytes = 0U;
    size_t log_storage_bytes = 0U;
    size_t trace_capacity_records = 0U;
    uint8_t recovery_channel_count = 0U;
    bool has_sd_storage = false;
    bool has_hardware_watchdog = false;
};

} // namespace nura::platform
