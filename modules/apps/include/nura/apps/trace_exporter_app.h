#pragma once

#include <array>

#include "nura/core/build_config.h"
#include "nura/core/executive.h"
#include "nura/core/trace_map.h"
#include "nura/platform/ports.h"

namespace nura::apps
{

class TraceExporterApp final : public nura::core::IExecutableApp
{
public:
    TraceExporterApp(nura::core::SystemTraceMap &trace_map,
                     nura::platform::ITraceSink &sink)
        : trace_map_(trace_map), sink_(sink) {}

    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
    uint32_t exportGapCount() const { return export_gap_count_; }
    uint32_t sinkFailureCount() const { return sink_failure_count_; }

private:
    nura::core::SystemTraceMap &trace_map_;
    nura::platform::ITraceSink &sink_;
    std::array<nura::contracts::TraceRecord,
               NURA_TRACE_EXPORT_BATCH_RECORDS> batch_{};
    uint32_t last_sequence_ = 0U;
    uint32_t export_gap_count_ = 0U;
    uint32_t sink_failure_count_ = 0U;
};

} // namespace nura::apps
