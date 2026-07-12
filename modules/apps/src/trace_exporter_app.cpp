#include "nura/apps/trace_exporter_app.h"

namespace nura::apps
{
namespace c = nura::contracts;
namespace core = nura::core;
namespace p = nura::platform;

c::AppId TraceExporterApp::id() const { return c::AppId::TRACE_EXPORTER; }

core::AppRunResult TraceExporterApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    (void)now_ms;
    (void)cycle_id;
    uint32_t gap = 0U;
    const size_t count = trace_map_.tryReadAfter(last_sequence_, batch_.data(),
                                                batch_.size(), gap);
    export_gap_count_ += gap;
    if (count == 0U)
    {
        return core::AppRunResult::NO_INPUT;
    }

    for (size_t i = 0U; i < count; ++i)
    {
        if (sink_.tryWrite(batch_[i]) != p::PortResult::OK)
        {
            ++sink_failure_count_;
            return core::AppRunResult::DEGRADED;
        }
        last_sequence_ = batch_[i].sequence;
    }
    return gap == 0U ? core::AppRunResult::OK : core::AppRunResult::DEGRADED;
}

} // namespace nura::apps
