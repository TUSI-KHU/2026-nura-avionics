#pragma once

#include <cstdio>
#include <vector>

#include "nura/core/monotonic_clock.h"
#include "nura/platform/ports.h"

namespace nura::platform::host
{

class ManualClock final : public nura::core::IMonotonicClock
{
public:
    uint64_t nowUs() const override { return now_us_++; }
    void setNowMs(uint32_t now_ms) { now_us_ = static_cast<uint64_t>(now_ms) * 1000U; }

private:
    mutable uint64_t now_us_ = 0U;
};

struct RecoveryEvent
{
    uint32_t order = 0U;
    nura::contracts::ActuationOperation operation =
        nura::contracts::ActuationOperation::ALL_OFF;
    nura::contracts::RecoveryChannel channel =
        nura::contracts::RecoveryChannel::DROGUE_PRIMARY;
    bool enabled = false;
};

class FakeFlightPlatform final : public ILowGImu,
                                 public IHighGImu,
                                 public IBarometer,
                                 public IMagnetometer,
                                 public IGnss,
                                 public IPowerMonitor,
                                 public ISafetyInput,
                                 public IRecoveryOutput
{
public:
    PortResult initialize() override { return PortResult::OK; }
    PortResult read(uint32_t now_ms,
                    nura::contracts::LowGImuSample &sample) override;
    PortResult read(uint32_t now_ms,
                    nura::contracts::HighGImuSample &sample) override;
    PortResult read(uint32_t now_ms,
                    nura::contracts::BarometerSample &sample) override;
    PortResult read(uint32_t now_ms,
                    nura::contracts::MagnetometerSample &sample) override;
    PortResult read(uint32_t now_ms,
                    nura::contracts::GnssSample &sample) override;
    PortResult read(uint32_t now_ms,
                    nura::contracts::PowerSample &sample) override;
    PortResult read(uint32_t now_ms,
                    nura::contracts::SafetyStatus &status) override;
    PortResult allOff() override;
    PortResult setChannel(nura::contracts::RecoveryChannel channel,
                          bool enabled) override;

    void setAbortActive(bool active) { abort_active_ = active; }
    void setRecoveryFailure(bool fail) { recovery_failure_ = fail; }
    const std::vector<RecoveryEvent> &recoveryEvents() const { return recovery_events_; }
    bool drogueEnabled() const { return drogue_enabled_; }
    bool mainEnabled() const { return main_enabled_; }
    static float altitudeAt(uint32_t now_ms);

private:
    void record(nura::contracts::ActuationOperation operation,
                nura::contracts::RecoveryChannel channel, bool enabled);

    bool abort_active_ = false;
    bool recovery_failure_ = false;
    bool drogue_enabled_ = false;
    bool main_enabled_ = false;
    uint32_t recovery_order_ = 0U;
    std::vector<RecoveryEvent> recovery_events_{};
};

class VectorTraceSink final : public ITraceSink
{
public:
    PortResult tryWrite(const nura::contracts::TraceRecord &record) override
    {
        records_.push_back(record);
        return PortResult::OK;
    }
    const std::vector<nura::contracts::TraceRecord> &records() const { return records_; }

private:
    std::vector<nura::contracts::TraceRecord> records_{};
};

class VectorEventSink final : public IEventSink
{
public:
    PortResult tryWrite(const nura::contracts::TransitionEvent &event) override
    {
        transitions_.push_back(event);
        return PortResult::OK;
    }
    PortResult tryWrite(const nura::contracts::DecisionTrace &event) override
    {
        decisions_.push_back(event);
        return PortResult::OK;
    }
    PortResult tryWrite(const nura::contracts::SensorFaultEvent &event) override
    {
        faults_.push_back(event);
        return PortResult::OK;
    }
    const std::vector<nura::contracts::TransitionEvent> &transitions() const
    {
        return transitions_;
    }
    const std::vector<nura::contracts::DecisionTrace> &decisions() const
    {
        return decisions_;
    }

private:
    std::vector<nura::contracts::TransitionEvent> transitions_{};
    std::vector<nura::contracts::DecisionTrace> decisions_{};
    std::vector<nura::contracts::SensorFaultEvent> faults_{};
};

class CsvTraceSink final : public ITraceSink
{
public:
    explicit CsvTraceSink(const char *path);
    ~CsvTraceSink() override;
    CsvTraceSink(const CsvTraceSink &) = delete;
    CsvTraceSink &operator=(const CsvTraceSink &) = delete;
    bool valid() const { return file_ != nullptr; }
    PortResult tryWrite(const nura::contracts::TraceRecord &record) override;

private:
    std::FILE *file_ = nullptr;
};

} // namespace nura::platform::host
