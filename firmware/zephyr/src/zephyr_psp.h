#pragma once

#include <zephyr/kernel.h>

#include "nura/core/monotonic_clock.h"
#include "nura/platform/ports.h"

namespace nura::platform::zephyr
{

class ZephyrClock final : public nura::core::IMonotonicClock
{
public:
    uint64_t nowUs() const override
    {
        return k_ticks_to_us_floor64(k_uptime_ticks());
    }
};

class ZephyrPsp final : public ILowGImu,
                        public IHighGImu,
                        public IBarometer,
                        public IMagnetometer,
                        public IGnss,
                        public IPowerMonitor,
                        public ISafetyInput,
                        public IRecoveryOutput
{
public:
    PortResult initialize() override;
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

private:
    PortResult configureRecoveryOutputs();
    bool recovery_ready_ = false;
};

class ZephyrTraceSink final : public ITraceSink
{
public:
    PortResult tryWrite(const nura::contracts::TraceRecord &record) override;
};

class ZephyrEventSink final : public IEventSink
{
public:
    PortResult tryWrite(const nura::contracts::TransitionEvent &event) override;
    PortResult tryWrite(const nura::contracts::DecisionTrace &event) override;
    PortResult tryWrite(const nura::contracts::SensorFaultEvent &event) override;
};

} // namespace nura::platform::zephyr
