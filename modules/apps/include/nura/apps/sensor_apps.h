#pragma once

#include <cstdint>

#include "nura/core/executive.h"
#include "nura/core/monotonic_clock.h"
#include "nura/core/software_bus.h"
#include "nura/platform/ports.h"

namespace nura::apps
{

class LowGSensorApp final : public nura::core::IExecutableApp
{
public:
    LowGSensorApp(nura::platform::ILowGImu &port, nura::core::ILowGPublisher &bus,
                  nura::core::IMonotonicClock &clock)
        : port_(port), bus_(bus), clock_(clock) {}
    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
private:
    nura::platform::ILowGImu &port_;
    nura::core::ILowGPublisher &bus_;
    nura::core::IMonotonicClock &clock_;
    uint32_t sequence_ = 0U;
    bool initialized_ = false;
};

class HighGSensorApp final : public nura::core::IExecutableApp
{
public:
    HighGSensorApp(nura::platform::IHighGImu &port, nura::core::IHighGPublisher &bus,
                   nura::core::IMonotonicClock &clock)
        : port_(port), bus_(bus), clock_(clock) {}
    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
private:
    nura::platform::IHighGImu &port_;
    nura::core::IHighGPublisher &bus_;
    nura::core::IMonotonicClock &clock_;
    uint32_t sequence_ = 0U;
    bool initialized_ = false;
};

class BarometerSensorApp final : public nura::core::IExecutableApp
{
public:
    BarometerSensorApp(nura::platform::IBarometer &port,
                       nura::core::IBarometerPublisher &bus,
                       nura::core::IMonotonicClock &clock)
        : port_(port), bus_(bus), clock_(clock) {}
    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
private:
    nura::platform::IBarometer &port_;
    nura::core::IBarometerPublisher &bus_;
    nura::core::IMonotonicClock &clock_;
    uint32_t sequence_ = 0U;
    bool initialized_ = false;
};

class MagnetometerSensorApp final : public nura::core::IExecutableApp
{
public:
    MagnetometerSensorApp(nura::platform::IMagnetometer &port,
                          nura::core::IMagnetometerPublisher &bus,
                          nura::core::IMonotonicClock &clock)
        : port_(port), bus_(bus), clock_(clock) {}
    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
private:
    nura::platform::IMagnetometer &port_;
    nura::core::IMagnetometerPublisher &bus_;
    nura::core::IMonotonicClock &clock_;
    uint32_t sequence_ = 0U;
    bool initialized_ = false;
};

class GnssSensorApp final : public nura::core::IExecutableApp
{
public:
    GnssSensorApp(nura::platform::IGnss &port, nura::core::IGnssPublisher &bus,
                  nura::core::IMonotonicClock &clock)
        : port_(port), bus_(bus), clock_(clock) {}
    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
private:
    nura::platform::IGnss &port_;
    nura::core::IGnssPublisher &bus_;
    nura::core::IMonotonicClock &clock_;
    uint32_t sequence_ = 0U;
    bool initialized_ = false;
};

class PowerSensorApp final : public nura::core::IExecutableApp
{
public:
    PowerSensorApp(nura::platform::IPowerMonitor &port,
                   nura::core::IPowerPublisher &bus,
                   nura::core::IMonotonicClock &clock)
        : port_(port), bus_(bus), clock_(clock) {}
    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
private:
    nura::platform::IPowerMonitor &port_;
    nura::core::IPowerPublisher &bus_;
    nura::core::IMonotonicClock &clock_;
    uint32_t sequence_ = 0U;
    bool initialized_ = false;
};

class SafetyInputApp final : public nura::core::IExecutableApp
{
public:
    SafetyInputApp(nura::platform::ISafetyInput &port,
                   nura::core::ISafetyPublisher &bus,
                   nura::core::IMonotonicClock &clock)
        : port_(port), bus_(bus), clock_(clock) {}
    nura::contracts::AppId id() const override;
    nura::core::AppRunResult run(uint32_t now_ms, uint32_t cycle_id) override;
private:
    nura::platform::ISafetyInput &port_;
    nura::core::ISafetyPublisher &bus_;
    nura::core::IMonotonicClock &clock_;
    uint32_t sequence_ = 0U;
    bool initialized_ = false;
};

} // namespace nura::apps
