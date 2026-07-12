#pragma once

#include <cstdint>

#include "nura/contracts/flight_topics.h"
#include "nura/contracts/sensor_topics.h"
#include "nura/contracts/trace_topics.h"
#include "nura/platform/board_capabilities.h"

namespace nura::platform
{

// Sensor initialize/read implementations are called from bounded periodic
// execution domains. They must return within the App Catalog deadline and may
// report NO_DATA while an asynchronous conversion is pending. Drivers must not
// sleep, retry indefinitely, write logs, or wait on another application here.

enum class PortResult : int32_t
{
    OK = 0,
    NO_DATA = 1,
    UNAVAILABLE = 2,
    FAULT = -1,
};

class ILowGImu
{
public:
    virtual ~ILowGImu() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult read(uint32_t now_ms,
                            nura::contracts::LowGImuSample &sample) = 0;
};

class IHighGImu
{
public:
    virtual ~IHighGImu() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult read(uint32_t now_ms,
                            nura::contracts::HighGImuSample &sample) = 0;
};

class IBarometer
{
public:
    virtual ~IBarometer() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult read(uint32_t now_ms,
                            nura::contracts::BarometerSample &sample) = 0;
};

class IMagnetometer
{
public:
    virtual ~IMagnetometer() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult read(uint32_t now_ms,
                            nura::contracts::MagnetometerSample &sample) = 0;
};

class IGnss
{
public:
    virtual ~IGnss() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult read(uint32_t now_ms,
                            nura::contracts::GnssSample &sample) = 0;
};

class IPowerMonitor
{
public:
    virtual ~IPowerMonitor() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult read(uint32_t now_ms,
                            nura::contracts::PowerSample &sample) = 0;
};

class ISafetyInput
{
public:
    virtual ~ISafetyInput() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult read(uint32_t now_ms,
                            nura::contracts::SafetyStatus &status) = 0;
};

class IRecoveryOutput
{
public:
    virtual ~IRecoveryOutput() = default;
    virtual PortResult initialize() = 0;
    virtual PortResult allOff() = 0;
    virtual PortResult setChannel(nura::contracts::RecoveryChannel channel,
                                  bool enabled) = 0;
};

// Implementations must return promptly. Slow filesystem/UART work belongs in
// the low-priority exporter execution domain, never in TraceMap::tryRecord().
class ITraceSink
{
public:
    virtual ~ITraceSink() = default;
    virtual PortResult tryWrite(const nura::contracts::TraceRecord &record) = 0;
};

class IEventSink
{
public:
    virtual ~IEventSink() = default;
    virtual PortResult tryWrite(const nura::contracts::TransitionEvent &event) = 0;
    virtual PortResult tryWrite(const nura::contracts::DecisionTrace &event) = 0;
    virtual PortResult tryWrite(const nura::contracts::SensorFaultEvent &event) = 0;
};

class IWatchdog
{
public:
    virtual ~IWatchdog() = default;
    virtual PortResult initialize(uint32_t timeout_ms) = 0;
    virtual PortResult feed() = 0;
};

struct PlatformServices
{
    const BoardCapabilities *capabilities = nullptr;
    ILowGImu *low_g = nullptr;
    IHighGImu *high_g = nullptr;
    IBarometer *barometer = nullptr;
    IMagnetometer *magnetometer = nullptr;
    IGnss *gnss = nullptr;
    IPowerMonitor *power = nullptr;
    ISafetyInput *safety = nullptr;
    IRecoveryOutput *recovery = nullptr;
    ITraceSink *trace_sink = nullptr;
    IEventSink *event_sink = nullptr;
    IWatchdog *watchdog = nullptr;
};

} // namespace nura::platform
