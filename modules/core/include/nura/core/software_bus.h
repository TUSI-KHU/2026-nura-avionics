#pragma once

#include <cstdint>

#include "nura/contracts/flight_topics.h"
#include "nura/contracts/sensor_topics.h"
#include "nura/core/build_config.h"
#include "nura/core/latest_topic.h"
#include "nura/core/spsc_queue.h"
#include "nura/core/trace_map.h"

namespace nura::core
{

struct BusQueueMetrics
{
    uint32_t command_high_water = 0U;
    uint32_t command_dropped = 0U;
    uint32_t actuation_high_water = 0U;
    uint32_t actuation_dropped = 0U;
    uint32_t transition_high_water = 0U;
    uint32_t transition_dropped = 0U;
    uint32_t decision_high_water = 0U;
    uint32_t decision_dropped = 0U;
    uint32_t sensor_fault_high_water = 0U;
    uint32_t sensor_fault_dropped = 0U;
};

struct BusLatestMetrics
{
    uint32_t publish_contention = 0U;
    uint32_t read_contention = 0U;
};

class ILowGPublisher
{
public:
    virtual ~ILowGPublisher() = default;
    virtual bool publishLowG(const nura::contracts::LowGImuSample &, uint32_t) = 0;
};
class IHighGPublisher
{
public:
    virtual ~IHighGPublisher() = default;
    virtual bool publishHighG(const nura::contracts::HighGImuSample &, uint32_t) = 0;
};
class IBarometerPublisher
{
public:
    virtual ~IBarometerPublisher() = default;
    virtual bool publishBarometer(const nura::contracts::BarometerSample &, uint32_t) = 0;
};
class IMagnetometerPublisher
{
public:
    virtual ~IMagnetometerPublisher() = default;
    virtual bool publishMagnetometer(const nura::contracts::MagnetometerSample &, uint32_t) = 0;
};
class IGnssPublisher
{
public:
    virtual ~IGnssPublisher() = default;
    virtual bool publishGnss(const nura::contracts::GnssSample &, uint32_t) = 0;
};
class IPowerPublisher
{
public:
    virtual ~IPowerPublisher() = default;
    virtual bool publishPower(const nura::contracts::PowerSample &, uint32_t) = 0;
};
class ISafetyPublisher
{
public:
    virtual ~ISafetyPublisher() = default;
    virtual bool publishSafetyStatus(const nura::contracts::SafetyStatus &, uint32_t) = 0;
};

class IInputAggregationBus
{
public:
    virtual ~IInputAggregationBus() = default;
    virtual bool readLowG(nura::contracts::LowGImuSample &, nura::contracts::AppId,
                          uint32_t, uint64_t) = 0;
    virtual bool readHighG(nura::contracts::HighGImuSample &, nura::contracts::AppId,
                           uint32_t, uint64_t) = 0;
    virtual bool readBarometer(nura::contracts::BarometerSample &,
                               nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool readSensorHealth(nura::contracts::SensorHealthSnapshot &,
                                  nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool readSafetyStatus(nura::contracts::SafetyStatus &,
                                  nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool publishFlightInputs(const nura::contracts::FlightInputs &,
                                     uint32_t) = 0;
};

class IFlightCoordinatorBus
{
public:
    virtual ~IFlightCoordinatorBus() = default;
    virtual bool readFlightInputs(nura::contracts::FlightInputs &,
                                  nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool popCommand(nura::contracts::CommandRequest &,
                            nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool publishFlightStatus(const nura::contracts::FlightStatus &,
                                     uint32_t) = 0;
    virtual bool publishAppEnable(const nura::contracts::AppEnableSet &, uint32_t) = 0;
    virtual bool pushActuation(const nura::contracts::ActuationIntent &,
                               uint32_t, uint64_t) = 0;
    virtual bool pushTransition(const nura::contracts::TransitionEvent &,
                                uint32_t, uint64_t) = 0;
    virtual bool pushDecision(const nura::contracts::DecisionTrace &,
                              uint32_t, uint64_t) = 0;
};

class IRecoveryBus
{
public:
    virtual ~IRecoveryBus() = default;
    virtual bool readFlightStatus(nura::contracts::FlightStatus &,
                                  nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool popActuation(nura::contracts::ActuationIntent &,
                              nura::contracts::AppId, uint32_t, uint64_t) = 0;
};

class IEventRecorderBus
{
public:
    virtual ~IEventRecorderBus() = default;
    virtual bool popTransition(nura::contracts::TransitionEvent &,
                               nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool popDecision(nura::contracts::DecisionTrace &,
                             nura::contracts::AppId, uint32_t, uint64_t) = 0;
    virtual bool popSensorFault(nura::contracts::SensorFaultEvent &,
                                nura::contracts::AppId, uint32_t, uint64_t) = 0;
};

class ISupervisorBus
{
public:
    virtual ~ISupervisorBus() = default;
    virtual BusQueueMetrics queueMetrics() const = 0;
    virtual bool publishSensorHealth(
        const nura::contracts::SensorHealthSnapshot &, uint32_t) = 0;
};

class SoftwareBus final : public ILowGPublisher,
                          public IHighGPublisher,
                          public IBarometerPublisher,
                          public IMagnetometerPublisher,
                          public IGnssPublisher,
                          public IPowerPublisher,
                          public ISafetyPublisher,
                          public IInputAggregationBus,
                          public IFlightCoordinatorBus,
                          public IRecoveryBus,
                          public IEventRecorderBus,
                          public ISupervisorBus
{
public:
    explicit SoftwareBus(SystemTraceMap &trace_map) : trace_map_(trace_map) {}

    bool publishLowG(const nura::contracts::LowGImuSample &value, uint32_t cycle_id);
    bool readLowG(nura::contracts::LowGImuSample &value, nura::contracts::AppId consumer,
                  uint32_t cycle_id, uint64_t now_us);
    bool publishHighG(const nura::contracts::HighGImuSample &value, uint32_t cycle_id);
    bool readHighG(nura::contracts::HighGImuSample &value, nura::contracts::AppId consumer,
                   uint32_t cycle_id, uint64_t now_us);
    bool publishBarometer(const nura::contracts::BarometerSample &value, uint32_t cycle_id);
    bool readBarometer(nura::contracts::BarometerSample &value,
                       nura::contracts::AppId consumer, uint32_t cycle_id, uint64_t now_us);
    bool publishMagnetometer(const nura::contracts::MagnetometerSample &value,
                             uint32_t cycle_id);
    bool readMagnetometer(nura::contracts::MagnetometerSample &value,
                          nura::contracts::AppId consumer, uint32_t cycle_id,
                          uint64_t now_us);
    bool publishGnss(const nura::contracts::GnssSample &value, uint32_t cycle_id);
    bool readGnss(nura::contracts::GnssSample &value,
                  nura::contracts::AppId consumer, uint32_t cycle_id,
                  uint64_t now_us);
    bool publishPower(const nura::contracts::PowerSample &value, uint32_t cycle_id);
    bool readPower(nura::contracts::PowerSample &value,
                   nura::contracts::AppId consumer, uint32_t cycle_id,
                   uint64_t now_us);
    bool publishSensorHealth(const nura::contracts::SensorHealthSnapshot &value,
                             uint32_t cycle_id);
    bool readSensorHealth(nura::contracts::SensorHealthSnapshot &value,
                          nura::contracts::AppId consumer, uint32_t cycle_id,
                          uint64_t now_us);
    bool publishSafetyStatus(const nura::contracts::SafetyStatus &value,
                             uint32_t cycle_id);
    bool readSafetyStatus(nura::contracts::SafetyStatus &value,
                          nura::contracts::AppId consumer, uint32_t cycle_id,
                          uint64_t now_us);
    bool publishFlightInputs(const nura::contracts::FlightInputs &value, uint32_t cycle_id);
    bool readFlightInputs(nura::contracts::FlightInputs &value,
                          nura::contracts::AppId consumer, uint32_t cycle_id,
                          uint64_t now_us);
    bool publishFlightStatus(const nura::contracts::FlightStatus &value, uint32_t cycle_id);
    bool readFlightStatus(nura::contracts::FlightStatus &value,
                          nura::contracts::AppId consumer, uint32_t cycle_id,
                          uint64_t now_us);
    bool publishAppEnable(const nura::contracts::AppEnableSet &value, uint32_t cycle_id);
    bool readAppEnable(nura::contracts::AppEnableSet &value,
                       nura::contracts::AppId consumer, uint32_t cycle_id,
                       uint64_t now_us);

    bool pushCommand(const nura::contracts::CommandRequest &value, uint32_t cycle_id,
                     uint64_t now_us);
    bool popCommand(nura::contracts::CommandRequest &value, nura::contracts::AppId consumer,
                    uint32_t cycle_id, uint64_t now_us);
    bool pushActuation(const nura::contracts::ActuationIntent &value, uint32_t cycle_id,
                       uint64_t now_us);
    bool popActuation(nura::contracts::ActuationIntent &value,
                      nura::contracts::AppId consumer, uint32_t cycle_id, uint64_t now_us);
    bool pushTransition(const nura::contracts::TransitionEvent &value, uint32_t cycle_id,
                        uint64_t now_us);
    bool popTransition(nura::contracts::TransitionEvent &value,
                       nura::contracts::AppId consumer, uint32_t cycle_id, uint64_t now_us);
    bool pushDecision(const nura::contracts::DecisionTrace &value, uint32_t cycle_id,
                      uint64_t now_us);
    bool popDecision(nura::contracts::DecisionTrace &value,
                     nura::contracts::AppId consumer, uint32_t cycle_id, uint64_t now_us);
    bool pushSensorFault(const nura::contracts::SensorFaultEvent &value, uint32_t cycle_id,
                         uint64_t now_us);
    bool popSensorFault(nura::contracts::SensorFaultEvent &value,
                        nura::contracts::AppId consumer, uint32_t cycle_id, uint64_t now_us);

    BusQueueMetrics queueMetrics() const;
    BusLatestMetrics latestMetrics() const;

private:
    void traceBus(nura::contracts::TraceEvent event, nura::contracts::AppId app,
                  nura::contracts::AppId peer, nura::contracts::TopicId topic,
                  uint32_t cycle_id, uint32_t correlation_id, uint64_t now_us,
                  int32_t result, uint32_t detail = 0U);

    SystemTraceMap &trace_map_;
    LatestTopic<nura::contracts::LowGImuSample> low_g_{};
    LatestTopic<nura::contracts::HighGImuSample> high_g_{};
    LatestTopic<nura::contracts::BarometerSample> barometer_{};
    LatestTopic<nura::contracts::MagnetometerSample> magnetometer_{};
    LatestTopic<nura::contracts::GnssSample> gnss_{};
    LatestTopic<nura::contracts::PowerSample> power_{};
    LatestTopic<nura::contracts::SensorHealthSnapshot> sensor_health_{};
    LatestTopic<nura::contracts::SafetyStatus> safety_status_{};
    LatestTopic<nura::contracts::FlightInputs> flight_inputs_{};
    LatestTopic<nura::contracts::FlightStatus> flight_status_{};
    LatestTopic<nura::contracts::AppEnableSet> app_enable_{};
    SpscQueue<nura::contracts::CommandRequest, NURA_COMMAND_QUEUE_SLOTS> command_queue_{};
    SpscQueue<nura::contracts::ActuationIntent, NURA_ACTUATION_QUEUE_SLOTS> actuation_queue_{};
    SpscQueue<nura::contracts::TransitionEvent, NURA_TRANSITION_QUEUE_SLOTS> transition_queue_{};
    SpscQueue<nura::contracts::DecisionTrace, NURA_DECISION_QUEUE_SLOTS> decision_queue_{};
    SpscQueue<nura::contracts::SensorFaultEvent, NURA_SENSOR_FAULT_QUEUE_SLOTS> sensor_fault_queue_{};
};

} // namespace nura::core
