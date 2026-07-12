#pragma once

#include <stdint.h>

#include "core/logger/logger.h"
#include "core/tasks.h"
#include "logging/flight_log_ram_buffer.h"
#include "logging/flight_log_record.h"
#include "logging/flight_log_storage.h"
#include "state/flight_state.h"
#include "state/gps_state.h"
#include "state/high_g_imu_state.h"
#include "state/imu_state.h"
#include "state/magnetometer_state.h"
#include "state/telemetry_state.h"

class FlightLogTask : public Task
{
public:
    FlightLogTask(FlightState &flightState,
                  const ImuState &imuState,
                  const HighGImuState &highGImuState,
                  const MagnetometerState &magnetometerState,
                  const GpsState &gpsState,
                  TelemetryState &telemetryState,
                  IFlightLogStorage &storage,
                  Logger &logger);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    bool enqueueFastSample(uint32_t nowMs);
    bool enqueueSlowSample(uint32_t nowMs);
    bool enqueueDecision(const FlightDecisionTrace &decision);
    bool enqueueEvent(nura_log::EventId eventId,
                      uint32_t nowMs,
                      State previousState,
                      State currentState,
                      uint32_t data0,
                      uint32_t data1);
    bool enqueueFrame(nura_log::RecordType type,
                      uint32_t timestampMs,
                      const void *payload,
                      uint16_t payloadLength);
    void drainToStorage(uint32_t nowMs, uint16_t maxRecords);
    uint8_t fastFlags() const;
    uint8_t gpsFlags() const;
    uint16_t healthFlags() const;
    void stopAtGround(uint32_t nowMs);

    static int16_t scaledFloatToI16(float value, float scale);
    static int32_t scaledFloatToI32(float value, float scale);
    static int32_t scaledDoubleToI32(double value, double scale);
    static uint16_t scaledDoubleToU16(double value, double scale);

    FlightState &flightState_;
    const ImuState &imuState_;
    const HighGImuState &highGImuState_;
    const MagnetometerState &magnetometerState_;
    const GpsState &gpsState_;
    TelemetryState &telemetryState_;
    IFlightLogStorage &storage_;
    Logger &logger_;

    FlightLogRamBuffer ramBuffer_;
    uint8_t scratch_[nura_log::kMaxEncodedFrameBytes] = {};
    uint32_t nextSequence_ = 0U;
    uint32_t lastFastSampleMs_ = 0U;
    uint32_t lastSlowSampleMs_ = 0U;
    uint32_t lastDroppedRecords_ = 0U;
    uint32_t lastDroppedDecisionTraces_ = 0U;
    State lastState_ = State::INIT;
    bool storageStarted_ = false;
    bool stopped_ = false;
};
