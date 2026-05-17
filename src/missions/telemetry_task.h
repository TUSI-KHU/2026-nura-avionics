#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/tasks.h"
#include "hal/sx127x_lora_hal.h"
#include "nura_protocol_v1_lite.h"
#include "state/abort_state.h"
#include "state/flight_state.h"
#include "state/gps_state.h"
#include "state/imu_state.h"
#include "state/telemetry_state.h"

class TelemetryTask : public Task
{
public:
    TelemetryTask(Sx127xLoRaHAL &radio,
                  const ImuState &imuState,
                  const GpsState &gpsState,
                  TelemetryState &telemetryState,
                  FlightState &flightState,
                  const AbortState &abortState,
                  Logger &logger,
                  const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    struct RecentCommand
    {
        bool valid = false;
        uint8_t commandId = 0U;
        uint16_t commandSeq = 0U;
        uint32_t nonce = 0UL;
    };

    struct AckQueue
    {
        nura::ControlPayload items[4];
        uint8_t head = 0U;
        uint8_t count = 0U;

        bool push(const nura::ControlPayload &item);
        bool pop(nura::ControlPayload &out);
    };

    bool receiveControl(uint32_t nowMs);
    bool sendAckIfQueued();
    bool sendFastTelemetry(uint32_t nowMs);
    bool sendGpsTelemetry(uint32_t nowMs);
    bool sendRawFrame(uint8_t type, const uint8_t *payload, uint8_t payloadLen);
    void handleCommand(const nura::ParsedFrame &frame, const nura::ControlPayload &command, uint32_t nowMs);
    void enqueueAck(const nura::ControlPayload &command, uint8_t stage, uint8_t result, uint8_t reason, uint16_t detailFlags = 0U);
    bool wasRecentlyProcessed(const nura::ControlPayload &command) const;
    void rememberCommand(const nura::ControlPayload &command);
    nura::FastTelemetry buildFastTelemetry(uint32_t nowMs) const;
    nura::GpsTelemetry buildGpsTelemetry(uint32_t nowMs) const;
    uint16_t buildStatusWord(uint32_t nowMs) const;
    uint8_t currentFlightStateCode() const;
    Sx127xLoRaConfig buildRadioConfig() const;

    Sx127xLoRaHAL &radio_;
    const ImuState &imuState_;
    const GpsState &gpsState_;
    TelemetryState &telemetryState_;
    FlightState &flightState_;
    const AbortState &abortState_;
    Logger &logger_;
    const IAppConfig &config_;
    nura::Parser parser_;
    AckQueue ackQueue_;
    RecentCommand recentCommands_[4];
    uint8_t recentCommandWriteIndex_ = 0U;
    uint16_t downlinkSeq_ = 0U;
    uint32_t lastFastTxMs_ = 0UL;
    uint32_t lastGpsTxMs_ = 0UL;
    bool radioReady_ = false;
    bool sentFast_ = false;
    bool sentGps_ = false;
};
