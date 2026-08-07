#include <cstdio>
#include <vector>

#include "Arduino.h"
#include "app/app_config.h"
#include "core/logger/logger.h"
#include "hal/panic_handler.h"
#include "hal/pyro_output.h"
#include "missions/flight/fsm_task.h"
#include "missions/telemetry/telemetry_task.h"

namespace
{
struct FakeConfig final : IAppConfig
{
    unsigned long serialBaudRate() const override { return NuraConstants::App::kSerialBaudRate; }
    uint8_t statusIndicatorPin() const override { return 13U; }
    uint16_t faultBlinkIntervalMs() const override { return NuraConstants::App::kFaultBlinkIntervalMs; }
    uint8_t imuCsPin() const override { return 10U; }
    uint8_t imuReadFailureThreshold() const override { return NuraConstants::Sensors::kImuReadFailureThreshold; }
    uint8_t imuMaxRecoveryAttempts() const override { return NuraConstants::Sensors::kImuMaxRecoveryAttempts; }
    uint32_t imuRecoveryIntervalMs() const override { return NuraConstants::Sensors::kImuRecoveryIntervalMs; }
    uint32_t imuTaskPeriodMs() const override { return NuraConstants::Sensors::kImuTaskPeriodMs; }
    uint32_t magnetometerTaskPeriodMs() const override { return NuraConstants::Sensors::kMagnetometerTaskPeriodMs; }
    uint32_t barometerTaskPeriodMs() const override { return NuraConstants::Sensors::kBarometerTaskPeriodMs; }
    uint32_t barometerRecoveryIntervalMs() const override { return NuraConstants::Sensors::kBarometerRecoveryIntervalMs; }
    uint32_t gnssTaskPeriodMs() const override { return NuraConstants::Sensors::kGnssTaskPeriodMs; }
    uint16_t gnssPollByteBudget() const override { return NuraConstants::Sensors::kGnssPollByteBudget; }
    uint32_t gnssMaxFixAgeMs() const override { return NuraConstants::Sensors::kGnssMaxFixAgeMs; }
    uint32_t watchdogTaskPeriodMs() const override { return NuraConstants::Tasks::kWatchdogTaskPeriodMs; }
    uint32_t flightStateTaskPeriodMs() const override { return NuraConstants::Tasks::kFlightStateTaskPeriodMs; }
    uint32_t loggerTaskPeriodMs() const override { return NuraConstants::Tasks::kLoggerTaskPeriodMs; }
    uint32_t telemetryTaskPeriodMs() const override { return NuraConstants::Tasks::kTelemetryTaskPeriodMs; }
    uint32_t telemetryFastPeriodMs() const override { return NuraConstants::Telemetry::kFastPeriodMs; }
    uint32_t telemetryGpsPeriodMs() const override { return NuraConstants::Telemetry::kGpsPeriodMs; }
    uint32_t telemetrySensorFreshMs() const override { return NuraConstants::Telemetry::kSensorFreshMs; }
    uint8_t loggerDrainBudget() const override { return NuraConstants::Logger::kDrainBudget; }
    uint8_t loggerOutputFailThreshold() const override { return NuraConstants::Logger::kOutputFailThreshold; }
    long loraFrequencyHz() const override { return NuraConstants::LoRa::kFlightFrequencyHz; }
    uint32_t loraSpiFrequencyHz() const override { return NuraConstants::LoRa::kFlightSpiFrequencyHz; }
    int loraTxPowerDbm() const override { return NuraConstants::LoRa::kFlightTxPowerDbm; }
    int loraSpreadingFactor() const override { return NuraConstants::LoRa::kSpreadingFactor; }
    long loraSignalBandwidthHz() const override { return NuraConstants::LoRa::kSignalBandwidthHz; }
    int loraCodingRateDenominator() const override { return NuraConstants::LoRa::kCodingRateDenominator; }
    long loraPreambleLength() const override { return NuraConstants::LoRa::kPreambleLength; }
    int loraSyncWord() const override { return NuraConstants::LoRa::kSyncWord; }
    uint8_t loraInitAttempts() const override { return 1U; }
    uint8_t loraSpiMode() const override { return NuraConstants::LoRa::kFlightSpiMode; }
    bool loraProbeSpiMode() const override { return false; }
};

struct FakePanicHandler final : IPanicHandler
{
    void panic(const char *reason = nullptr) override
    {
        (void)reason;
    }
};

struct RecordingPyro final : IPyroOutput
{
    bool begin() override { return true; }
    void allOff() override
    {
        drogue = false;
        main = false;
    }
    void setDrogue(bool enabled) override
    {
        drogue = enabled;
        if (enabled)
        {
            ++drogueOnCount;
        }
    }
    void setMain(bool enabled) override
    {
        main = enabled;
        if (enabled)
        {
            ++mainOnCount;
        }
    }

    bool drogue = false;
    bool main = false;
    uint8_t drogueOnCount = 0U;
    uint8_t mainOnCount = 0U;
};

struct Fixture
{
    FakeConfig config;
    Logger logger;
    FlightState flight;
    AbortState abort;
    ImuState imu;
    HighGImuState highG;
    GpsState gps;
    BarometerState barometer;
    PowerState power;
    SystemHealthState health;
    TelemetrySnapshot snapshot{barometer, power, health};
    FlightTraceBuffer trace;
    FakePanicHandler panic;
    RecordingPyro pyro;
    TelemetryLoRaHAL radio;
    FlightStateMachineTask fsm{flight, abort, highG, imu, snapshot, logger, config, panic, &pyro, nullptr, &trace};
    TelemetryTask telemetry{radio, imu, gps, snapshot, flight, abort, logger, config};

    bool init()
    {
        nuraTestMillis = 0UL;
        if (!fsm.init())
        {
            return false;
        }
        fsm.tick(0UL);
        return flight.state == State::SAFE && telemetry.init();
    }

    void telemetryTick(uint32_t nowMs)
    {
        nuraTestMillis = nowMs;
        telemetry.tick(nowMs);
    }
};

nura::ControlPayload armCommand(uint16_t commandSeq, uint32_t nonce, uint32_t validUntilMs)
{
    nura::ControlPayload command;
    command.subtype = nura::CONTROL_CMD;
    command.commandId = nura::COMMAND_ARM_FLIGHT;
    command.commandSeq = commandSeq;
    command.nonce = nonce;
    command.validUntilMs = validUntilMs;
    command.param0 = nura::FLIGHT_SAFE;
    command.param1 = nura::FLIGHT_ARMED;
    return command;
}

std::vector<uint8_t> encodeCommand(nura::ControlPayload command,
                                   uint16_t frameSeq,
                                   bool corruptControlAuth = false,
                                   bool corruptCrc = false)
{
    nura::makeControlAuthTag(command,
                             frameSeq,
                             NuraConstants::Telemetry::kControlAuthKey,
                             command.authOrAck);
    if (corruptControlAuth)
    {
        command.authOrAck[0] ^= 0x01U;
    }

    uint8_t payload[nura::kControlPayloadLen];
    uint8_t frame[nura::kMaxFrameLen];
    if (!nura::encodeControlPayload(command, payload, sizeof(payload)))
    {
        return {};
    }
    const size_t length = nura::encodeFrame(nura::MESSAGE_CONTROL,
                                             NuraConstants::Telemetry::kVehicleId,
                                             frameSeq,
                                             nura::FrameDirection::UPLINK,
                                             NuraConstants::Telemetry::kControlAuthKey,
                                             payload,
                                             sizeof(payload),
                                             frame,
                                             sizeof(frame));
    if (length == 0U)
    {
        return {};
    }
    if (corruptCrc)
    {
        frame[length - 1U] ^= 0x01U;
    }
    return std::vector<uint8_t>(frame, frame + length);
}

bool decodeAck(const std::vector<uint8_t> &frame, nura::ControlPayload &ack)
{
    nura::ParsedFrame parsed;
    return nura::decodeFrame(frame.data(),
                             frame.size(),
                             NuraConstants::Telemetry::kVehicleId,
                             nura::FrameDirection::DOWNLINK,
                             NuraConstants::Telemetry::kControlAuthKey,
                             parsed) &&
           parsed.type == nura::MESSAGE_CONTROL &&
           nura::decodeControlPayload(parsed.payload, parsed.payloadLen, ack);
}

bool hasDownlinkType(const std::vector<uint8_t> &frame, uint8_t expectedType)
{
    nura::ParsedFrame parsed;
    return nura::decodeFrame(frame.data(),
                             frame.size(),
                             NuraConstants::Telemetry::kVehicleId,
                             nura::FrameDirection::DOWNLINK,
                             NuraConstants::Telemetry::kControlAuthKey,
                             parsed) &&
           parsed.type == expectedType;
}

bool expectAck(const char *name,
               const Fixture &fixture,
               size_t index,
               uint8_t stage,
               uint8_t result,
               uint8_t reason,
               uint8_t state)
{
    nura::ControlPayload ack;
    if (index >= fixture.radio.txFrames.size() ||
        !decodeAck(fixture.radio.txFrames[index], ack) ||
        ack.authOrAck[0] != stage ||
        ack.authOrAck[1] != result ||
        ack.authOrAck[2] != reason ||
        ack.authOrAck[3] != state)
    {
        std::fprintf(stderr, "FAIL %s ACK mismatch at index %zu\n", name, index);
        return false;
    }
    return true;
}

bool checkAcceptedDuplicateExecutedFlow()
{
    Fixture fixture;
    if (!fixture.init() || fixture.radio.lastConfig.downlinkOnly)
    {
        std::fprintf(stderr, "FAIL arm_flow init/RX configuration\n");
        return false;
    }

    const std::vector<uint8_t> frame = encodeCommand(armCommand(0x3344U, 0xA1B2C3D4UL, 5000UL), 0x1234U);
    fixture.radio.queueRx(frame);
    fixture.telemetryTick(100UL);
    bool ok = expectAck("accepted", fixture, 0U,
                        nura::ACK_ACCEPTED, nura::RESULT_OK, nura::REJECT_NONE, nura::FLIGHT_SAFE);
    if (!fixture.flight.armRequested || fixture.flight.state != State::SAFE)
    {
        std::fprintf(stderr, "FAIL arm_flow request was not queued in SAFE\n");
        return false;
    }

    fixture.fsm.tick(110UL);
    if (fixture.flight.state != State::ARMED ||
        !fixture.flight.armExecuted ||
        fixture.flight.armExecutedSeq != 0x3344U)
    {
        std::fprintf(stderr, "FAIL arm_flow FSM did not execute matching request\n");
        return false;
    }

    fixture.radio.queueRx(frame);
    fixture.telemetryTick(120UL);
    ok = expectAck("duplicate_before_executed_ack", fixture, 1U,
                   nura::ACK_DUPLICATE, nura::RESULT_ALREADY_DONE, nura::REJECT_NONE, nura::FLIGHT_ARMED) &&
         ok;
    fixture.telemetryTick(140UL);
    ok = expectAck("executed", fixture, 2U,
                   nura::ACK_EXECUTED, nura::RESULT_OK, nura::REJECT_NONE, nura::FLIGHT_ARMED) &&
         ok;

    for (uint8_t retry = 1U; retry < 8U; ++retry)
    {
        const uint32_t nowMs = 140UL + (static_cast<uint32_t>(retry) * 250UL);
        fixture.radio.queueRx(frame);
        fixture.telemetryTick(nowMs);
        ok = expectAck("duplicate_retry", fixture, static_cast<size_t>(retry + 2U),
                       nura::ACK_DUPLICATE, nura::RESULT_ALREADY_DONE, nura::REJECT_NONE, nura::FLIGHT_ARMED) &&
             ok;
        fixture.fsm.tick(nowMs + 1UL);
    }

    fixture.telemetryTick(2000UL);
    fixture.telemetryTick(2020UL);
    if (fixture.radio.txFrames.size() != 12U ||
        !hasDownlinkType(fixture.radio.txFrames[10U], nura::MESSAGE_FAST_TLM) ||
        !hasDownlinkType(fixture.radio.txFrames[11U], nura::MESSAGE_GPS_TLM))
    {
        std::fprintf(stderr, "FAIL arm_flow FAST/GPS did not resume after ACK traffic\n");
        return false;
    }

    uint8_t armTransitionCount = 0U;
    FlightStateTransitionTrace transition;
    while (fixture.trace.popTransition(transition))
    {
        if (transition.previous == State::SAFE && transition.current == State::ARMED)
        {
            ++armTransitionCount;
        }
    }
    if (armTransitionCount != 1U ||
        fixture.pyro.drogueOnCount != 0U ||
        fixture.pyro.mainOnCount != 0U)
    {
        std::fprintf(stderr, "FAIL arm_flow duplicate execution or pyro activation\n");
        return false;
    }
    return ok;
}

bool expectRejected(const char *name,
                    nura::ControlPayload command,
                    uint32_t nowMs,
                    uint8_t result,
                    uint8_t reason,
                    bool corruptControlAuth = false,
                    State state = State::SAFE,
                    bool abortActive = false)
{
    Fixture fixture;
    if (!fixture.init())
    {
        return false;
    }
    fixture.flight.state = state;
    fixture.abort.status.active = abortActive;
    fixture.radio.queueRx(encodeCommand(command, 7U, corruptControlAuth));
    fixture.telemetryTick(nowMs);
    const bool ackOk = expectAck(name, fixture, 0U,
                                 nura::ACK_REJECTED, result, reason,
                                 state == State::SAFE ? nura::FLIGHT_SAFE : nura::FLIGHT_ARMED);
    if (fixture.flight.armRequested || fixture.pyro.drogueOnCount != 0U || fixture.pyro.mainOnCount != 0U)
    {
        std::fprintf(stderr, "FAIL %s rejection changed flight outputs\n", name);
        return false;
    }
    return ackOk;
}

bool checkRejections()
{
    bool ok = true;
    ok = expectRejected("expired",
                        armCommand(1U, 11UL, 99UL),
                        100UL,
                        nura::RESULT_EXPIRED,
                        nura::REJECT_COMMAND_EXPIRED) &&
         ok;
    ok = expectRejected("zero_expiry",
                        armCommand(2U, 12UL, 0UL),
                        100UL,
                        nura::RESULT_BAD_FORMAT,
                        nura::REJECT_NONE) &&
         ok;
    nura::ControlPayload badParams = armCommand(3U, 13UL, 500UL);
    badParams.param1 = nura::FLIGHT_LAUNCH;
    ok = expectRejected("bad_params",
                        badParams,
                        100UL,
                        nura::RESULT_BAD_FORMAT,
                        nura::REJECT_NONE) &&
         ok;
    ok = expectRejected("wrong_state",
                        armCommand(4U, 14UL, 500UL),
                        100UL,
                        nura::RESULT_BAD_STATE,
                        nura::REJECT_STATE_REJECTED,
                        false,
                        State::ARMED) &&
         ok;
    ok = expectRejected("abort_active",
                        armCommand(5U, 15UL, 500UL),
                        100UL,
                        nura::RESULT_BAD_STATE,
                        nura::REJECT_STATE_REJECTED,
                        false,
                        State::SAFE,
                        true) &&
         ok;
    ok = expectRejected("control_auth",
                        armCommand(6U, 16UL, 500UL),
                        100UL,
                        nura::RESULT_AUTH_FAILED,
                        nura::REJECT_AUTH_TAG_MISMATCH,
                        true) &&
         ok;

    Fixture crcFixture;
    if (!crcFixture.init())
    {
        return false;
    }
    crcFixture.radio.queueRx(encodeCommand(armCommand(7U, 17UL, 500UL), 8U, false, true));
    crcFixture.telemetryTick(100UL);
    for (const std::vector<uint8_t> &frame : crcFixture.radio.txFrames)
    {
        nura::ControlPayload ack;
        if (decodeAck(frame, ack))
        {
            std::fprintf(stderr, "FAIL crc_error produced a CONTROL ACK\n");
            return false;
        }
    }
    if (crcFixture.flight.armRequested)
    {
        std::fprintf(stderr, "FAIL crc_error queued ARM\n");
        return false;
    }
    return ok;
}

bool checkAcceptedAbortRace()
{
    Fixture fixture;
    if (!fixture.init())
    {
        return false;
    }

    fixture.radio.queueRx(encodeCommand(armCommand(88U, 0x10203040UL, 1000UL), 9U));
    fixture.telemetryTick(100UL);
    bool ok = expectAck("race_accepted", fixture, 0U,
                        nura::ACK_ACCEPTED, nura::RESULT_OK, nura::REJECT_NONE, nura::FLIGHT_SAFE);

    fixture.abort.status.active = true;
    fixture.fsm.tick(110UL);
    fixture.telemetryTick(120UL);
    ok = expectAck("race_rejected", fixture, 1U,
                   nura::ACK_REJECTED,
                   nura::RESULT_BAD_STATE,
                   nura::REJECT_STATE_REJECTED,
                   nura::FLIGHT_SAFE) &&
         ok;

    if (fixture.flight.state != State::SAFE ||
        fixture.flight.armRequested ||
        fixture.flight.armExecuted ||
        !fixture.flight.armRejected ||
        fixture.pyro.drogueOnCount != 0U ||
        fixture.pyro.mainOnCount != 0U)
    {
        std::fprintf(stderr, "FAIL accepted_abort_race state/output mismatch\n");
        return false;
    }
    return ok;
}
} // namespace

int main()
{
    const bool ok = checkAcceptedDuplicateExecutedFlow() &&
                    checkRejections() &&
                    checkAcceptedAbortRace();
    if (!ok)
    {
        return 1;
    }
    std::puts("Telemetry ARM flow tests passed");
    return 0;
}
