#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "logging/flight_log_ram_buffer.h"
#include "logging/flight_log_byte_queue.h"
#include "logging/flight_log_mirror_storage.h"
#include "logging/flight_log_record.h"
#include "missions/logging/flight_log_task.h"

namespace
{
void testEncodeFrameCrc()
{
    nura_log::EventPayload payload{};
    payload.eventId = 1U;
    payload.currentState = 2U;
    payload.data0 = 123U;

    uint8_t frame[nura_log::kMaxEncodedFrameBytes] = {};
    const size_t length = nura_log::encodeFrame(nura_log::RecordType::EVENT,
                                                7U,
                                                99U,
                                                &payload,
                                                sizeof(payload),
                                                frame,
                                                sizeof(frame));
    assert(length == sizeof(nura_log::FrameHeader) + sizeof(payload) + sizeof(uint16_t));

    nura_log::FrameHeader header{};
    memcpy(&header, frame, sizeof(header));
    assert(header.magic == nura_log::kFrameMagic);
    assert(header.version == nura_log::kFrameVersion);
    assert(header.type == static_cast<uint8_t>(nura_log::RecordType::EVENT));
    assert(header.sequence == 7U);
    assert(header.timestampMs == 99U);

    uint16_t storedCrc = 0U;
    memcpy(&storedCrc, frame + length - sizeof(storedCrc), sizeof(storedCrc));
    assert(storedCrc == nura_log::crc16Ccitt(frame, length - sizeof(storedCrc)));
}

void testRamBufferDropsOldest()
{
    FlightLogRamBuffer buffer;
    uint8_t record[200] = {};
    for (uint16_t i = 0U; i < sizeof(record); ++i)
    {
        record[i] = static_cast<uint8_t>(i);
    }

    uint32_t pushed = 0U;
    while (buffer.droppedRecords() == 0U)
    {
        assert(buffer.push(record, sizeof(record)));
        ++pushed;
        assert(pushed < 200U);
    }

    assert(buffer.recordCount() > 0U);
    assert(buffer.used() <= buffer.capacity());
}

void testByteQueueWrapsWithoutChangingData()
{
    FlightLogByteQueue queue;
    uint8_t first[7000] = {};
    uint8_t second[2000] = {};
    for (size_t i = 0U; i < sizeof(first); ++i)
    {
        first[i] = static_cast<uint8_t>(i & 0xFFU);
    }
    for (size_t i = 0U; i < sizeof(second); ++i)
    {
        second[i] = static_cast<uint8_t>((i + 37U) & 0xFFU);
    }

    assert(queue.push(first, sizeof(first)));
    uint8_t out[6000] = {};
    assert(queue.peek(out, sizeof(out)) == sizeof(out));
    assert(memcmp(out, first, sizeof(out)) == 0);
    assert(queue.consume(sizeof(out)));
    assert(queue.push(second, sizeof(second)));

    uint8_t tail[3000] = {};
    assert(queue.peek(tail, sizeof(tail)) == sizeof(tail));
    assert(memcmp(tail, first + sizeof(out), 1000U) == 0);
    assert(memcmp(tail + 1000U, second, sizeof(second)) == 0);
    assert(queue.consume(sizeof(tail)));
    assert(queue.empty());
}

class FakeAsyncStorage : public IFlightLogStorage
{
public:
    bool begin() override
    {
        ++beginCalls;
        active = beginOk;
        return active;
    }
    bool canAppend(uint16_t length) const override
    {
        return active && pending + length <= capacity;
    }
    bool append(const uint8_t *data, uint16_t length) override
    {
        if (data == nullptr || !canAppend(length))
        {
            return false;
        }
        pending = static_cast<uint16_t>(pending + length);
        accepted = static_cast<uint16_t>(accepted + length);
        return true;
    }
    bool service(uint32_t nowMs) override
    {
        (void)nowMs;
        if (!active || failService)
        {
            active = false;
            return false;
        }
        const uint16_t drained = pending < drainPerService ? pending : drainPerService;
        pending = static_cast<uint16_t>(pending - drained);
        ++serviceCalls;
        return true;
    }
    bool requestFlush() override
    {
        flushing = active;
        return active;
    }
    bool idle() const override { return pending == 0U; }
    void stop() override
    {
        active = false;
        stopped = true;
    }
    bool healthy() const override { return active; }

    uint16_t capacity = 1024U;
    uint16_t pending = 0U;
    uint16_t accepted = 0U;
    uint16_t drainPerService = 0xFFFFU;
    uint32_t beginCalls = 0U;
    uint32_t serviceCalls = 0U;
    bool beginOk = true;
    bool active = false;
    bool failService = false;
    bool flushing = false;
    bool stopped = false;
};

void testMirrorBackpressureAndRuntimeDegradation()
{
    FakeAsyncStorage primary;
    FakeAsyncStorage mirror;
    FlightLogMirrorStorage storage(primary, mirror);
    assert(storage.begin());
    assert(storage.fullyHealthy());

    uint8_t record[128] = {};
    assert(storage.canAppend(sizeof(record)));
    assert(storage.append(record, sizeof(record)));
    assert(primary.accepted == sizeof(record));
    assert(mirror.accepted == sizeof(record));

    primary.failService = true;
    assert(storage.service(20U));
    assert(!storage.primaryHealthy());
    assert(storage.mirrorHealthy());
    assert(storage.healthy());

    assert(storage.canAppend(sizeof(record)));
    assert(storage.append(record, sizeof(record)));
    assert(primary.accepted == sizeof(record));
    assert(mirror.accepted == sizeof(record) * 2U);
}

void testAlreadyPreparedStorageIsNotReopened()
{
    FlightState flightState;
    ImuState imuState;
    HighGImuState highGImuState;
    MagnetometerState magnetometerState;
    GpsState gpsState;
    BarometerState barometerState;
    PowerState powerState;
    SystemHealthState healthState;
    TelemetrySnapshot telemetryState{barometerState, powerState, healthState};
    FlightTraceBuffer flightTrace;
    Logger logger;
    FakeAsyncStorage storage;

    assert(storage.begin());
    assert(storage.beginCalls == 1U);

    FlightLogTask task(flightState,
                       imuState,
                       highGImuState,
                       magnetometerState,
                       gpsState,
                       telemetryState,
                       flightTrace,
                       storage,
                       logger);
    assert(task.init());
    assert(storage.beginCalls == 1U);
}

void testGroundDrainRemainsIncremental()
{
    FlightState flightState;
    ImuState imuState;
    HighGImuState highGImuState;
    MagnetometerState magnetometerState;
    GpsState gpsState;
    BarometerState barometerState;
    PowerState powerState;
    SystemHealthState healthState;
    TelemetrySnapshot telemetryState{barometerState, powerState, healthState};
    FlightTraceBuffer flightTrace;
    Logger logger;
    FakeAsyncStorage storage;
    storage.capacity = 8192U;
    storage.drainPerService = 32U;

    FlightLogTask task(flightState,
                       imuState,
                       highGImuState,
                       magnetometerState,
                       gpsState,
                       telemetryState,
                       flightTrace,
                       storage,
                       logger);
    flightState.state = State::SAFE;
    assert(task.init());
    assert(task.tick(20U));

    flightState.state = State::GROUND;
    assert(task.tick(40U));
    assert(!storage.stopped);

    for (uint32_t nowMs = 60U; nowMs < 5000U && !storage.stopped; nowMs += 20U)
    {
        const uint32_t before = storage.serviceCalls;
        assert(task.tick(nowMs));
        assert(storage.serviceCalls - before <= 1U);
    }
    assert(storage.stopped);
    assert(storage.flushing);
}

} // namespace

int main()
{
    testEncodeFrameCrc();
    testRamBufferDropsOldest();
    testByteQueueWrapsWithoutChangingData();
    testMirrorBackpressureAndRuntimeDegradation();
    testAlreadyPreparedStorageIsNotReopened();
    testGroundDrainRemainsIncremental();
    std::cout << "logging tests passed\n";
    return 0;
}
