#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "logging/flight_log_ram_buffer.h"
#include "logging/flight_log_record.h"

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

} // namespace

int main()
{
    testEncodeFrameCrc();
    testRamBufferDropsOldest();
    std::cout << "logging tests passed\n";
    return 0;
}
