#include "flight_log_record.h"

#include <string.h>

namespace nura_log
{
uint16_t crc16Ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

size_t encodeFrame(RecordType type,
                   uint32_t sequence,
                   uint32_t timestampMs,
                   const void *payload,
                   uint16_t payloadLength,
                   uint8_t *out,
                   size_t outCapacity)
{
    const size_t frameLength = sizeof(FrameHeader) + payloadLength + sizeof(uint16_t);
    if (payload == nullptr || out == nullptr || frameLength > outCapacity)
    {
        return 0U;
    }

    FrameHeader header;
    header.magic = kFrameMagic;
    header.version = kFrameVersion;
    header.type = static_cast<uint8_t>(type);
    header.payloadLength = payloadLength;
    header.sequence = sequence;
    header.timestampMs = timestampMs;

    memcpy(out, &header, sizeof(header));
    memcpy(out + sizeof(header), payload, payloadLength);

    const uint16_t crc = crc16Ccitt(out, sizeof(header) + payloadLength);
    memcpy(out + sizeof(header) + payloadLength, &crc, sizeof(crc));
    return frameLength;
}
} // namespace nura_log
