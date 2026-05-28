#include "flight_log_flash_format.h"

namespace nura_log
{
namespace
{
constexpr uint32_t kFnvOffset = 2166136261UL;
constexpr uint32_t kFnvPrime = 16777619UL;

uint32_t fnv1a(const uint8_t *data, uint32_t length)
{
    uint32_t hash = kFnvOffset;
    for (uint32_t i = 0U; i < length; ++i)
    {
        hash ^= data[i];
        hash *= kFnvPrime;
    }
    return hash;
}
} // namespace

uint32_t flashHeaderCrc(const FlashSectorHeader &header)
{
    FlashSectorHeader copy = header;
    copy.headerCrc = 0U;
    return fnv1a(reinterpret_cast<const uint8_t *>(&copy), static_cast<uint32_t>(sizeof(copy)));
}

FlashSectorHeader makeFlashSectorHeader(uint32_t sessionId,
                                        uint32_t sectorSequence,
                                        uint32_t sectorBytes,
                                        uint32_t payloadOffset)
{
    FlashSectorHeader header;
    header.magic = kFlashSectorMagic;
    header.version = kFlashSectorVersion;
    header.headerBytes = static_cast<uint16_t>(sizeof(FlashSectorHeader));
    header.sectorBytes = sectorBytes;
    header.payloadOffset = payloadOffset;
    header.sessionId = sessionId;
    header.sectorSequence = sectorSequence;
    header.headerCrc = 0U;
    header.headerCrc = flashHeaderCrc(header);
    return header;
}

bool validFlashSectorHeader(const FlashSectorHeader &header,
                            uint32_t expectedSectorBytes,
                            uint32_t expectedPayloadOffset)
{
    return header.magic == kFlashSectorMagic &&
           header.version == kFlashSectorVersion &&
           header.headerBytes == sizeof(FlashSectorHeader) &&
           header.sectorBytes == expectedSectorBytes &&
           header.payloadOffset == expectedPayloadOffset &&
           header.sessionId != 0U &&
           header.sessionId != 0xFFFFFFFFUL &&
           header.headerCrc == flashHeaderCrc(header);
}

uint32_t nextFlashSessionId(uint32_t previousSessionId)
{
    uint32_t next = previousSessionId + 1UL;
    if (next == 0U || next == 0xFFFFFFFFUL)
    {
        next = 1UL;
    }
    return next;
}
} // namespace nura_log
