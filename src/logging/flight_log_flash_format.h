#pragma once

#include <stdint.h>

namespace nura_log
{
constexpr uint32_t kFlashSectorMagic = 0x4e4c4653UL; // "SFLN" in little-endian dumps.
constexpr uint16_t kFlashSectorVersion = 1U;

#pragma pack(push, 1)
struct FlashSectorHeader
{
    uint32_t magic;
    uint16_t version;
    uint16_t headerBytes;
    uint32_t sectorBytes;
    uint32_t payloadOffset;
    uint32_t sessionId;
    uint32_t sectorSequence;
    uint32_t headerCrc;
};
#pragma pack(pop)

uint32_t flashHeaderCrc(const FlashSectorHeader &header);
FlashSectorHeader makeFlashSectorHeader(uint32_t sessionId,
                                        uint32_t sectorSequence,
                                        uint32_t sectorBytes,
                                        uint32_t payloadOffset);
bool validFlashSectorHeader(const FlashSectorHeader &header,
                            uint32_t expectedSectorBytes,
                            uint32_t expectedPayloadOffset);
uint32_t nextFlashSessionId(uint32_t previousSessionId);
} // namespace nura_log
