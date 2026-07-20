#pragma once

#include <Arduino.h>
#include <SD.h>
#include <stdint.h>

#include "flight_log_byte_queue.h"
#include "flight_log_storage.h"

namespace nura_sd_log
{
constexpr uint32_t kBlockMagic = 0x4E534442UL; // "NSDB"
constexpr uint8_t kBlockVersion = 1U;

#pragma pack(push, 1)
struct BlockHeader
{
    uint32_t magic;
    uint32_t sessionId;
    uint32_t blockSequence;
    uint32_t streamOffset;
    uint16_t payloadLength;
    uint16_t payloadCrc16;
    uint16_t headerCrc16;
    uint8_t version;
    uint8_t flags;
};
#pragma pack(pop)
static_assert(sizeof(BlockHeader) == 24U, "SD block header layout changed");
} // namespace nura_sd_log

class SdFlightLogStorage : public IFlightLogStorage
{
public:
    SdFlightLogStorage(uint8_t csPin, const char *directory = "/NURA_LOG");

    bool begin() override;
    bool canAppend(uint16_t length) const override;
    bool append(const uint8_t *data, uint16_t length) override;
    bool service(uint32_t nowMs) override;
    bool requestFlush() override;
    bool idle() const override;
    void stop() override;
    bool healthy() const override;
    const char *path() const;
    uint32_t logicalBytesWritten() const;
    uint32_t sessionId() const;

private:
    bool openNextFile();
    void makeBlock(uint16_t payloadLength);
    uint32_t makeSessionId(uint16_t fileIndex) const;

    uint8_t csPin_;
    const char *directory_;
    mutable FsFile file_;
    FlightLogByteQueue queue_;
    alignas(4) uint8_t sectorBuffer_[NuraConstants::Logger::kFlightLogSdSectorBytes] = {};
    char path_[32] = {};
    uint32_t logicalBytesWritten_ = 0U;
    uint32_t physicalBytesWritten_ = 0U;
    uint32_t blockSequence_ = 0U;
    uint32_t sessionId_ = 0U;
    uint32_t busyStartedMs_ = 0U;
    bool healthy_ = false;
    bool flushRequested_ = false;
    bool busyObserved_ = false;
    bool stopped_ = false;
};
