#pragma once

#include <stdint.h>

#include "hal/w25q128_qspi_hal.h"
#include "logging/flight_log_byte_queue.h"
#include "logging/flight_log_storage.h"

namespace nura_flash_log
{
constexpr uint32_t kSectorMagic = 0x4E534543UL; // "NSEC"
constexpr uint32_t kPageMagic = 0x4E504147UL;   // "NPAG"
constexpr uint16_t kJournalVersion = 1U;

#pragma pack(push, 1)
struct SectorHeader
{
    uint32_t magic;
    uint32_t sectorSequence;
    uint32_t firstStreamOffset;
    uint16_t version;
    uint16_t crc16;
};

struct PageHeader
{
    uint32_t magic;
    uint32_t streamOffset;
    uint16_t payloadLength;
    uint16_t payloadCrc16;
    uint16_t headerCrc16;
    uint8_t version;
    uint8_t flags;
};
#pragma pack(pop)
static_assert(sizeof(SectorHeader) == 16U, "QSPI sector header layout changed");
static_assert(sizeof(PageHeader) == 16U, "QSPI page header layout changed");
} // namespace nura_flash_log

class ProgramFlashFlightLogStorage : public IFlightLogStorage
{
public:
    explicit ProgramFlashFlightLogStorage(W25Q128QspiHAL &flash);

    bool begin() override;
    bool canAppend(uint16_t length) const override;
    bool append(const uint8_t *data, uint16_t length) override;
    bool service(uint32_t nowMs) override;
    bool requestFlush() override;
    bool idle() const override;
    void stop() override;
    bool healthy() const override;

    const char *path() const;
    uint64_t totalBytes() const;
    uint64_t usedBytes() const;
    bool verifyJournal(uint32_t &validPages, uint32_t &payloadBytes);

private:
    enum class Operation : uint8_t
    {
        NONE,
        DATA_PAGE_PROGRAM,
        SECTOR_HEADER_PROGRAM,
        SECTOR_ERASE,
    };

    static constexpr uint16_t kPageBytes = NuraConstants::Logger::kFlightLogQspiPageBytes;
    static constexpr uint16_t kSectorBytes = NuraConstants::Logger::kFlightLogQspiEraseSectorBytes;
    static constexpr uint16_t kPagesPerSector = kSectorBytes / kPageBytes;
    static constexpr uint16_t kDataPayloadBytes =
        kPageBytes - static_cast<uint16_t>(sizeof(nura_flash_log::PageHeader));
    static constexpr uint16_t kSectorCount =
        NuraConstants::Logger::kFlightLogQspiFlashBytes / kSectorBytes;

    bool scanJournal(bool &found);
    bool initializeFreshJournal();
    bool restoreCurrentSector(const nura_flash_log::SectorHeader &header);
    bool startDataPage(uint32_t nowMs);
    bool startSectorHeader(uint32_t nowMs);
    bool startEraseAhead(uint32_t nowMs);
    bool finishOperation(uint32_t nowMs);
    bool rotateSector(uint32_t nowMs);
    bool operationTimedOut(uint32_t nowMs) const;

    bool readSectorHeader(uint16_t sectorIndex,
                          nura_flash_log::SectorHeader &header,
                          bool &blank);
    bool readPage(uint16_t sectorIndex, uint8_t pageIndex, uint8_t *page);
    bool validSectorHeader(const nura_flash_log::SectorHeader &header) const;
    bool validDataPage(const uint8_t *page) const;
    bool allErased(const void *data, uint16_t length) const;
    bool sequenceNewer(uint32_t candidate, uint32_t current) const;
    uint32_t sectorAddress(uint16_t sectorIndex) const;
    uint32_t pageAddress(uint16_t sectorIndex, uint8_t pageIndex) const;
    void makeSectorHeader(nura_flash_log::SectorHeader &header) const;
    void makeDataPage(uint16_t payloadLength);

    W25Q128QspiHAL &flash_;
    FlightLogByteQueue queue_;
    alignas(4) uint8_t pageBuffer_[kPageBytes] = {};
    alignas(4) uint8_t verifyBuffer_[kPageBytes] = {};

    uint16_t currentSector_ = 0U;
    uint16_t nextEraseSector_ = 1U;
    uint32_t currentSectorSequence_ = 0U;
    uint32_t streamOffset_ = 0U;
    uint32_t validSectorCount_ = 0U;
    uint32_t operationStartedMs_ = 0U;
    uint16_t operationPayloadBytes_ = 0U;
    uint8_t nextPageIndex_ = 1U;
    uint8_t erasedAheadSectors_ = 0U;
    Operation operation_ = Operation::NONE;
    bool healthy_ = false;
    bool flushRequested_ = false;
    bool stopped_ = false;
};
