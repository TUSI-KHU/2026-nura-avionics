#include "program_flash_flight_log_storage.h"

#include <string.h>

#include "logging/flight_log_record.h"
#include "nura_constants.h"

namespace
{
constexpr const char *kRawJournalPath = "RAW:W25Q128/NURA_LOG";

uint16_t crcWithZeroedField(const void *data,
                            uint16_t length,
                            uint16_t fieldOffset)
{
    uint8_t copy[sizeof(nura_flash_log::PageHeader)] = {};
    if (length > sizeof(copy) || fieldOffset + sizeof(uint16_t) > length)
    {
        return 0U;
    }
    memcpy(copy, data, length);
    copy[fieldOffset] = 0U;
    copy[fieldOffset + 1U] = 0U;
    return nura_log::crc16Ccitt(copy, length);
}
} // namespace

ProgramFlashFlightLogStorage::ProgramFlashFlightLogStorage(W25Q128QspiHAL &flash)
    : flash_(flash)
{
}

bool ProgramFlashFlightLogStorage::begin()
{
    queue_.clear();
    healthy_ = false;
    flushRequested_ = false;
    stopped_ = false;
    operation_ = Operation::NONE;
    operationPayloadBytes_ = 0U;
    erasedAheadSectors_ = 0U;
    validSectorCount_ = 0U;

    if (!flash_.begin())
    {
        return false;
    }

    bool found = false;
    if (!scanJournal(found))
    {
        return false;
    }
    if (!found && !initializeFreshJournal())
    {
        return false;
    }

    healthy_ = true;
    return true;
}

bool ProgramFlashFlightLogStorage::canAppend(uint16_t length) const
{
    return healthy() && !flushRequested_ && length > 0U && length <= queue_.free();
}

bool ProgramFlashFlightLogStorage::append(const uint8_t *data, uint16_t length)
{
    if (!canAppend(length) || data == nullptr)
    {
        return false;
    }
    return queue_.push(data, length);
}

bool ProgramFlashFlightLogStorage::service(uint32_t nowMs)
{
    if (!healthy() || stopped_)
    {
        return false;
    }

    if (operation_ != Operation::NONE)
    {
        bool flashReady = false;
        if (!flash_.ready(flashReady))
        {
            healthy_ = false;
            return false;
        }
        if (!flashReady)
        {
            if (operationTimedOut(nowMs))
            {
                healthy_ = false;
                return false;
            }
            return true;
        }
        if (!finishOperation(nowMs))
        {
            healthy_ = false;
            return false;
        }
    }

    if (flushRequested_ && queue_.empty())
    {
        return true;
    }

    if (nextPageIndex_ >= kPagesPerSector)
    {
        if (erasedAheadSectors_ == 0U)
        {
            return startEraseAhead(nowMs);
        }
        return rotateSector(nowMs);
    }

    if (!flushRequested_ &&
        erasedAheadSectors_ < NuraConstants::Logger::kFlightLogQspiEraseAheadSectors)
    {
        return startEraseAhead(nowMs);
    }

    if (queue_.used() >= kDataPayloadBytes ||
        (flushRequested_ && !queue_.empty()))
    {
        return startDataPage(nowMs);
    }

    return true;
}

bool ProgramFlashFlightLogStorage::requestFlush()
{
    if (!healthy() || stopped_)
    {
        return false;
    }
    flushRequested_ = true;
    return true;
}

bool ProgramFlashFlightLogStorage::idle() const
{
    return queue_.empty() && operation_ == Operation::NONE;
}

void ProgramFlashFlightLogStorage::stop()
{
    if (idle())
    {
        stopped_ = true;
    }
}

bool ProgramFlashFlightLogStorage::healthy() const
{
    return healthy_ && !stopped_;
}

const char *ProgramFlashFlightLogStorage::path() const
{
    return kRawJournalPath;
}

uint64_t ProgramFlashFlightLogStorage::totalBytes() const
{
    return flash_.capacityBytes();
}

uint64_t ProgramFlashFlightLogStorage::usedBytes() const
{
    return static_cast<uint64_t>(validSectorCount_) * kSectorBytes;
}

bool ProgramFlashFlightLogStorage::scanJournal(bool &found)
{
    found = false;
    nura_flash_log::SectorHeader newest = {};
    uint16_t newestSector = 0U;

    for (uint16_t sector = 0U; sector < kSectorCount; ++sector)
    {
        nura_flash_log::SectorHeader header = {};
        bool blank = false;
        if (!readSectorHeader(sector, header, blank))
        {
            return false;
        }
        if (!blank && validSectorHeader(header))
        {
            ++validSectorCount_;
            if (!found || sequenceNewer(header.sectorSequence, newest.sectorSequence))
            {
                newest = header;
                newestSector = sector;
                found = true;
            }
        }
    }

    if (!found)
    {
        return true;
    }

    currentSector_ = newestSector;
    currentSectorSequence_ = newest.sectorSequence;
    streamOffset_ = newest.firstStreamOffset;
    nextEraseSector_ = static_cast<uint16_t>((currentSector_ + 1U) % kSectorCount);
    return restoreCurrentSector(newest);
}

bool ProgramFlashFlightLogStorage::initializeFreshJournal()
{
    currentSector_ = 0U;
    currentSectorSequence_ = 0U;
    streamOffset_ = 0U;
    nextPageIndex_ = 1U;
    nextEraseSector_ = 1U;

    if (!flash_.eraseSectorForInit(
            sectorAddress(currentSector_),
            NuraConstants::Logger::kFlightLogQspiInitEraseTimeoutUs))
    {
        return false;
    }

    memset(pageBuffer_, 0xFF, sizeof(pageBuffer_));
    nura_flash_log::SectorHeader header = {};
    makeSectorHeader(header);
    memcpy(pageBuffer_, &header, sizeof(header));
    if (!flash_.programForInit(
            pageAddress(currentSector_, 0U),
            pageBuffer_,
            kPageBytes,
            NuraConstants::Logger::kFlightLogQspiProgramTimeoutUs))
    {
        return false;
    }

    if (!flash_.read(pageAddress(currentSector_, 0U), verifyBuffer_, kPageBytes) ||
        memcmp(pageBuffer_, verifyBuffer_, kPageBytes) != 0)
    {
        return false;
    }
    validSectorCount_ = 1U;
    return true;
}

bool ProgramFlashFlightLogStorage::restoreCurrentSector(
    const nura_flash_log::SectorHeader &header)
{
    nextPageIndex_ = 1U;
    streamOffset_ = header.firstStreamOffset;

    for (uint8_t page = 1U; page < kPagesPerSector; ++page)
    {
        if (!readPage(currentSector_, page, verifyBuffer_))
        {
            return false;
        }
        if (allErased(verifyBuffer_, kPageBytes))
        {
            nextPageIndex_ = page;
            return true;
        }

        nextPageIndex_ = static_cast<uint8_t>(page + 1U);
        if (validDataPage(verifyBuffer_))
        {
            nura_flash_log::PageHeader pageHeader = {};
            memcpy(&pageHeader, verifyBuffer_, sizeof(pageHeader));
            const uint32_t endOffset = pageHeader.streamOffset + pageHeader.payloadLength;
            if (sequenceNewer(endOffset, streamOffset_) || endOffset == streamOffset_)
            {
                streamOffset_ = endOffset;
            }
        }
    }
    return true;
}

bool ProgramFlashFlightLogStorage::startDataPage(uint32_t nowMs)
{
    const uint16_t payloadLength = queue_.used() < kDataPayloadBytes
                                       ? queue_.used()
                                       : kDataPayloadBytes;
    if (payloadLength == 0U)
    {
        return true;
    }

    makeDataPage(payloadLength);
    if (!flash_.startPageProgram(pageAddress(currentSector_, nextPageIndex_),
                                 pageBuffer_,
                                 kPageBytes))
    {
        return false;
    }
    operation_ = Operation::DATA_PAGE_PROGRAM;
    operationPayloadBytes_ = payloadLength;
    operationStartedMs_ = nowMs;
    return true;
}

bool ProgramFlashFlightLogStorage::startSectorHeader(uint32_t nowMs)
{
    memset(pageBuffer_, 0xFF, sizeof(pageBuffer_));
    nura_flash_log::SectorHeader header = {};
    makeSectorHeader(header);
    memcpy(pageBuffer_, &header, sizeof(header));
    if (!flash_.startPageProgram(pageAddress(currentSector_, 0U),
                                 pageBuffer_,
                                 kPageBytes))
    {
        return false;
    }
    operation_ = Operation::SECTOR_HEADER_PROGRAM;
    operationPayloadBytes_ = 0U;
    operationStartedMs_ = nowMs;
    return true;
}

bool ProgramFlashFlightLogStorage::startEraseAhead(uint32_t nowMs)
{
    if (!flash_.startSectorErase(sectorAddress(nextEraseSector_)))
    {
        return false;
    }
    operation_ = Operation::SECTOR_ERASE;
    operationPayloadBytes_ = 0U;
    operationStartedMs_ = nowMs;
    return true;
}

bool ProgramFlashFlightLogStorage::finishOperation(uint32_t nowMs)
{
    (void)nowMs;
    if (operation_ == Operation::SECTOR_ERASE)
    {
        nura_flash_log::SectorHeader erased = {};
        if (!flash_.read(sectorAddress(nextEraseSector_), &erased, sizeof(erased)) ||
            !allErased(&erased, sizeof(erased)))
        {
            return false;
        }
        nextEraseSector_ = static_cast<uint16_t>((nextEraseSector_ + 1U) % kSectorCount);
        ++erasedAheadSectors_;
        operation_ = Operation::NONE;
        return true;
    }

    const uint8_t pageIndex = operation_ == Operation::SECTOR_HEADER_PROGRAM
                                  ? 0U
                                  : nextPageIndex_;
    if (!flash_.read(pageAddress(currentSector_, pageIndex), verifyBuffer_, kPageBytes) ||
        memcmp(pageBuffer_, verifyBuffer_, kPageBytes) != 0)
    {
        return false;
    }

    if (operation_ == Operation::DATA_PAGE_PROGRAM)
    {
        if (!queue_.consume(operationPayloadBytes_))
        {
            return false;
        }
        streamOffset_ += operationPayloadBytes_;
        ++nextPageIndex_;
    }
    else if (operation_ == Operation::SECTOR_HEADER_PROGRAM)
    {
        nextPageIndex_ = 1U;
        if (validSectorCount_ < kSectorCount)
        {
            ++validSectorCount_;
        }
    }

    operationPayloadBytes_ = 0U;
    operation_ = Operation::NONE;
    return true;
}

bool ProgramFlashFlightLogStorage::rotateSector(uint32_t nowMs)
{
    currentSector_ = static_cast<uint16_t>((currentSector_ + 1U) % kSectorCount);
    ++currentSectorSequence_;
    --erasedAheadSectors_;
    nextPageIndex_ = 1U;
    return startSectorHeader(nowMs);
}

bool ProgramFlashFlightLogStorage::operationTimedOut(uint32_t nowMs) const
{
    const uint32_t timeoutMs = operation_ == Operation::SECTOR_ERASE
                                   ? (NuraConstants::Logger::kFlightLogQspiInitEraseTimeoutUs / 1000UL) + 20UL
                                   : (NuraConstants::Logger::kFlightLogQspiProgramTimeoutUs / 1000UL) + 20UL;
    return static_cast<uint32_t>(nowMs - operationStartedMs_) > timeoutMs;
}

bool ProgramFlashFlightLogStorage::readSectorHeader(
    uint16_t sectorIndex,
    nura_flash_log::SectorHeader &header,
    bool &blank)
{
    if (!flash_.read(sectorAddress(sectorIndex), &header, sizeof(header)))
    {
        return false;
    }
    blank = allErased(&header, sizeof(header));
    return true;
}

bool ProgramFlashFlightLogStorage::readPage(uint16_t sectorIndex,
                                            uint8_t pageIndex,
                                            uint8_t *page)
{
    return page != nullptr &&
           flash_.read(pageAddress(sectorIndex, pageIndex), page, kPageBytes);
}

bool ProgramFlashFlightLogStorage::validSectorHeader(
    const nura_flash_log::SectorHeader &header) const
{
    const uint16_t crcOffset = static_cast<uint16_t>(
        reinterpret_cast<const uint8_t *>(&header.crc16) -
        reinterpret_cast<const uint8_t *>(&header));
    return header.magic == nura_flash_log::kSectorMagic &&
           header.version == nura_flash_log::kJournalVersion &&
           header.crc16 == crcWithZeroedField(&header, sizeof(header), crcOffset);
}

bool ProgramFlashFlightLogStorage::validDataPage(const uint8_t *page) const
{
    if (page == nullptr)
    {
        return false;
    }
    nura_flash_log::PageHeader header = {};
    memcpy(&header, page, sizeof(header));
    if (header.magic != nura_flash_log::kPageMagic ||
        header.version != nura_flash_log::kJournalVersion ||
        header.payloadLength == 0U || header.payloadLength > kDataPayloadBytes)
    {
        return false;
    }

    const uint16_t crcOffset = static_cast<uint16_t>(
        reinterpret_cast<const uint8_t *>(&header.headerCrc16) -
        reinterpret_cast<const uint8_t *>(&header));
    return header.headerCrc16 == crcWithZeroedField(&header, sizeof(header), crcOffset) &&
           header.payloadCrc16 == nura_log::crc16Ccitt(
                                      page + sizeof(header),
                                      header.payloadLength);
}

bool ProgramFlashFlightLogStorage::allErased(const void *data, uint16_t length) const
{
    if (data == nullptr)
    {
        return false;
    }
    const uint8_t *bytes = static_cast<const uint8_t *>(data);
    for (uint16_t i = 0U; i < length; ++i)
    {
        if (bytes[i] != 0xFFU)
        {
            return false;
        }
    }
    return true;
}

bool ProgramFlashFlightLogStorage::sequenceNewer(uint32_t candidate,
                                                 uint32_t current) const
{
    return static_cast<int32_t>(candidate - current) > 0;
}

uint32_t ProgramFlashFlightLogStorage::sectorAddress(uint16_t sectorIndex) const
{
    return static_cast<uint32_t>(sectorIndex) * kSectorBytes;
}

uint32_t ProgramFlashFlightLogStorage::pageAddress(uint16_t sectorIndex,
                                                   uint8_t pageIndex) const
{
    return sectorAddress(sectorIndex) + static_cast<uint32_t>(pageIndex) * kPageBytes;
}

void ProgramFlashFlightLogStorage::makeSectorHeader(
    nura_flash_log::SectorHeader &header) const
{
    memset(&header, 0, sizeof(header));
    header.magic = nura_flash_log::kSectorMagic;
    header.sectorSequence = currentSectorSequence_;
    header.firstStreamOffset = streamOffset_;
    header.version = nura_flash_log::kJournalVersion;
    const uint16_t crcOffset = static_cast<uint16_t>(
        reinterpret_cast<const uint8_t *>(&header.crc16) -
        reinterpret_cast<const uint8_t *>(&header));
    header.crc16 = crcWithZeroedField(&header, sizeof(header), crcOffset);
}

void ProgramFlashFlightLogStorage::makeDataPage(uint16_t payloadLength)
{
    memset(pageBuffer_, 0xFF, sizeof(pageBuffer_));
    const uint16_t copied = queue_.peek(pageBuffer_ + sizeof(nura_flash_log::PageHeader),
                                        payloadLength);

    nura_flash_log::PageHeader header = {};
    header.magic = nura_flash_log::kPageMagic;
    header.streamOffset = streamOffset_;
    header.payloadLength = copied;
    header.payloadCrc16 = nura_log::crc16Ccitt(
        pageBuffer_ + sizeof(nura_flash_log::PageHeader), copied);
    header.version = static_cast<uint8_t>(nura_flash_log::kJournalVersion);
    const uint16_t crcOffset = static_cast<uint16_t>(
        reinterpret_cast<const uint8_t *>(&header.headerCrc16) -
        reinterpret_cast<const uint8_t *>(&header));
    header.headerCrc16 = crcWithZeroedField(&header, sizeof(header), crcOffset);
    memcpy(pageBuffer_, &header, sizeof(header));
}

bool ProgramFlashFlightLogStorage::verifyJournal(uint32_t &validPages,
                                                 uint32_t &payloadBytes)
{
    validPages = 0U;
    payloadBytes = 0U;
    for (uint16_t sector = 0U; sector < kSectorCount; ++sector)
    {
        nura_flash_log::SectorHeader header = {};
        bool blank = false;
        if (!readSectorHeader(sector, header, blank))
        {
            return false;
        }
        if (blank || !validSectorHeader(header))
        {
            continue;
        }
        for (uint8_t page = 1U; page < kPagesPerSector; ++page)
        {
            if (!readPage(sector, page, verifyBuffer_))
            {
                return false;
            }
            if (allErased(verifyBuffer_, kPageBytes))
            {
                break;
            }
            if (!validDataPage(verifyBuffer_))
            {
                continue;
            }
            nura_flash_log::PageHeader pageHeader = {};
            memcpy(&pageHeader, verifyBuffer_, sizeof(pageHeader));
            ++validPages;
            payloadBytes += pageHeader.payloadLength;
        }
    }
    return true;
}
