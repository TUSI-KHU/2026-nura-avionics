#include "program_flash_flight_log_storage.h"

#include <stdio.h>
#include <string.h>

#include "nura_constants.h"

namespace
{
constexpr uint16_t kMaxLogFiles = 1000U;
} // namespace

ProgramFlashFlightLogStorage::ProgramFlashFlightLogStorage(LittleFS_Program &fs,
                                                           uint32_t sizeBytes,
                                                           const char *directory)
    : fs_(fs),
      sizeBytes_(sizeBytes),
      directory_(directory)
{
}

bool ProgramFlashFlightLogStorage::begin()
{
    stopped_ = false;
    healthy_ = false;
    activeFileBytes_ = 0U;
    path_[0] = '\0';

    if (!fs_.begin(sizeBytes_))
    {
        return false;
    }

    if (!fs_.exists(directory_) && !fs_.mkdir(directory_))
    {
        return false;
    }

    healthy_ = openNextFile();
    return healthy_;
}

bool ProgramFlashFlightLogStorage::append(const uint8_t *data, uint16_t length)
{
    if (!healthy_ || stopped_ || data == nullptr || length == 0U)
    {
        return false;
    }

    if ((activeFileBytes_ + length) > NuraConstants::Logger::kFlightLogFileSegmentBytes && !rotateFile())
    {
        healthy_ = false;
        return false;
    }

    if (!reclaimSpace(static_cast<uint32_t>(length) + NuraConstants::Logger::kFlightLogMinFreeBytes))
    {
        healthy_ = false;
        return false;
    }

    const size_t written = file_.write(data, length);
    if (written != length)
    {
        healthy_ = false;
        return false;
    }

    activeFileBytes_ += static_cast<uint32_t>(written);
    return true;
}

bool ProgramFlashFlightLogStorage::flush()
{
    if (!healthy_ || stopped_)
    {
        return false;
    }
    file_.flush();
    return true;
}

void ProgramFlashFlightLogStorage::stop()
{
    if (file_)
    {
        file_.flush();
        file_.close();
    }
    stopped_ = true;
}

bool ProgramFlashFlightLogStorage::healthy() const
{
    return healthy_ && !stopped_;
}

const char *ProgramFlashFlightLogStorage::path() const
{
    return path_;
}

uint64_t ProgramFlashFlightLogStorage::totalBytes() const
{
    return fs_.totalSize();
}

uint64_t ProgramFlashFlightLogStorage::usedBytes() const
{
    return fs_.usedSize();
}

bool ProgramFlashFlightLogStorage::openNextFile()
{
    for (uint16_t index = 0U; index < kMaxLogFiles; ++index)
    {
        makePath(index, path_, sizeof(path_));
        if (fs_.exists(path_))
        {
            continue;
        }

        file_ = fs_.open(path_, FILE_WRITE);
        if (!file_)
        {
            path_[0] = '\0';
            return false;
        }

        activeFileBytes_ = 0U;
        return true;
    }

    if (removeOldestLogFile())
    {
        return openNextFile();
    }

    path_[0] = '\0';
    return false;
}

bool ProgramFlashFlightLogStorage::rotateFile()
{
    if (file_)
    {
        file_.flush();
        file_.close();
    }
    return openNextFile();
}

bool ProgramFlashFlightLogStorage::reclaimSpace(uint32_t bytesNeeded)
{
    const uint64_t total = fs_.totalSize();
    if (total == 0U)
    {
        return true;
    }

    uint64_t used = fs_.usedSize();
    uint64_t freeBytes = total > used ? total - used : 0U;
    while (freeBytes < bytesNeeded)
    {
        if (!removeOldestLogFile())
        {
            return false;
        }
        used = fs_.usedSize();
        freeBytes = total > used ? total - used : 0U;
    }
    return true;
}

bool ProgramFlashFlightLogStorage::removeOldestLogFile()
{
    char candidate[sizeof(path_)] = {};
    for (uint16_t index = 0U; index < kMaxLogFiles; ++index)
    {
        makePath(index, candidate, sizeof(candidate));
        if (strcmp(candidate, path_) == 0)
        {
            continue;
        }
        if (fs_.exists(candidate))
        {
            return fs_.remove(candidate);
        }
    }
    return false;
}

void ProgramFlashFlightLogStorage::makePath(uint16_t index, char *buffer, size_t bufferLength) const
{
    snprintf(buffer, bufferLength, "%s/FL%03u.NLG", directory_, index);
}
