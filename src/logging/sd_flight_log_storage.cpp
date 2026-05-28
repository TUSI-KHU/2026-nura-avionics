#include "sd_flight_log_storage.h"

#include <stdio.h>

SdFlightLogStorage::SdFlightLogStorage(uint8_t csPin, const char *directory)
    : csPin_(csPin),
      directory_(directory)
{
}

bool SdFlightLogStorage::begin()
{
    stopped_ = false;
    healthy_ = false;
    path_[0] = '\0';

    if (!SD.begin(csPin_))
    {
        return false;
    }

    if (!SD.exists(directory_) && !SD.mkdir(directory_))
    {
        return false;
    }

    healthy_ = openNextFile();
    return healthy_;
}

bool SdFlightLogStorage::append(const uint8_t *data, uint16_t length)
{
    if (!healthy_ || stopped_ || data == nullptr || length == 0U)
    {
        return false;
    }

    const size_t written = file_.write(data, length);
    if (written != length)
    {
        healthy_ = false;
        return false;
    }

    return true;
}

bool SdFlightLogStorage::flush()
{
    if (!healthy_ || stopped_)
    {
        return false;
    }
    file_.flush();
    return true;
}

void SdFlightLogStorage::stop()
{
    if (file_)
    {
        file_.flush();
        file_.close();
    }
    stopped_ = true;
}

bool SdFlightLogStorage::healthy() const
{
    return healthy_ && !stopped_;
}

const char *SdFlightLogStorage::path() const
{
    return path_;
}

bool SdFlightLogStorage::openNextFile()
{
    for (uint16_t index = 0U; index < 1000U; ++index)
    {
        snprintf(path_, sizeof(path_), "%s/FL%03u.NLG", directory_, index);
        if (SD.exists(path_))
        {
            continue;
        }

        file_ = SD.open(path_, FILE_WRITE);
        return static_cast<bool>(file_);
    }

    path_[0] = '\0';
    return false;
}
