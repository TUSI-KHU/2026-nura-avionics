#include "sd_flight_log_storage.h"

#include <stdio.h>
#include <string.h>

#include "nura_constants.h"

namespace
{
uint8_t digit(char value)
{
    return value >= '0' && value <= '9' ? static_cast<uint8_t>(value - '0') : 0U;
}

uint8_t compileMonth()
{
    static constexpr char kMonths[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *month = strstr(kMonths, __DATE__);
    return month == nullptr ? 1U : static_cast<uint8_t>(((month - kMonths) / 3) + 1);
}

uint8_t compileDay()
{
    const uint8_t day = static_cast<uint8_t>(digit(__DATE__[4]) * 10U + digit(__DATE__[5]));
    return day == 0U ? 1U : day;
}

uint16_t compileYear()
{
    return static_cast<uint16_t>(digit(__DATE__[7]) * 1000U +
                                 digit(__DATE__[8]) * 100U +
                                 digit(__DATE__[9]) * 10U +
                                 digit(__DATE__[10]));
}

uint8_t compileHour()
{
    return static_cast<uint8_t>(digit(__TIME__[0]) * 10U + digit(__TIME__[1]));
}

uint8_t compileMinute()
{
    return static_cast<uint8_t>(digit(__TIME__[3]) * 10U + digit(__TIME__[4]));
}

uint8_t compileSecond()
{
    return static_cast<uint8_t>(digit(__TIME__[6]) * 10U + digit(__TIME__[7]));
}

void buildDateTime(uint16_t *date, uint16_t *time)
{
    if (date != nullptr)
    {
        *date = FS_DATE(compileYear(), compileMonth(), compileDay());
    }
    if (time != nullptr)
    {
        *time = FS_TIME(compileHour(), compileMinute(), compileSecond());
    }
}

void traceSdFailure(const char *reason)
{
    const uint32_t startMs = millis();
    while ((millis() - startMs) < 5000UL)
    {
        if (Serial)
        {
            Serial.print("SD_INIT_FAIL ");
            Serial.println(reason);
        }
        delay(500);
    }
}
} // namespace

SdFlightLogStorage::SdFlightLogStorage(uint8_t csPin, const char *directory)
    : csPin_(csPin),
      directory_(directory)
{
}

bool SdFlightLogStorage::begin()
{
    if (healthy())
    {
        return true;
    }

    stopped_ = false;
    healthy_ = false;
    path_[0] = '\0';
    if (file_)
    {
        file_.close();
    }

    bool mounted = false;
    for (uint8_t attempt = 0U; attempt < NuraConstants::Logger::kSdInitRetryAttempts; ++attempt)
    {
        mounted = SD.begin(csPin_);
        if (mounted)
        {
            break;
        }
        delay(NuraConstants::Logger::kSdInitRetryDelayMs);
    }

    if (!mounted)
    {
        traceSdFailure("mount");
        return false;
    }
    FsDateTime::setCallback(buildDateTime);

    bool directoryReady = SD.exists(directory_);
    for (uint8_t attempt = 0U; !directoryReady && attempt < NuraConstants::Logger::kSdInitRetryAttempts; ++attempt)
    {
        directoryReady = SD.mkdir(directory_) || SD.exists(directory_);
        if (!directoryReady)
        {
            delay(NuraConstants::Logger::kSdInitRetryDelayMs);
        }
    }

    if (!directoryReady)
    {
        traceSdFailure("directory");
        return false;
    }

    healthy_ = openNextFile();
    if (!healthy_)
    {
        traceSdFailure("open_file");
    }
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
