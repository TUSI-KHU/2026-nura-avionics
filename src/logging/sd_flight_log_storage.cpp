#include "sd_flight_log_storage.h"

#include <stdio.h>
#include <string.h>

#include "logging/flight_log_record.h"
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
    for (uint8_t month = 0U; month < 12U; ++month)
    {
        if (strncmp(__DATE__, kMonths + (month * 3U), 3U) == 0)
        {
            return static_cast<uint8_t>(month + 1U);
        }
    }
    return 1U;
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

void buildCompileTimeStem(char *stem, size_t length)
{
    if (stem == nullptr || length == 0U)
    {
        return;
    }

    snprintf(stem,
             length,
             "%02u%02u%02u%02u",
             static_cast<unsigned int>(compileMonth()),
             static_cast<unsigned int>(compileDay()),
             static_cast<unsigned int>(compileHour()),
             static_cast<unsigned int>(compileMinute()));
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
            Serial.print("SD_ERROR code=0x");
            Serial.print(SD.sdfs.sdErrorCode(), HEX);
            Serial.print(" data=0x");
            Serial.println(SD.sdfs.sdErrorData(), HEX);
        }
        delay(500);
    }
}

bool mountStorage(uint8_t csPin)
{
#if defined(BUILTIN_SDCARD)
    if (csPin == BUILTIN_SDCARD)
    {
        return SD.sdfs.begin(SdioConfig(FIFO_SDIO));
    }
#endif
    return SD.begin(csPin);
}

uint16_t blockHeaderCrc(const nura_sd_log::BlockHeader &header)
{
    nura_sd_log::BlockHeader copy = header;
    copy.headerCrc16 = 0U;
    return nura_log::crc16Ccitt(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));
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
    flushRequested_ = false;
    logicalBytesWritten_ = 0U;
    physicalBytesWritten_ = 0U;
    blockSequence_ = 0U;
    sessionId_ = 0U;
    busyStartedMs_ = 0U;
    busyObserved_ = false;
    queue_.clear();
    path_[0] = '\0';
    if (file_)
    {
        file_.close();
    }

    bool mounted = false;
    for (uint8_t attempt = 0U; attempt < NuraConstants::Logger::kSdInitRetryAttempts; ++attempt)
    {
        mounted = mountStorage(csPin_);
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
    bool directoryReady = SD.sdfs.exists(directory_);
    for (uint8_t attempt = 0U;
         !directoryReady && attempt < NuraConstants::Logger::kSdInitRetryAttempts;
         ++attempt)
    {
        directoryReady = SD.sdfs.mkdir(directory_) || SD.sdfs.exists(directory_);
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
        traceSdFailure("open_or_preallocate");
    }
    return healthy_;
}

bool SdFlightLogStorage::canAppend(uint16_t length) const
{
    return healthy() && !flushRequested_ && length > 0U && length <= queue_.free();
}

bool SdFlightLogStorage::append(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || !canAppend(length))
    {
        return false;
    }
    return queue_.push(data, length);
}

bool SdFlightLogStorage::service(uint32_t nowMs)
{
    if (!healthy() || stopped_)
    {
        return false;
    }

    if (file_.isBusy())
    {
        if (!busyObserved_)
        {
            busyObserved_ = true;
            busyStartedMs_ = nowMs;
        }
        else if (static_cast<uint32_t>(nowMs - busyStartedMs_) >
                 NuraConstants::Logger::kFlightLogSdBusyTimeoutMs)
        {
            healthy_ = false;
            return false;
        }
        return true;
    }
    busyObserved_ = false;

    const uint16_t sectorBytes = NuraConstants::Logger::kFlightLogSdSectorBytes;
    const uint16_t payloadCapacity = static_cast<uint16_t>(
        sectorBytes - sizeof(nura_sd_log::BlockHeader));
    if (queue_.used() < payloadCapacity && !(flushRequested_ && !queue_.empty()))
    {
        return true;
    }
    if (physicalBytesWritten_ + sectorBytes >
        NuraConstants::Logger::kFlightLogSdPreallocateBytes)
    {
        healthy_ = false;
        return false;
    }

    const uint16_t validBytes = queue_.used() < payloadCapacity
                                    ? queue_.used()
                                    : payloadCapacity;
    makeBlock(validBytes);
    if (validBytes == 0U)
    {
        return true;
    }

    const size_t written = file_.write(sectorBuffer_, sectorBytes);
    if (written != sectorBytes || !queue_.consume(validBytes))
    {
        healthy_ = false;
        return false;
    }
    logicalBytesWritten_ += validBytes;
    physicalBytesWritten_ += sectorBytes;
    ++blockSequence_;
    return true;
}

bool SdFlightLogStorage::requestFlush()
{
    if (!healthy() || stopped_)
    {
        return false;
    }
    flushRequested_ = true;
    return true;
}

bool SdFlightLogStorage::idle() const
{
    return queue_.empty() && (!file_ || !file_.isBusy());
}

void SdFlightLogStorage::stop()
{
    if (idle())
    {
        // The file remains preallocated and open. Closing or truncating would
        // synchronously wait for FAT metadata after the cooperative scheduler starts.
        stopped_ = true;
    }
}

bool SdFlightLogStorage::healthy() const
{
    return healthy_ && !stopped_;
}

const char *SdFlightLogStorage::path() const
{
    return path_;
}

uint32_t SdFlightLogStorage::logicalBytesWritten() const
{
    return logicalBytesWritten_;
}

uint32_t SdFlightLogStorage::sessionId() const
{
    return sessionId_;
}

bool SdFlightLogStorage::openNextFile()
{
    char compileTimeStem[9] = {};
    buildCompileTimeStem(compileTimeStem, sizeof(compileTimeStem));

    for (uint16_t index = 0U; index < 1000U; ++index)
    {
        if (index == 0U)
        {
            snprintf(path_, sizeof(path_), "%s/%s.NLG", directory_, compileTimeStem);
        }
        else
        {
            snprintf(path_,
                     sizeof(path_),
                     "%s/%s_%03u.NLG",
                     directory_,
                     compileTimeStem,
                     static_cast<unsigned int>(index));
        }
        if (SD.sdfs.exists(path_))
        {
            continue;
        }

        file_ = SD.sdfs.open(path_, O_RDWR | O_CREAT | O_TRUNC);
        if (!file_)
        {
            break;
        }
        if (!file_.preAllocate(NuraConstants::Logger::kFlightLogSdPreallocateBytes))
        {
            file_.close();
            break;
        }
        file_.rewind();
        if (!file_.sync())
        {
            file_.close();
            break;
        }
        sessionId_ = makeSessionId(index);
        return true;
    }

    path_[0] = '\0';
    return false;
}

void SdFlightLogStorage::makeBlock(uint16_t payloadLength)
{
    memset(sectorBuffer_, 0xFF, sizeof(sectorBuffer_));
    const uint16_t copied = queue_.peek(
        sectorBuffer_ + sizeof(nura_sd_log::BlockHeader), payloadLength);

    nura_sd_log::BlockHeader header = {};
    header.magic = nura_sd_log::kBlockMagic;
    header.sessionId = sessionId_;
    header.blockSequence = blockSequence_;
    header.streamOffset = logicalBytesWritten_;
    header.payloadLength = copied;
    header.payloadCrc16 = nura_log::crc16Ccitt(
        sectorBuffer_ + sizeof(nura_sd_log::BlockHeader), copied);
    header.version = nura_sd_log::kBlockVersion;
    header.headerCrc16 = blockHeaderCrc(header);
    memcpy(sectorBuffer_, &header, sizeof(header));
}

uint32_t SdFlightLogStorage::makeSessionId(uint16_t fileIndex) const
{
    uint32_t value = 2166136261UL;
    const char buildStamp[] = __DATE__ " " __TIME__;
    for (uint16_t i = 0U; i < sizeof(buildStamp) - 1U; ++i)
    {
        value ^= static_cast<uint8_t>(buildStamp[i]);
        value *= 16777619UL;
    }
    value ^= static_cast<uint32_t>(fileIndex) * 0x9E3779B9UL;
    value ^= micros();
    return value == 0U || value == 0xFFFFFFFFUL ? 0x4E555241UL : value;
}
