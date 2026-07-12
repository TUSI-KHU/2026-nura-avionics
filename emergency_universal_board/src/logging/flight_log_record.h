#pragma once

#include <stddef.h>
#include <stdint.h>

namespace nura_log
{
constexpr uint16_t kFrameMagic = 0x4e4cU; // "NL"
constexpr uint8_t kFrameVersion = 1U;
constexpr size_t kMaxEncodedFrameBytes = 256U;

enum class RecordType : uint8_t
{
    FAST_SAMPLE = 1U,
    SLOW_SAMPLE = 2U,
    EVENT = 3U,
    DECISION = 4U,
};

enum class EventId : uint8_t
{
    BOOT = 1U,
    STATE_TRANSITION = 2U,
    GROUND_STOP = 3U,
    STORAGE_FAULT = 4U,
};

#pragma pack(push, 1)
struct FrameHeader
{
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint16_t payloadLength;
    uint32_t sequence;
    uint32_t timestampMs;
};

struct FastSamplePayload
{
    uint8_t state;
    uint8_t flags;
    uint16_t healthFlags;
    uint32_t decisionSeq;
    uint32_t lowImuUpdatedMs;
    int16_t lowAccelMg[3];
    int16_t lowGyroDps10[3];
    int16_t rollDeg10;
    int16_t pitchDeg10;
    int16_t yawDeg10;
    int16_t tiltDeg10;
    uint32_t highGUpdatedMs;
    int16_t highRaw[3];
    int16_t highAccelMg[3];
    uint32_t baroUpdatedMs;
    int32_t pressurePa;
    int32_t rawAltitudeCm;
    int32_t filteredAltitudeCm;
    uint16_t batteryMv;
};

struct SlowSamplePayload
{
    uint8_t state;
    uint8_t gpsFlags;
    uint16_t healthFlags;
    uint32_t magUpdatedMs;
    int16_t magRaw[3];
    int16_t magUt10[3];
    uint32_t gpsUpdatedMs;
    int32_t latitudeE7;
    int32_t longitudeE7;
    int32_t gpsAltitudeCm;
    int16_t gpsSpeedCms;
    int16_t gpsCourseDeg10;
    uint16_t gpsHdop100;
    uint8_t gpsSatellites;
    uint32_t gpsCharsProcessed;
    uint32_t gpsPassedChecksum;
    uint32_t gpsFailedChecksum;
    uint16_t baroFaultFlags;
    uint8_t baroConsecutiveReadFailCount;
    uint8_t baroConsecutiveBadValueCount;
    uint8_t baroTotalBadValueCount;
};

struct EventPayload
{
    uint8_t eventId;
    uint8_t previousState;
    uint8_t currentState;
    uint8_t reserved;
    uint32_t data0;
    uint32_t data1;
};

struct DecisionPayload
{
    uint32_t decisionSeq;
    uint8_t state;
    uint8_t kind;
    uint8_t result;
    uint8_t count0;
    uint8_t count1;
    uint8_t reserved;
    uint16_t reason;
    float value0;
    float value1;
    float value2;
    float value3;
};
#pragma pack(pop)

uint16_t crc16Ccitt(const uint8_t *data, size_t length);
size_t encodeFrame(RecordType type,
                   uint32_t sequence,
                   uint32_t timestampMs,
                   const void *payload,
                   uint16_t payloadLength,
                   uint8_t *out,
                   size_t outCapacity);
} // namespace nura_log
