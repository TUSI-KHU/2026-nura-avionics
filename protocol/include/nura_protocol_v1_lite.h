#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace nura
{
static constexpr uint8_t kSync0 = 0xAAU;
static constexpr uint8_t kSync1 = 0x55U;
static constexpr uint8_t kVersion = 1U;

static constexpr uint8_t kFastPayloadLen = 22U;
static constexpr uint8_t kGpsPayloadLen = 18U;
static constexpr uint8_t kControlPayloadLen = 24U;
static constexpr uint8_t kFrameOverhead = 7U;
static constexpr uint8_t kMaxPayloadLen = kControlPayloadLen;
static constexpr uint8_t kMaxFrameLen = kFrameOverhead + kMaxPayloadLen;

enum MessageType : uint8_t
{
    MESSAGE_FAST_TLM = 0x1U,
    MESSAGE_GPS_TLM = 0x2U,
    MESSAGE_CONTROL = 0x3U,
};

enum ControlSubtype : uint8_t
{
    CONTROL_CMD = 0x01U,
    CONTROL_ACK = 0x81U,
};

enum CommandId : uint8_t
{
    COMMAND_FORCE_DEPLOY_RECOVERY = 0x01U,
    COMMAND_ABORT_PROPULSION_DEPRECATED = 0x02U,
    COMMAND_SET_TELEMETRY_PROFILE = 0x03U,
};

enum AckStage : uint8_t
{
    ACK_RECEIVED = 0U,
    ACK_ACCEPTED = 1U,
    ACK_EXECUTED = 2U,
    ACK_REJECTED = 3U,
    ACK_DUPLICATE = 4U,
};

enum ResultCode : uint8_t
{
    RESULT_OK = 0U,
    RESULT_AUTH_FAILED = 1U,
    RESULT_EXPIRED = 2U,
    RESULT_BAD_FORMAT = 3U,
    RESULT_BAD_STATE = 4U,
    RESULT_NOT_ARMED = 5U,
    RESULT_ALREADY_DONE = 6U,
    RESULT_NOT_SUPPORTED = 7U,
    RESULT_ACTUATOR_FAULT = 8U,
    RESULT_INTERNAL_ERROR = 9U,
};

enum RejectReason : uint8_t
{
    REJECT_NONE = 0U,
    REJECT_COMMAND_EXPIRED = 1U,
    REJECT_UNKNOWN_COMMAND = 2U,
    REJECT_AUTH_TAG_MISMATCH = 3U,
    REJECT_DUPLICATE_OLDER_COMMAND = 4U,
    REJECT_DEPLOYMENT_INHIBITED = 5U,
    REJECT_CONTINUITY_BAD = 6U,
    REJECT_STATE_REJECTED = 7U,
    REJECT_DEPRECATED_COMMAND = 8U,
    REJECT_PROFILE_REJECTED = 9U,
};

enum FlightStateCode : uint8_t
{
    FLIGHT_BOOT = 0U,
    FLIGHT_IDLE = 1U,
    FLIGHT_ARMED = 2U,
    FLIGHT_LAUNCH = 3U,
    FLIGHT_DESCENT = 4U,
    FLIGHT_GROUND = 5U,
    FLIGHT_SAFE = 6U,
};

enum StatusBit : uint16_t
{
    STATUS_LOW_IMU_OK = 1U << 0,
    STATUS_HIGH_ACCEL_OK = 1U << 1,
    STATUS_BARO_OK = 1U << 2,
    STATUS_GPS_OK = 1U << 3,
    STATUS_MAG_OK = 1U << 4,
    STATUS_STORAGE_OK = 1U << 5,
    STATUS_PYRO_CONTINUITY_OK = 1U << 6,
    STATUS_RADIO_OK = 1U << 7,
    STATUS_ARMED = 1U << 12,
    STATUS_ABORT_ACTIVE = 1U << 13,
    STATUS_DEPLOY_FIRED = 1U << 14,
    STATUS_CRITICAL_FAULT = 1U << 15,
};

struct FastTelemetry
{
    uint16_t statusWord = 0U;
    uint32_t bootMs = 0UL;
    int16_t baroDp2Pa = 0;
    int16_t lowAccelXCg = 0;
    int16_t lowAccelYCg = 0;
    int16_t lowAccelZCg = 0;
    int16_t gyroXDps10 = 0;
    int16_t gyroYDps10 = 0;
    int16_t gyroZDps10 = 0;
    uint16_t battMv = 0U;
};

struct GpsTelemetry
{
    int32_t latitudeE7 = 0L;
    int32_t longitudeE7 = 0L;
    int16_t gpsAltDm = 0;
    uint16_t speedCms = 0U;
    uint16_t courseCdeg = 0U;
    uint8_t hdopX10 = 0U;
    uint8_t satellites = 0U;
    uint8_t fixFlags = 0U;
    uint8_t age100Ms = 0U;
};

struct ControlPayload
{
    uint8_t subtype = 0U;
    uint8_t commandId = 0U;
    uint16_t commandSeq = 0U;
    uint32_t nonce = 0UL;
    uint32_t validUntilMs = 0UL;
    int16_t param0 = 0;
    int16_t param1 = 0;
    uint8_t authOrAck[8] = {0};
};

struct ParsedFrame
{
    uint8_t type = 0U;
    uint16_t seq = 0U;
    uint8_t payloadLen = 0U;
    uint8_t payload[kMaxPayloadLen] = {0};
};

inline uint8_t makeVerType(uint8_t type)
{
    return static_cast<uint8_t>((kVersion << 4) | (type & 0x0FU));
}

inline uint8_t frameVersion(uint8_t verType)
{
    return static_cast<uint8_t>(verType >> 4);
}

inline uint8_t frameType(uint8_t verType)
{
    return static_cast<uint8_t>(verType & 0x0FU);
}

inline uint8_t payloadLengthForType(uint8_t type)
{
    switch (type)
    {
    case MESSAGE_FAST_TLM:
        return kFastPayloadLen;
    case MESSAGE_GPS_TLM:
        return kGpsPayloadLen;
    case MESSAGE_CONTROL:
        return kControlPayloadLen;
    default:
        return 0U;
    }
}

inline void writeU16(uint8_t *out, uint16_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFU);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
}

inline void writeI16(uint8_t *out, int16_t value)
{
    writeU16(out, static_cast<uint16_t>(value));
}

inline void writeU32(uint8_t *out, uint32_t value)
{
    out[0] = static_cast<uint8_t>(value & 0xFFUL);
    out[1] = static_cast<uint8_t>((value >> 8) & 0xFFUL);
    out[2] = static_cast<uint8_t>((value >> 16) & 0xFFUL);
    out[3] = static_cast<uint8_t>((value >> 24) & 0xFFUL);
}

inline void writeI32(uint8_t *out, int32_t value)
{
    writeU32(out, static_cast<uint32_t>(value));
}

inline uint16_t readU16(const uint8_t *in)
{
    return static_cast<uint16_t>(in[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(in[1]) << 8);
}

inline int16_t readI16(const uint8_t *in)
{
    return static_cast<int16_t>(readU16(in));
}

inline uint32_t readU32(const uint8_t *in)
{
    return static_cast<uint32_t>(in[0]) |
           (static_cast<uint32_t>(in[1]) << 8) |
           (static_cast<uint32_t>(in[2]) << 16) |
           (static_cast<uint32_t>(in[3]) << 24);
}

inline int32_t readI32(const uint8_t *in)
{
    return static_cast<int32_t>(readU32(in));
}

inline uint16_t crc16CcittFalse(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

inline bool encodeFastPayload(const FastTelemetry &value, uint8_t *out, size_t capacity)
{
    if (out == nullptr || capacity < kFastPayloadLen)
    {
        return false;
    }
    writeU16(out + 0, value.statusWord);
    writeU32(out + 2, value.bootMs);
    writeI16(out + 6, value.baroDp2Pa);
    writeI16(out + 8, value.lowAccelXCg);
    writeI16(out + 10, value.lowAccelYCg);
    writeI16(out + 12, value.lowAccelZCg);
    writeI16(out + 14, value.gyroXDps10);
    writeI16(out + 16, value.gyroYDps10);
    writeI16(out + 18, value.gyroZDps10);
    writeU16(out + 20, value.battMv);
    return true;
}

inline bool decodeFastPayload(const uint8_t *in, size_t length, FastTelemetry &out)
{
    if (in == nullptr || length != kFastPayloadLen)
    {
        return false;
    }
    out.statusWord = readU16(in + 0);
    out.bootMs = readU32(in + 2);
    out.baroDp2Pa = readI16(in + 6);
    out.lowAccelXCg = readI16(in + 8);
    out.lowAccelYCg = readI16(in + 10);
    out.lowAccelZCg = readI16(in + 12);
    out.gyroXDps10 = readI16(in + 14);
    out.gyroYDps10 = readI16(in + 16);
    out.gyroZDps10 = readI16(in + 18);
    out.battMv = readU16(in + 20);
    return true;
}

inline bool encodeGpsPayload(const GpsTelemetry &value, uint8_t *out, size_t capacity)
{
    if (out == nullptr || capacity < kGpsPayloadLen)
    {
        return false;
    }
    writeI32(out + 0, value.latitudeE7);
    writeI32(out + 4, value.longitudeE7);
    writeI16(out + 8, value.gpsAltDm);
    writeU16(out + 10, value.speedCms);
    writeU16(out + 12, value.courseCdeg);
    out[14] = value.hdopX10;
    out[15] = value.satellites;
    out[16] = value.fixFlags;
    out[17] = value.age100Ms;
    return true;
}

inline bool decodeGpsPayload(const uint8_t *in, size_t length, GpsTelemetry &out)
{
    if (in == nullptr || length != kGpsPayloadLen)
    {
        return false;
    }
    out.latitudeE7 = readI32(in + 0);
    out.longitudeE7 = readI32(in + 4);
    out.gpsAltDm = readI16(in + 8);
    out.speedCms = readU16(in + 10);
    out.courseCdeg = readU16(in + 12);
    out.hdopX10 = in[14];
    out.satellites = in[15];
    out.fixFlags = in[16];
    out.age100Ms = in[17];
    return true;
}

inline bool encodeControlPayload(const ControlPayload &value, uint8_t *out, size_t capacity)
{
    if (out == nullptr || capacity < kControlPayloadLen)
    {
        return false;
    }
    out[0] = value.subtype;
    out[1] = value.commandId;
    writeU16(out + 2, value.commandSeq);
    writeU32(out + 4, value.nonce);
    writeU32(out + 8, value.validUntilMs);
    writeI16(out + 12, value.param0);
    writeI16(out + 14, value.param1);
    memcpy(out + 16, value.authOrAck, sizeof(value.authOrAck));
    return true;
}

inline bool decodeControlPayload(const uint8_t *in, size_t length, ControlPayload &out)
{
    if (in == nullptr || length != kControlPayloadLen)
    {
        return false;
    }
    out.subtype = in[0];
    out.commandId = in[1];
    out.commandSeq = readU16(in + 2);
    out.nonce = readU32(in + 4);
    out.validUntilMs = readU32(in + 8);
    out.param0 = readI16(in + 12);
    out.param1 = readI16(in + 14);
    memcpy(out.authOrAck, in + 16, sizeof(out.authOrAck));
    return true;
}

inline size_t encodeFrame(uint8_t type, uint16_t seq, const uint8_t *payload, uint8_t payloadLen, uint8_t *out, size_t capacity)
{
    const uint8_t expectedPayloadLen = payloadLengthForType(type);
    const size_t frameLen = static_cast<size_t>(kFrameOverhead) + expectedPayloadLen;
    if (out == nullptr || payload == nullptr || expectedPayloadLen == 0U ||
        payloadLen != expectedPayloadLen || capacity < frameLen)
    {
        return 0U;
    }

    out[0] = kSync0;
    out[1] = kSync1;
    out[2] = makeVerType(type);
    writeU16(out + 3, seq);
    memcpy(out + 5, payload, expectedPayloadLen);

    const uint16_t crc = crc16CcittFalse(out + 2, static_cast<size_t>(1U + 2U + expectedPayloadLen));
    writeU16(out + 5 + expectedPayloadLen, crc);
    return frameLen;
}

inline uint64_t rotateLeft64(uint64_t value, uint8_t shift)
{
    return (value << shift) | (value >> (64U - shift));
}

inline uint64_t readU64Le(const uint8_t *in)
{
    return static_cast<uint64_t>(in[0]) |
           (static_cast<uint64_t>(in[1]) << 8) |
           (static_cast<uint64_t>(in[2]) << 16) |
           (static_cast<uint64_t>(in[3]) << 24) |
           (static_cast<uint64_t>(in[4]) << 32) |
           (static_cast<uint64_t>(in[5]) << 40) |
           (static_cast<uint64_t>(in[6]) << 48) |
           (static_cast<uint64_t>(in[7]) << 56);
}

inline void sipRound(uint64_t &v0, uint64_t &v1, uint64_t &v2, uint64_t &v3)
{
    v0 += v1;
    v1 = rotateLeft64(v1, 13);
    v1 ^= v0;
    v0 = rotateLeft64(v0, 32);
    v2 += v3;
    v3 = rotateLeft64(v3, 16);
    v3 ^= v2;
    v0 += v3;
    v3 = rotateLeft64(v3, 21);
    v3 ^= v0;
    v2 += v1;
    v1 = rotateLeft64(v1, 17);
    v1 ^= v2;
    v2 = rotateLeft64(v2, 32);
}

inline uint64_t sipHash24(const uint8_t *data, size_t length, const uint8_t key[16])
{
    const uint64_t k0 = readU64Le(key);
    const uint64_t k1 = readU64Le(key + 8);
    uint64_t v0 = 0x736f6d6570736575ULL ^ k0;
    uint64_t v1 = 0x646f72616e646f6dULL ^ k1;
    uint64_t v2 = 0x6c7967656e657261ULL ^ k0;
    uint64_t v3 = 0x7465646279746573ULL ^ k1;

    size_t offset = 0U;
    while ((offset + 8U) <= length)
    {
        const uint64_t m = readU64Le(data + offset);
        v3 ^= m;
        sipRound(v0, v1, v2, v3);
        sipRound(v0, v1, v2, v3);
        v0 ^= m;
        offset += 8U;
    }

    uint64_t b = static_cast<uint64_t>(length) << 56;
    const size_t remaining = length - offset;
    for (size_t i = 0U; i < remaining; ++i)
    {
        b |= static_cast<uint64_t>(data[offset + i]) << (8U * i);
    }

    v3 ^= b;
    sipRound(v0, v1, v2, v3);
    sipRound(v0, v1, v2, v3);
    v0 ^= b;
    v2 ^= 0xFFU;
    sipRound(v0, v1, v2, v3);
    sipRound(v0, v1, v2, v3);
    sipRound(v0, v1, v2, v3);
    sipRound(v0, v1, v2, v3);
    return v0 ^ v1 ^ v2 ^ v3;
}

inline void writeU64Le(uint8_t *out, uint64_t value)
{
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        out[i] = static_cast<uint8_t>((value >> (8U * i)) & 0xFFU);
    }
}

inline void makeControlAuthTag(const ControlPayload &control, uint16_t frameSeq, const uint8_t key[16], uint8_t out[8])
{
    uint8_t input[19];
    input[0] = makeVerType(MESSAGE_CONTROL);
    writeU16(input + 1, frameSeq);
    input[3] = control.subtype;
    input[4] = control.commandId;
    writeU16(input + 5, control.commandSeq);
    writeU32(input + 7, control.nonce);
    writeU32(input + 11, control.validUntilMs);
    writeI16(input + 15, control.param0);
    writeI16(input + 17, control.param1);
    writeU64Le(out, sipHash24(input, sizeof(input), key));
}

inline bool verifyControlAuthTag(const ControlPayload &control, uint16_t frameSeq, const uint8_t key[16])
{
    uint8_t expected[8];
    makeControlAuthTag(control, frameSeq, key, expected);
    uint8_t diff = 0U;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        diff |= static_cast<uint8_t>(expected[i] ^ control.authOrAck[i]);
    }
    return diff == 0U;
}

class Parser
{
public:
    bool feed(uint8_t byte, ParsedFrame &out)
    {
        switch (state_)
        {
        case State::ScanSync0:
            if (byte == kSync0)
            {
                buffer_[0] = byte;
                state_ = State::ScanSync1;
            }
            return false;

        case State::ScanSync1:
            if (byte == kSync1)
            {
                buffer_[1] = byte;
                index_ = 2U;
                state_ = State::ReadType;
            }
            else if (byte != kSync0)
            {
                reset();
            }
            return false;

        case State::ReadType:
        {
            buffer_[index_++] = byte;
            const uint8_t version = frameVersion(byte);
            const uint8_t type = frameType(byte);
            payloadLen_ = payloadLengthForType(type);
            if (version != kVersion || payloadLen_ == 0U)
            {
                reset();
                if (byte == kSync0)
                {
                    buffer_[0] = byte;
                    state_ = State::ScanSync1;
                }
                return false;
            }
            state_ = State::ReadSeq0;
            return false;
        }

        case State::ReadSeq0:
            buffer_[index_++] = byte;
            state_ = State::ReadSeq1;
            return false;

        case State::ReadSeq1:
            buffer_[index_++] = byte;
            payloadRead_ = 0U;
            state_ = payloadLen_ == 0U ? State::ReadCrc0 : State::ReadPayload;
            return false;

        case State::ReadPayload:
            buffer_[index_++] = byte;
            ++payloadRead_;
            if (payloadRead_ >= payloadLen_)
            {
                state_ = State::ReadCrc0;
            }
            return false;

        case State::ReadCrc0:
            buffer_[index_++] = byte;
            state_ = State::ReadCrc1;
            return false;

        case State::ReadCrc1:
        {
            buffer_[index_++] = byte;
            const uint16_t receivedCrc = readU16(buffer_ + index_ - 2U);
            const uint16_t computedCrc = crc16CcittFalse(buffer_ + 2, static_cast<size_t>(1U + 2U + payloadLen_));
            const bool valid = receivedCrc == computedCrc;
            if (valid)
            {
                out.type = frameType(buffer_[2]);
                out.seq = readU16(buffer_ + 3);
                out.payloadLen = payloadLen_;
                memcpy(out.payload, buffer_ + 5, payloadLen_);
            }
            reset();
            return valid;
        }
        }

        reset();
        return false;
    }

    void reset()
    {
        state_ = State::ScanSync0;
        index_ = 0U;
        payloadLen_ = 0U;
        payloadRead_ = 0U;
    }

private:
    enum class State : uint8_t
    {
        ScanSync0,
        ScanSync1,
        ReadType,
        ReadSeq0,
        ReadSeq1,
        ReadPayload,
        ReadCrc0,
        ReadCrc1,
    };

    State state_ = State::ScanSync0;
    uint8_t buffer_[kMaxFrameLen] = {0};
    uint8_t index_ = 0U;
    uint8_t payloadLen_ = 0U;
    uint8_t payloadRead_ = 0U;
};

inline uint16_t statusWithFlightState(uint16_t status, uint8_t flightState)
{
    return static_cast<uint16_t>((status & 0xF0FFU) | ((static_cast<uint16_t>(flightState) & 0x0FU) << 8));
}

inline uint8_t flightStateFromStatus(uint16_t status)
{
    return static_cast<uint8_t>((status >> 8) & 0x0FU);
}
} // namespace nura
