#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nura_protocol_v1_lite.h"

namespace
{
constexpr uint32_t kVehicleId = 0x4E555241UL;
constexpr uint8_t kAuthKey[16] = {
    0x4e, 0x55, 0x52, 0x41, 0x2d, 0x56, 0x31, 0x4c,
    0x49, 0x54, 0x45, 0x2d, 0x54, 0x45, 0x53, 0x54};

bool decodeDownlink(const uint8_t *frame, size_t length, nura::ParsedFrame &parsed)
{
    return nura::decodeFrame(frame,
                             length,
                             kVehicleId,
                             nura::FrameDirection::DOWNLINK,
                             kAuthKey,
                             parsed);
}

bool expectRejected(const uint8_t *frame, size_t length, const char *caseName)
{
    nura::ParsedFrame parsed;
    if (decodeDownlink(frame, length, parsed))
    {
        fprintf(stderr, "accepted invalid frame: %s\n", caseName);
        return false;
    }
    return true;
}

bool checkArmGoldenVector()
{
    static constexpr uint8_t kArmFrame[] = {
        0xaa, 0x55, 0x23, 0x41, 0x52, 0x55, 0x4e, 0x34, 0x12, 0x01, 0x04,
        0x44, 0x33, 0xd4, 0xc3, 0xb2, 0xa1, 0x31, 0xd4, 0x00, 0x00, 0x01,
        0x00, 0x02, 0x00, 0xcf, 0x9d, 0x6d, 0x96, 0x41, 0xa0, 0x11, 0x0a,
        0x13, 0xd5, 0x1b, 0x51, 0xd9, 0xde, 0x6e, 0xcd, 0xd2, 0x3d};
    static_assert(sizeof(kArmFrame) == nura::kFrameOverhead + nura::kControlPayloadLen,
                  "ARM golden frame length drifted");

    nura::ParsedFrame parsed;
    if (!nura::decodeFrame(kArmFrame,
                           sizeof(kArmFrame),
                           kVehicleId,
                           nura::FrameDirection::UPLINK,
                           kAuthKey,
                           parsed))
    {
        fprintf(stderr, "ARM golden frame rejected\n");
        return false;
    }

    nura::ControlPayload command;
    if (!nura::decodeControlPayload(parsed.payload, parsed.payloadLen, command) ||
        parsed.seq != 0x1234U ||
        command.subtype != nura::CONTROL_CMD ||
        command.commandId != nura::COMMAND_ARM_FLIGHT ||
        command.commandSeq != 0x3344U ||
        command.nonce != 0xA1B2C3D4UL ||
        command.validUntilMs != 54321UL ||
        command.param0 != nura::FLIGHT_SAFE ||
        command.param1 != nura::FLIGHT_ARMED ||
        !nura::verifyControlAuthTag(command, parsed.seq, kAuthKey))
    {
        fprintf(stderr, "ARM golden payload contract mismatch\n");
        return false;
    }

    uint8_t encodedPayload[nura::kControlPayloadLen];
    uint8_t encodedFrame[nura::kMaxFrameLen];
    const size_t encodedLength = nura::encodeControlPayload(command, encodedPayload, sizeof(encodedPayload))
                                     ? nura::encodeFrame(nura::MESSAGE_CONTROL,
                                                         kVehicleId,
                                                         parsed.seq,
                                                         nura::FrameDirection::UPLINK,
                                                         kAuthKey,
                                                         encodedPayload,
                                                         sizeof(encodedPayload),
                                                         encodedFrame,
                                                         sizeof(encodedFrame))
                                     : 0U;
    if (encodedLength != sizeof(kArmFrame) ||
        memcmp(encodedFrame, kArmFrame, sizeof(kArmFrame)) != 0)
    {
        fprintf(stderr, "ARM golden frame did not round-trip byte-for-byte\n");
        return false;
    }

    command.authOrAck[0] ^= 0x01U;
    if (nura::verifyControlAuthTag(command, parsed.seq, kAuthKey))
    {
        fprintf(stderr, "ARM control auth tamper accepted\n");
        return false;
    }
    return true;
}
} // namespace

int main()
{
    nura::ControlPayload command;
    command.subtype = nura::CONTROL_ACK;
    command.commandId = nura::COMMAND_FORCE_DEPLOY_RECOVERY;
    command.commandSeq = 42U;
    command.nonce = 0x12345678UL;

    uint8_t payload[nura::kControlPayloadLen];
    if (!nura::encodeControlPayload(command, payload, sizeof(payload)))
    {
        return 1;
    }

    uint8_t frame[nura::kMaxFrameLen + 1U] = {0U};
    const size_t frameLen = nura::encodeFrame(nura::MESSAGE_CONTROL,
                                               kVehicleId,
                                               7U,
                                               nura::FrameDirection::DOWNLINK,
                                               kAuthKey,
                                               payload,
                                               sizeof(payload),
                                               frame,
                                               sizeof(frame));
    nura::ParsedFrame parsed;
    if (frameLen != nura::kMaxFrameLen ||
        !decodeDownlink(frame, frameLen, parsed) ||
        parsed.type != nura::MESSAGE_CONTROL ||
        parsed.vehicleId != kVehicleId ||
        parsed.seq != 7U)
    {
        fprintf(stderr, "valid authenticated frame rejected\n");
        return 1;
    }

    bool ok = checkArmGoldenVector();
    ok = expectRejected(frame, frameLen - 1U, "truncated") && ok;
    frame[frameLen] = 0x00U;
    ok = expectRejected(frame, frameLen + 1U, "trailing byte") && ok;

    nura::ParsedFrame rejected;
    ok = !nura::decodeFrame(frame,
                            frameLen,
                            0x01020304UL,
                            nura::FrameDirection::DOWNLINK,
                            kAuthKey,
                            rejected) &&
         ok;
    ok = !nura::decodeFrame(frame,
                            frameLen,
                            kVehicleId,
                            nura::FrameDirection::UPLINK,
                            kAuthKey,
                            rejected) &&
         ok;

    uint8_t wrongKey[16];
    memcpy(wrongKey, kAuthKey, sizeof(wrongKey));
    wrongKey[0] ^= 0x01U;
    ok = !nura::decodeFrame(frame,
                            frameLen,
                            kVehicleId,
                            nura::FrameDirection::DOWNLINK,
                            wrongKey,
                            rejected) &&
         ok;

    frame[0] ^= 0x01U;
    ok = expectRejected(frame, frameLen, "foreign sync") && ok;
    frame[0] ^= 0x01U;

    frame[2] = static_cast<uint8_t>((1U << 4) | nura::MESSAGE_CONTROL);
    ok = expectRejected(frame, frameLen, "old version") && ok;
    frame[2] = nura::makeVerType(nura::MESSAGE_CONTROL);

    frame[nura::kFrameHeaderLen] ^= 0x01U;
    const uint16_t tamperedCrc = nura::crc16CcittFalse(frame + 2,
                                                       frameLen - 2U - nura::kFrameCrcLen);
    nura::writeU16(frame + frameLen - nura::kFrameCrcLen, tamperedCrc);
    ok = expectRejected(frame, frameLen, "tampered payload with repaired crc") && ok;

    if (!ok)
    {
        fprintf(stderr, "authenticated frame rejection failed\n");
        return 1;
    }

    printf("authenticated protocol frame tests passed\n");
    return 0;
}
