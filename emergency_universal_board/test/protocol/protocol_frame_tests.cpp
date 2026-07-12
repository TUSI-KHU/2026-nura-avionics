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

    bool ok = true;
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
