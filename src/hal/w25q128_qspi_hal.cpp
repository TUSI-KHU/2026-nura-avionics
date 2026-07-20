#include "w25q128_qspi_hal.h"

#include <Arduino.h>
#include <string.h>

#include "nura_constants.h"

#if defined(__IMXRT1062__)
namespace
{
#define NURA_LUT0(opcode, pads, operand) \
    (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)))
#define NURA_LUT1(opcode, pads, operand) \
    (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)) << 16)

constexpr uint8_t kLutReadId = 8U;
constexpr uint8_t kLutRead = 9U;
constexpr uint8_t kLutWriteEnable = 10U;
constexpr uint8_t kLutPageProgram = 11U;
constexpr uint8_t kLutSectorErase = 12U;
constexpr uint8_t kLutReadStatus = 13U;
constexpr uint8_t kLutResetEnable = 14U;
constexpr uint8_t kLutReset = 15U;

constexpr uint8_t kW25q128ManufacturerId = 0xEFU;
constexpr uint8_t kW25q128MemoryTypeQ = 0x40U;
constexpr uint8_t kW25q128MemoryTypeM = 0x70U;
constexpr uint8_t kW25q128CapacityId = 0x18U;

uint32_t flashAddressOffset()
{
    return (FLEXSPI2_FLSHA1CR0 & 0x7FFFFFU) << 10U;
}

bool elapsed(uint32_t startUs, uint32_t timeoutUs)
{
    return static_cast<uint32_t>(micros() - startUs) >= timeoutUs;
}
} // namespace
#endif

bool W25Q128QspiHAL::begin()
{
#if !defined(__IMXRT1062__)
    return false;
#else
    initialized_ = false;
    memset(jedecId_, 0, sizeof(jedecId_));

    FLEXSPI2_FLSHA2CR0 = NuraConstants::Logger::kFlightLogQspiFlashBytes / 1024UL;
    if (!configureLut())
    {
        return false;
    }

    // The MCU can reset while QSPI power remains present. Finish an erase or
    // program started before reset before issuing commands the busy NOR ignores.
    const uint32_t busyStartUs = micros();
    while (true)
    {
        uint8_t status = 0xFFU;
        if (!readCommand(kLutReadStatus, 0U, &status, 1U))
        {
            return false;
        }
        if ((status & 0x01U) == 0U)
        {
            break;
        }
        if (elapsed(busyStartUs,
                    NuraConstants::Logger::kFlightLogQspiInitEraseTimeoutUs))
        {
            return false;
        }
        delayMicroseconds(100U);
    }

    if (!command(kLutResetEnable, 0U) || !command(kLutReset, 0U))
    {
        return false;
    }
    delayMicroseconds(50U);

    if (!readCommand(kLutReadId, 0U, jedecId_, sizeof(jedecId_)))
    {
        return false;
    }

    const bool supportedType = jedecId_[1] == kW25q128MemoryTypeQ ||
                               jedecId_[1] == kW25q128MemoryTypeM;
    initialized_ = jedecId_[0] == kW25q128ManufacturerId &&
                   supportedType &&
                   jedecId_[2] == kW25q128CapacityId;
    return initialized_;
#endif
}

bool W25Q128QspiHAL::read(uint32_t address, void *data, uint16_t length)
{
#if !defined(__IMXRT1062__)
    (void)address;
    (void)data;
    (void)length;
    return false;
#else
    if (!initialized_ || data == nullptr || length == 0U ||
        address > capacityBytes() || length > (capacityBytes() - address))
    {
        return false;
    }
    return readCommand(kLutRead, address, data, length);
#endif
}

bool W25Q128QspiHAL::ready(bool &isReady)
{
#if !defined(__IMXRT1062__)
    isReady = false;
    return false;
#else
    uint8_t status = 0xFFU;
    if (!initialized_ || !readCommand(kLutReadStatus, 0U, &status, 1U))
    {
        isReady = false;
        return false;
    }
    isReady = (status & 0x01U) == 0U;
    return true;
#endif
}

bool W25Q128QspiHAL::startPageProgram(uint32_t address,
                                     const uint8_t *data,
                                     uint16_t length)
{
#if !defined(__IMXRT1062__)
    (void)address;
    (void)data;
    (void)length;
    return false;
#else
    if (!initialized_ || data == nullptr || length == 0U ||
        length > NuraConstants::Logger::kFlightLogQspiPageBytes ||
        (address % NuraConstants::Logger::kFlightLogQspiPageBytes) + length >
            NuraConstants::Logger::kFlightLogQspiPageBytes ||
        address > capacityBytes() || length > (capacityBytes() - address))
    {
        return false;
    }
    return writeEnable() && writeCommand(kLutPageProgram, address, data, length);
#endif
}

bool W25Q128QspiHAL::startSectorErase(uint32_t address)
{
#if !defined(__IMXRT1062__)
    (void)address;
    return false;
#else
    if (!initialized_ || address >= capacityBytes())
    {
        return false;
    }
    const uint32_t aligned = address -
                             (address % NuraConstants::Logger::kFlightLogQspiEraseSectorBytes);
    return writeEnable() && command(kLutSectorErase, aligned);
#endif
}

bool W25Q128QspiHAL::waitUntilReady(uint32_t timeoutUs)
{
    const uint32_t startUs = micros();
    while (!elapsed(startUs, timeoutUs))
    {
        bool isReady = false;
        if (!ready(isReady))
        {
            return false;
        }
        if (isReady)
        {
            return true;
        }
        delayMicroseconds(100U);
    }
    return false;
}

bool W25Q128QspiHAL::eraseSectorForInit(uint32_t address, uint32_t timeoutUs)
{
    return startSectorErase(address) && waitUntilReady(timeoutUs);
}

bool W25Q128QspiHAL::programForInit(uint32_t address,
                                   const uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeoutUs)
{
    return startPageProgram(address, data, length) && waitUntilReady(timeoutUs);
}

uint32_t W25Q128QspiHAL::capacityBytes() const
{
    return NuraConstants::Logger::kFlightLogQspiFlashBytes;
}

const uint8_t *W25Q128QspiHAL::jedecId() const
{
    return jedecId_;
}

bool W25Q128QspiHAL::configureLut()
{
#if !defined(__IMXRT1062__)
    return false;
#else
    FLEXSPI2_LUTKEY = FLEXSPI_LUTKEY_VALUE;
    FLEXSPI2_LUTCR = FLEXSPI_LUTCR_UNLOCK;

    FLEXSPI2_LUT32 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x9FU) |
                       NURA_LUT1(FLEXSPI_LUT_OPCODE_READ_SDR,
                                 FLEXSPI_LUT_NUM_PADS_1,
                                 1U);
    FLEXSPI2_LUT33 = 0U;

    FLEXSPI2_LUT36 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x03U) |
                       NURA_LUT1(FLEXSPI_LUT_OPCODE_RADDR_SDR,
                                 FLEXSPI_LUT_NUM_PADS_1,
                                 24U);
    FLEXSPI2_LUT37 = NURA_LUT0(FLEXSPI_LUT_OPCODE_READ_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               1U);

    FLEXSPI2_LUT40 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x06U);
    FLEXSPI2_LUT41 = 0U;

    FLEXSPI2_LUT44 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x02U) |
                       NURA_LUT1(FLEXSPI_LUT_OPCODE_RADDR_SDR,
                                 FLEXSPI_LUT_NUM_PADS_1,
                                 24U);
    FLEXSPI2_LUT45 = NURA_LUT0(FLEXSPI_LUT_OPCODE_WRITE_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               1U);

    FLEXSPI2_LUT48 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x20U) |
                       NURA_LUT1(FLEXSPI_LUT_OPCODE_RADDR_SDR,
                                 FLEXSPI_LUT_NUM_PADS_1,
                                 24U);
    FLEXSPI2_LUT49 = 0U;

    FLEXSPI2_LUT52 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x05U) |
                       NURA_LUT1(FLEXSPI_LUT_OPCODE_READ_SDR,
                                 FLEXSPI_LUT_NUM_PADS_1,
                                 1U);
    FLEXSPI2_LUT53 = 0U;

    FLEXSPI2_LUT56 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x66U);
    FLEXSPI2_LUT57 = 0U;
    FLEXSPI2_LUT60 = NURA_LUT0(FLEXSPI_LUT_OPCODE_CMD_SDR,
                               FLEXSPI_LUT_NUM_PADS_1,
                               0x99U);
    FLEXSPI2_LUT61 = 0U;
    return true;
#endif
}

bool W25Q128QspiHAL::command(uint8_t lutIndex, uint32_t address)
{
#if !defined(__IMXRT1062__)
    (void)lutIndex;
    (void)address;
    return false;
#else
    FLEXSPI2_INTR = FLEXSPI_INTR_IPCMDDONE | FLEXSPI_INTR_IPCMDERR;
    FLEXSPI2_IPCR0 = address + flashAddressOffset();
    FLEXSPI2_IPCR1 = FLEXSPI_IPCR1_ISEQID(lutIndex);
    FLEXSPI2_IPCMD = FLEXSPI_IPCMD_TRG;

    const uint32_t startUs = micros();
    while ((FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDDONE) == 0U)
    {
        if (elapsed(startUs, NuraConstants::Logger::kFlightLogQspiIpCommandTimeoutUs))
        {
            return false;
        }
    }
    const bool ok = (FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDERR) == 0U;
    FLEXSPI2_INTR = FLEXSPI_INTR_IPCMDDONE | FLEXSPI_INTR_IPCMDERR;
    return ok;
#endif
}

bool W25Q128QspiHAL::readCommand(uint8_t lutIndex,
                                uint32_t address,
                                void *data,
                                uint16_t length)
{
#if !defined(__IMXRT1062__)
    (void)lutIndex;
    (void)address;
    (void)data;
    (void)length;
    return false;
#else
    if (data == nullptr || length == 0U)
    {
        return false;
    }

    uint8_t *destination = static_cast<uint8_t *>(data);
    uint16_t remaining = length;
    FLEXSPI2_INTR = FLEXSPI_INTR_IPRXWA | FLEXSPI_INTR_IPCMDDONE |
                    FLEXSPI_INTR_IPCMDERR;
    FLEXSPI2_IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF | FLEXSPI_IPRXFCR_RXWMRK(1);
    FLEXSPI2_IPCR0 = address + flashAddressOffset();
    FLEXSPI2_IPCR1 = FLEXSPI_IPCR1_ISEQID(lutIndex) |
                     FLEXSPI_IPCR1_IDATSZ(length);
    FLEXSPI2_IPCMD = FLEXSPI_IPCMD_TRG;

    const uint32_t startUs = micros();
    while (remaining > 0U)
    {
        if (remaining >= 16U && (FLEXSPI2_INTR & FLEXSPI_INTR_IPRXWA) != 0U)
        {
            volatile uint32_t *fifo = &FLEXSPI2_RFDR0;
            memcpy(destination, (const void *)fifo, 16U);
            destination += 16U;
            remaining = static_cast<uint16_t>(remaining - 16U);
            FLEXSPI2_INTR = FLEXSPI_INTR_IPRXWA;
        }
        else if (remaining < 16U &&
                 (FLEXSPI2_IPRXFSTS & 0xFFU) >= ((remaining + 7U) >> 3U))
        {
            volatile uint32_t *fifo = &FLEXSPI2_RFDR0;
            while (remaining >= 4U)
            {
                const uint32_t word = *fifo++;
                memcpy(destination, &word, 4U);
                destination += 4U;
                remaining = static_cast<uint16_t>(remaining - 4U);
            }
            if (remaining > 0U)
            {
                const uint32_t word = *fifo;
                memcpy(destination, &word, remaining);
                remaining = 0U;
            }
        }

        if (elapsed(startUs, NuraConstants::Logger::kFlightLogQspiIpCommandTimeoutUs))
        {
            return false;
        }
    }

    while ((FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDDONE) == 0U)
    {
        if (elapsed(startUs, NuraConstants::Logger::kFlightLogQspiIpCommandTimeoutUs))
        {
            return false;
        }
    }
    const bool ok = (FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDERR) == 0U;
    FLEXSPI2_INTR = FLEXSPI_INTR_IPRXWA | FLEXSPI_INTR_IPCMDDONE |
                    FLEXSPI_INTR_IPCMDERR;
    return ok;
#endif
}

bool W25Q128QspiHAL::writeCommand(uint8_t lutIndex,
                                 uint32_t address,
                                 const uint8_t *data,
                                 uint16_t length)
{
#if !defined(__IMXRT1062__)
    (void)lutIndex;
    (void)address;
    (void)data;
    (void)length;
    return false;
#else
    if (data == nullptr || length == 0U)
    {
        return false;
    }

    FLEXSPI2_INTR = FLEXSPI_INTR_IPTXWE | FLEXSPI_INTR_IPCMDDONE |
                    FLEXSPI_INTR_IPCMDERR;
    FLEXSPI2_IPTXFCR = FLEXSPI_IPTXFCR_CLRIPTXF | FLEXSPI_IPTXFCR_TXWMRK(0);
    FLEXSPI2_IPCR0 = address + flashAddressOffset();
    FLEXSPI2_IPCR1 = FLEXSPI_IPCR1_ISEQID(lutIndex) |
                     FLEXSPI_IPCR1_IDATSZ(length);
    FLEXSPI2_IPCMD = FLEXSPI_IPCMD_TRG;

    const uint32_t startUs = micros();
    const uint8_t *source = data;
    uint16_t remaining = length;
    while ((FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDDONE) == 0U)
    {
        if ((FLEXSPI2_INTR & FLEXSPI_INTR_IPTXWE) != 0U && remaining > 0U)
        {
            const uint16_t chunk = remaining > 8U ? 8U : remaining;
            memcpy((void *)&FLEXSPI2_TFDR0, source, chunk);
            source += chunk;
            remaining = static_cast<uint16_t>(remaining - chunk);
            FLEXSPI2_INTR = FLEXSPI_INTR_IPTXWE;
        }
        if (elapsed(startUs, NuraConstants::Logger::kFlightLogQspiIpCommandTimeoutUs))
        {
            return false;
        }
    }

    const bool ok = remaining == 0U &&
                    (FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDERR) == 0U;
    FLEXSPI2_INTR = FLEXSPI_INTR_IPTXWE | FLEXSPI_INTR_IPCMDDONE |
                    FLEXSPI_INTR_IPCMDERR;
    return ok;
#endif
}

bool W25Q128QspiHAL::writeEnable()
{
#if !defined(__IMXRT1062__)
    return false;
#else
    return command(kLutWriteEnable, 0U);
#endif
}
