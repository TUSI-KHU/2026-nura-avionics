#include <Arduino.h>
#include <LittleFS.h>

namespace
{
#define LUT0(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)))
#define LUT1(opcode, pads, operand) (FLEXSPI_LUT_INSTRUCTION((opcode), (pads), (operand)) << 16)
#define CMD_SDR FLEXSPI_LUT_OPCODE_CMD_SDR
#define READ_SDR FLEXSPI_LUT_OPCODE_READ_SDR
#define PINS1 FLEXSPI_LUT_NUM_PADS_1

constexpr uint32_t kSerialBaud = 115200UL;
constexpr uint32_t kSerialWaitMs = 5000UL;
constexpr uint32_t kCycleDelayMs = 500UL;
constexpr uint8_t kVerifyCycles = 10U;
constexpr size_t kChunkBytes = 4096U;
constexpr size_t kTestFileBytes = 512U * 1024U;
constexpr uint64_t kExpectedW25q128Bytes = 16ULL * 1024ULL * 1024ULL;
constexpr const char *kTestPath = "/w25q128_verify.bin";

LittleFS_QSPIFlash qspiFlash;
uint8_t writeBuffer[kChunkBytes];
uint8_t readBuffer[kChunkBytes];

bool rawRead(uint8_t lutIndex, void *data, uint32_t length)
{
    memset(data, 0, length);

    FLEXSPI2_INTR = FLEXSPI_INTR_IPRXWA | FLEXSPI_INTR_IPCMDDONE | FLEXSPI_INTR_IPCMDERR;
    FLEXSPI2_IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF | FLEXSPI_IPRXFCR_RXWMRK(1);
    const uint32_t addrOffset = (FLEXSPI2_FLSHA1CR0 & 0x7FFFFFU) << 10U;
    FLEXSPI2_IPCR0 = addrOffset;
    FLEXSPI2_IPCR1 = FLEXSPI_IPCR1_ISEQID(lutIndex) | FLEXSPI_IPCR1_IDATSZ(length);
    FLEXSPI2_IPCMD = FLEXSPI_IPCMD_TRG;

    elapsedMillis timeoutMs = 0;
    while (!(FLEXSPI2_INTR & FLEXSPI_INTR_IPCMDDONE))
    {
        if (timeoutMs > 50U)
        {
            FLEXSPI2_INTR = FLEXSPI_INTR_IPCMDERR;
            return false;
        }
    }

    memcpy(data, (const void *)&FLEXSPI2_RFDR0, length);
    FLEXSPI2_INTR = FLEXSPI_INTR_IPRXWA | FLEXSPI_INTR_IPCMDDONE | FLEXSPI_INTR_IPCMDERR;
    return true;
}

void configureRawJedecIdLut()
{
    FLEXSPI2_LUTKEY = FLEXSPI_LUTKEY_VALUE;
    FLEXSPI2_LUTCR = FLEXSPI_LUTCR_UNLOCK;
    FLEXSPI2_LUT32 = LUT0(CMD_SDR, PINS1, 0x9F) | LUT1(READ_SDR, PINS1, 1);
    FLEXSPI2_LUT33 = 0U;
}

void printHexByte(uint8_t value)
{
    if (value < 0x10U)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

uint8_t patternByte(size_t offset, uint8_t cycle)
{
    uint32_t x = static_cast<uint32_t>(offset);
    x ^= static_cast<uint32_t>(cycle) * 0x45D9F3BU;
    x ^= x >> 16U;
    x *= 0x7FEB352DU;
    x ^= x >> 15U;
    return static_cast<uint8_t>(x & 0xFFU);
}

void fillPattern(size_t absoluteOffset, uint8_t cycle, uint8_t *buffer, size_t length)
{
    for (size_t i = 0; i < length; ++i)
    {
        buffer[i] = patternByte(absoluteOffset + i, cycle);
    }
}

bool verifyBuffer(size_t absoluteOffset, uint8_t cycle, const uint8_t *buffer, size_t length)
{
    for (size_t i = 0; i < length; ++i)
    {
        const uint8_t expected = patternByte(absoluteOffset + i, cycle);
        if (buffer[i] != expected)
        {
            Serial.print("VERIFY_MISMATCH offset=");
            Serial.print(absoluteOffset + i);
            Serial.print(" expected=0x");
            if (expected < 0x10U)
            {
                Serial.print('0');
            }
            Serial.print(expected, HEX);
            Serial.print(" actual=0x");
            if (buffer[i] < 0x10U)
            {
                Serial.print('0');
            }
            Serial.println(buffer[i], HEX);
            return false;
        }
    }
    return true;
}

bool writeTestFile(uint8_t cycle)
{
    qspiFlash.remove(kTestPath);

    File file = qspiFlash.open(kTestPath, FILE_WRITE);
    if (!file)
    {
        Serial.println("WRITE_OPEN_FAIL");
        return false;
    }

    size_t writtenTotal = 0U;
    while (writtenTotal < kTestFileBytes)
    {
        const size_t todo = min(kChunkBytes, kTestFileBytes - writtenTotal);
        fillPattern(writtenTotal, cycle, writeBuffer, todo);
        const size_t written = file.write(writeBuffer, todo);
        if (written != todo)
        {
            Serial.print("WRITE_FAIL offset=");
            Serial.print(writtenTotal);
            Serial.print(" requested=");
            Serial.print(todo);
            Serial.print(" written=");
            Serial.println(written);
            file.close();
            return false;
        }
        writtenTotal += written;
        yield();
    }

    file.flush();
    const uint32_t fileSize = file.size();
    file.close();

    if (fileSize != kTestFileBytes)
    {
        Serial.print("WRITE_SIZE_FAIL expected=");
        Serial.print(kTestFileBytes);
        Serial.print(" actual=");
        Serial.println(fileSize);
        return false;
    }

    return true;
}

bool readBackTestFile(uint8_t cycle)
{
    File file = qspiFlash.open(kTestPath, FILE_READ);
    if (!file)
    {
        Serial.println("READ_OPEN_FAIL");
        return false;
    }

    const uint32_t fileSize = file.size();
    if (fileSize != kTestFileBytes)
    {
        Serial.print("READ_SIZE_FAIL expected=");
        Serial.print(kTestFileBytes);
        Serial.print(" actual=");
        Serial.println(fileSize);
        file.close();
        return false;
    }

    size_t readTotal = 0U;
    while (readTotal < kTestFileBytes)
    {
        const size_t todo = min(kChunkBytes, kTestFileBytes - readTotal);
        const size_t bytesRead = file.read(readBuffer, todo);
        if (bytesRead != todo)
        {
            Serial.print("READ_FAIL offset=");
            Serial.print(readTotal);
            Serial.print(" requested=");
            Serial.print(todo);
            Serial.print(" read=");
            Serial.println(bytesRead);
            file.close();
            return false;
        }
        if (!verifyBuffer(readTotal, cycle, readBuffer, bytesRead))
        {
            file.close();
            return false;
        }
        readTotal += bytesRead;
        yield();
    }

    file.close();
    return true;
}

bool runCycle(uint8_t cycle)
{
    Serial.print("CYCLE_BEGIN ");
    Serial.println(cycle);

    const uint32_t startMs = millis();
    if (!writeTestFile(cycle))
    {
        Serial.print("CYCLE_FAIL ");
        Serial.println(cycle);
        return false;
    }
    if (!readBackTestFile(cycle))
    {
        Serial.print("CYCLE_FAIL ");
        Serial.println(cycle);
        return false;
    }
    if (!qspiFlash.remove(kTestPath))
    {
        Serial.print("REMOVE_FAIL ");
        Serial.println(kTestPath);
        return false;
    }

    Serial.print("CYCLE_PASS ");
    Serial.print(cycle);
    Serial.print(" bytes=");
    Serial.print(kTestFileBytes);
    Serial.print(" elapsed_ms=");
    Serial.println(millis() - startMs);
    return true;
}
} // namespace

void setup()
{
    Serial.begin(kSerialBaud);
    const uint32_t serialStart = millis();
    while (!Serial && millis() - serialStart < kSerialWaitMs)
    {
        delay(10);
    }

    Serial.println();
    Serial.println("W25Q128_QSPI_FLASH_TEST_BEGIN");

    uint8_t jedecId[3] = {};
    configureRawJedecIdLut();
    const bool rawIdOk = rawRead(8U, jedecId, sizeof(jedecId));
    Serial.print("RAW_JEDEC_ID ");
    Serial.print(rawIdOk ? "OK " : "FAIL ");
    printHexByte(jedecId[0]);
    Serial.print(' ');
    printHexByte(jedecId[1]);
    Serial.print(' ');
    printHexByte(jedecId[2]);
    Serial.println();

    if (!qspiFlash.begin())
    {
        Serial.println("FLASH_BEGIN_FAIL");
        return;
    }

    Serial.print("FLASH_NAME ");
    Serial.println(qspiFlash.name());
    Serial.print("FLASH_TOTAL_BYTES ");
    Serial.println(static_cast<unsigned long long>(qspiFlash.totalSize()));
    Serial.print("FLASH_USED_BYTES ");
    Serial.println(static_cast<unsigned long long>(qspiFlash.usedSize()));

    if (qspiFlash.totalSize() != kExpectedW25q128Bytes)
    {
        Serial.print("FLASH_SIZE_UNEXPECTED expected=");
        Serial.print(static_cast<unsigned long long>(kExpectedW25q128Bytes));
        Serial.print(" actual=");
        Serial.println(static_cast<unsigned long long>(qspiFlash.totalSize()));
        return;
    }

    qspiFlash.remove(kTestPath);

    for (uint8_t cycle = 1U; cycle <= kVerifyCycles; ++cycle)
    {
        if (!runCycle(cycle))
        {
            Serial.println("W25Q128_QSPI_READ_WRITE_VERIFY_FAIL");
            return;
        }
        delay(kCycleDelayMs);
    }

    Serial.println("W25Q128_QSPI_READ_WRITE_VERIFY_PASS");
}

void loop()
{
    delay(1000);
}
