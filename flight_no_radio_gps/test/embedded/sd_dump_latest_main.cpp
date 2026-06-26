#include <Arduino.h>
#include <SD.h>

namespace
{
constexpr uint32_t kBaud = 115200UL;
bool dumped = false;

bool mountSd()
{
#if defined(BUILTIN_SDCARD)
    return SD.sdfs.begin(SdioConfig(FIFO_SDIO));
#else
    return SD.begin(10);
#endif
}

bool findLatestLog(char (&path)[32])
{
    path[0] = '\0';
    for (uint16_t index = 0U; index < 1000U; ++index)
    {
        char candidate[32] = {};
        snprintf(candidate, sizeof(candidate), "/NURA_LOG/FL%03u.NLG", index);
        if (SD.exists(candidate))
        {
            snprintf(path, sizeof(candidate), "%s", candidate);
        }
    }
    return path[0] != '\0';
}

void dumpFile(const char *path)
{
    File file = SD.open(path, FILE_READ);
    if (!file)
    {
        Serial.print("SD_DUMP_HEX_FAIL open ");
        Serial.println(path);
        return;
    }

    Serial.print("SD_DUMP_HEX_BEGIN ");
    Serial.print(path);
    Serial.print(" size=");
    Serial.println(file.size());

    static constexpr char kHex[] = "0123456789ABCDEF";
    uint8_t buffer[32];
    char line[65];
    line[64] = '\0';

    while (file.available())
    {
        const int readBytes = file.read(buffer, sizeof(buffer));
        if (readBytes <= 0)
        {
            break;
        }

        for (int i = 0; i < readBytes; ++i)
        {
            line[i * 2] = kHex[(buffer[i] >> 4) & 0x0F];
            line[(i * 2) + 1] = kHex[buffer[i] & 0x0F];
        }
        line[readBytes * 2] = '\0';
        Serial.println(line);
    }

    file.close();
    Serial.println("SD_DUMP_HEX_END");
}
} // namespace

void dumpLatestLog()
{
    Serial.println("SD_DUMP_READY");

    if (!mountSd())
    {
        Serial.print("SD_DUMP_HEX_FAIL mount code=0x");
        Serial.print(SD.sdfs.sdErrorCode(), HEX);
        Serial.print(" data=0x");
        Serial.println(SD.sdfs.sdErrorData(), HEX);
        return;
    }

    char latestPath[32];
    if (!findLatestLog(latestPath))
    {
        Serial.println("SD_DUMP_HEX_FAIL no_log_file");
        return;
    }

    dumpFile(latestPath);
}

void setup()
{
    Serial.begin(kBaud);
}

void loop()
{
    if (Serial && !dumped)
    {
        dumped = true;
        dumpLatestLog();
    }
    delay(100);
}
