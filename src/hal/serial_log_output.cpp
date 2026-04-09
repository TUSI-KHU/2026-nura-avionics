#include "serial_log_output.h"

#include <string.h>

namespace
{
    uint16_t digits10_u32(uint32_t value)
    {
        uint16_t digits = 1;
        while (value >= 10)
        {
            value /= 10;
            ++digits;
        }
        return digits;
    }
}

void SerialLogOutput::begin(unsigned long baudRate)
{
    Serial.begin(baudRate);
}

bool SerialLogOutput::write(const LogEntry &entry)
{
    if (!Serial)
    {
        return false;
    }

    const char *logLevel = logToString(entry.level);
    if (logLevel == nullptr || entry.src == nullptr || entry.msg == nullptr)
    {
        return false;
    }

    const uint16_t tsLen = digits10_u32(entry.ts);
    const uint16_t levelLen = static_cast<uint16_t>(strlen(logLevel));
    const uint16_t srcLen = static_cast<uint16_t>(strlen(entry.src));
    const uint16_t msgLen = static_cast<uint16_t>(strlen(entry.msg));

    // "[" + ts + "] " + level + " " + src + ": " + msg + "\r\n"
    const uint16_t bytesNeeded = 1U + tsLen + 2U + levelLen + 1U + srcLen + 2U + msgLen + 2U;
    const int available = Serial.availableForWrite();

    if (available < 0)
    {
        return false;
    }

    if (static_cast<uint16_t>(available) < bytesNeeded)
    {
        return false;
    }

    Serial.write('[');
    Serial.print(entry.ts);
    Serial.write("] ");
    Serial.print(logLevel);
    Serial.write(' ');
    Serial.print(entry.src);
    Serial.write(": ");
    Serial.print(entry.msg);
    Serial.write("\r\n");

    return true;
}
