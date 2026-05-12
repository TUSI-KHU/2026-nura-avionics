#include <Arduino.h>
#include <unity.h>

#include "app/app_config.h"
#include "core/logger/log_output.h"
#include "core/logger/logger.h"

namespace
{
class CountingLogOutput : public ILogOutput
{
public:
    bool write(const LogEntry &entry) override
    {
        last_ = entry;
        ++writes_;
        return true;
    }

    uint8_t writes() const
    {
        return writes_;
    }

    const LogEntry &last() const
    {
        return last_;
    }

private:
    LogEntry last_ = {0U, LogLevel::ERROR, nullptr, nullptr};
    uint8_t writes_ = 0U;
};
}

void test_default_config_uses_teensy_safe_defaults()
{
    DefaultAppConfig config;

    TEST_ASSERT_EQUAL_UINT32(115200UL, config.serialBaudRate());
    TEST_ASSERT_EQUAL_UINT8(LED_BUILTIN, config.statusIndicatorPin());
    TEST_ASSERT_EQUAL_UINT8(0x68U, config.imuI2cAddress());
    TEST_ASSERT_EQUAL_UINT32(10U, config.imuTaskPeriodMs());
}

void test_logger_flushes_error_entry()
{
    Logger logger;
    CountingLogOutput output;

    TEST_ASSERT_TRUE(logger.push(123U, LogLevel::ERROR, "test", "ok"));

    const LogFlushResult result = logger.flushTo(output, 1U);

    TEST_ASSERT_EQUAL_UINT8(1U, result.drained);
    TEST_ASSERT_EQUAL_UINT8(0U, result.outputFailures);
    TEST_ASSERT_EQUAL_UINT8(1U, output.writes());
    TEST_ASSERT_EQUAL_UINT32(123U, output.last().ts);
    TEST_ASSERT_EQUAL_STRING("test", output.last().src);
    TEST_ASSERT_EQUAL_STRING("ok", output.last().msg);
}

void setup()
{
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_default_config_uses_teensy_safe_defaults);
    RUN_TEST(test_logger_flushes_error_entry);
    UNITY_END();
}

void loop()
{
}
