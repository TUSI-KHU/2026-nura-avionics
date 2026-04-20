#pragma once
#include <stdint.h>

#define LOGGER_LEVEL_ERROR 0
#define LOGGER_LEVEL_WARN 1
#define LOGGER_LEVEL_INFO 2
#define LOGGER_LEVEL_DEBUG 3

#ifndef LOG_LEVEL
#define LOG_LEVEL LOGGER_LEVEL_DEBUG
#endif

enum class LogLevel : uint8_t
{
    ERROR = LOGGER_LEVEL_ERROR,
    WARN = LOGGER_LEVEL_WARN,
    INFO = LOGGER_LEVEL_INFO,
    DEBUG = LOGGER_LEVEL_DEBUG
};

inline const char *logToString(LogLevel level)
{
    switch (level)
    {
    case LogLevel::ERROR:
        return "ERROR";
    case LogLevel::WARN:
        return "WARN";
    case LogLevel::INFO:
        return "INFO";
    case LogLevel::DEBUG:
        return "DEBUG";
    default:
        return "UNKNOWN";
    }
}

struct LogEntry
{
    // 로그는 문자열 포맷 결과가 아니라 경량 엔트리로 큐에 저장된다.
    uint32_t ts;
    LogLevel level;
    const char *src;
    const char *msg;
};

class Logger
{
public:
    static const uint8_t kMaxBufferSize = 32;

    Logger();

    // Task들은 push만 호출하고 실제 로그 출력은 LoggerTask가 담당한다.
    bool push(const LogEntry &e);
    bool push(
        uint32_t ts,
        LogLevel level,
        const char *src,
        const char *msg);

    bool pop(LogEntry &out);

    bool empty() const;
    uint8_t size() const;
    uint32_t droppedCount() const;

    static bool isLevelEnabled(LogLevel level);

    void clear();

private:
    static uint8_t nextIndex(uint8_t index);

    LogEntry buffer_[kMaxBufferSize];
    uint8_t head_;
    uint8_t tail_;
    uint8_t count_;
    uint32_t dropped_;
};

#if LOG_LEVEL >= LOGGER_LEVEL_ERROR
#define LOGE(logger, ts, src, msg) (logger).push((ts), LogLevel::ERROR, (src), (msg))
#else
#define LOGE(logger, ts, src, msg) ((void)0)
#endif

#if LOG_LEVEL >= LOGGER_LEVEL_WARN
#define LOGW(logger, ts, src, msg) (logger).push((ts), LogLevel::WARN, (src), (msg))
#else
#define LOGW(logger, ts, src, msg) ((void)0)
#endif

#if LOG_LEVEL >= LOGGER_LEVEL_INFO
#define LOGI(logger, ts, src, msg) (logger).push((ts), LogLevel::INFO, (src), (msg))
#else
#define LOGI(logger, ts, src, msg) ((void)0)
#endif

#if LOG_LEVEL >= LOGGER_LEVEL_DEBUG
#define LOGD(logger, ts, src, msg) (logger).push((ts), LogLevel::DEBUG, (src), (msg))
#else
#define LOGD(logger, ts, src, msg) ((void)0)
#endif
