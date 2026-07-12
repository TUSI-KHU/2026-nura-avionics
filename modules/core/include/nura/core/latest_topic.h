#pragma once

#include <atomic>
#include <type_traits>

namespace nura::core
{

// A single-attempt lock keeps publisher and reader latency bounded. Contention is
// observable through the false return value instead of blocking a higher-priority task.
template <typename T>
class LatestTopic
{
    static_assert(std::is_trivially_copyable<T>::value,
                  "software-bus payloads must be trivially copyable");

public:
    bool tryPublish(const T &value)
    {
        if (lock_.test_and_set(std::memory_order_acquire))
        {
            publish_contention_.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }

        value_ = value;
        valid_ = true;
        publish_count_.fetch_add(1U, std::memory_order_relaxed);
        lock_.clear(std::memory_order_release);
        return true;
    }

    bool tryRead(T &value) const
    {
        if (lock_.test_and_set(std::memory_order_acquire))
        {
            read_contention_.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }

        if (!valid_)
        {
            lock_.clear(std::memory_order_release);
            return false;
        }

        value = value_;
        read_count_.fetch_add(1U, std::memory_order_relaxed);
        lock_.clear(std::memory_order_release);
        return true;
    }

    uint32_t publishCount() const { return publish_count_.load(std::memory_order_relaxed); }
    uint32_t readCount() const { return read_count_.load(std::memory_order_relaxed); }
    uint32_t publishContention() const { return publish_contention_.load(std::memory_order_relaxed); }
    uint32_t readContention() const { return read_contention_.load(std::memory_order_relaxed); }

private:
    mutable std::atomic_flag lock_ = ATOMIC_FLAG_INIT;
    T value_{};
    bool valid_ = false;
    std::atomic<uint32_t> publish_count_{0U};
    mutable std::atomic<uint32_t> read_count_{0U};
    std::atomic<uint32_t> publish_contention_{0U};
    mutable std::atomic<uint32_t> read_contention_{0U};
};

} // namespace nura::core
