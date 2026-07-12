#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace nura::core
{

template <typename T, size_t Capacity>
class SpscQueue
{
    static_assert(Capacity >= 2U, "SPSC queue needs at least two slots");
    static_assert(std::is_trivially_copyable<T>::value,
                  "software-bus queue payloads must be trivially copyable");

public:
    bool tryPush(const T &value)
    {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next = increment(head);
        if (next == tail_.load(std::memory_order_acquire))
        {
            dropped_.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }

        entries_[head] = value;
        head_.store(next, std::memory_order_release);

        const uint32_t depth_now = static_cast<uint32_t>(depth());
        uint32_t observed = high_water_.load(std::memory_order_relaxed);
        while (depth_now > observed &&
               !high_water_.compare_exchange_weak(observed, depth_now,
                                                  std::memory_order_relaxed,
                                                  std::memory_order_relaxed))
        {
        }
        return true;
    }

    bool tryPop(T &value)
    {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire))
        {
            return false;
        }

        value = entries_[tail];
        tail_.store(increment(tail), std::memory_order_release);
        return true;
    }

    size_t depth() const
    {
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t tail = tail_.load(std::memory_order_acquire);
        return head >= tail ? head - tail : Capacity - (tail - head);
    }

    constexpr size_t usableCapacity() const { return Capacity - 1U; }
    uint32_t highWater() const { return high_water_.load(std::memory_order_relaxed); }
    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

private:
    static constexpr size_t increment(size_t value)
    {
        return (value + 1U) % Capacity;
    }

    std::array<T, Capacity> entries_{};
    alignas(64) std::atomic<size_t> head_{0U};
    alignas(64) std::atomic<size_t> tail_{0U};
    std::atomic<uint32_t> high_water_{0U};
    std::atomic<uint32_t> dropped_{0U};
};

} // namespace nura::core
