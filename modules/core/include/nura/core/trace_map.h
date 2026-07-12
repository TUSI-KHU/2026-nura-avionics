#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "nura/contracts/trace_topics.h"
#include "nura/core/build_config.h"

namespace nura::core
{

template <size_t Capacity>
class TraceMap
{
    static_assert(Capacity > 0U, "TraceMap must have storage");

public:
    bool tryRecord(nura::contracts::TraceRecord record)
    {
        if (lock_.test_and_set(std::memory_order_acquire))
        {
            dropped_.fetch_add(1U, std::memory_order_relaxed);
            return false;
        }

        record.sequence = next_sequence_++;
        records_[write_index_] = record;
        write_index_ = (write_index_ + 1U) % Capacity;
        if (count_ < Capacity)
        {
            ++count_;
        }
        else
        {
            overwritten_.fetch_add(1U, std::memory_order_relaxed);
        }
        lock_.clear(std::memory_order_release);
        return true;
    }

    // The exporter is best-effort and never waits on a flight-critical writer.
    size_t trySnapshot(nura::contracts::TraceRecord *output, size_t output_capacity) const
    {
        if (output == nullptr || output_capacity == 0U ||
            lock_.test_and_set(std::memory_order_acquire))
        {
            return 0U;
        }

        const size_t to_copy = count_ < output_capacity ? count_ : output_capacity;
        const size_t oldest = count_ < Capacity ? 0U : write_index_;
        const size_t skip = count_ > to_copy ? count_ - to_copy : 0U;
        for (size_t i = 0U; i < to_copy; ++i)
        {
            output[i] = records_[(oldest + skip + i) % Capacity];
        }
        lock_.clear(std::memory_order_release);
        return to_copy;
    }

    // Copies the oldest records newer than last_sequence. A non-zero gap means
    // the exporter fell behind the bounded ring and records were overwritten.
    size_t tryReadAfter(uint32_t last_sequence,
                        nura::contracts::TraceRecord *output,
                        size_t output_capacity,
                        uint32_t &gap) const
    {
        gap = 0U;
        if (output == nullptr || output_capacity == 0U ||
            lock_.test_and_set(std::memory_order_acquire))
        {
            return 0U;
        }

        const size_t oldest_index = count_ < Capacity ? 0U : write_index_;
        size_t copied = 0U;
        bool first_new_found = false;
        for (size_t i = 0U; i < count_ && copied < output_capacity; ++i)
        {
            const auto &record = records_[(oldest_index + i) % Capacity];
            if (record.sequence <= last_sequence)
            {
                continue;
            }
            if (!first_new_found && last_sequence != 0U &&
                record.sequence > last_sequence + 1U)
            {
                gap = record.sequence - last_sequence - 1U;
            }
            first_new_found = true;
            output[copied++] = record;
        }
        lock_.clear(std::memory_order_release);
        return copied;
    }

    uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }
    uint32_t overwritten() const { return overwritten_.load(std::memory_order_relaxed); }
    constexpr size_t capacity() const { return Capacity; }

private:
    mutable std::atomic_flag lock_ = ATOMIC_FLAG_INIT;
    std::array<nura::contracts::TraceRecord, Capacity> records_{};
    size_t write_index_ = 0U;
    size_t count_ = 0U;
    uint32_t next_sequence_ = 1U;
    std::atomic<uint32_t> dropped_{0U};
    std::atomic<uint32_t> overwritten_{0U};
};

using SystemTraceMap = TraceMap<NURA_TRACE_CAPACITY_RECORDS>;

} // namespace nura::core
