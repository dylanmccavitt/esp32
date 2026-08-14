#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace fluid_demo {

template <typename Frame, std::size_t SlotCount>
class LatestFrameExchange {
    static_assert(SlotCount >= 3);

public:
    LatestFrameExchange()
    {
        for (std::size_t slot_index = 0; slot_index < SlotCount; ++slot_index) {
            states_[slot_index].store(SlotState::Free,
                                      std::memory_order_relaxed);
            sequences_[slot_index].store(0, std::memory_order_relaxed);
        }
    }

    LatestFrameExchange(const LatestFrameExchange &) = delete;
    LatestFrameExchange &operator=(const LatestFrameExchange &) = delete;

    Frame *begin_write()
    {
        if (writing_ != nullptr) {
            return nullptr;
        }
        for (std::size_t slot_index = 0; slot_index < SlotCount; ++slot_index) {
            SlotState expected = SlotState::Free;
            if (states_[slot_index].compare_exchange_strong(
                    expected, SlotState::Writing, std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                writing_ = &frames_[slot_index];
                return writing_;
            }
        }

        std::size_t oldest_slot = SlotCount;
        uint32_t oldest_sequence = 0;
        for (std::size_t slot_index = 0; slot_index < SlotCount; ++slot_index) {
            if (states_[slot_index].load(std::memory_order_acquire) !=
                SlotState::Ready) {
                continue;
            }
            const uint32_t sequence =
                sequences_[slot_index].load(std::memory_order_relaxed);
            if (oldest_slot == SlotCount ||
                sequence_is_newer(oldest_sequence, sequence)) {
                oldest_slot = slot_index;
                oldest_sequence = sequence;
            }
        }
        if (oldest_slot == SlotCount) {
            return nullptr;
        }

        SlotState expected = SlotState::Ready;
        if (!states_[oldest_slot].compare_exchange_strong(
                expected, SlotState::Writing, std::memory_order_acq_rel,
                std::memory_order_relaxed)) {
            return nullptr;
        }
        writing_ = &frames_[oldest_slot];
        return writing_;
    }

    void publish(Frame *frame)
    {
        const std::size_t slot_index = index_of(frame);
        if (frame != writing_ || slot_index == SlotCount) {
            return;
        }
        ++last_published_sequence_;
        if (last_published_sequence_ == 0u) {
            last_published_sequence_ = 1u;
        }
        sequences_[slot_index].store(last_published_sequence_,
                                     std::memory_order_relaxed);
        states_[slot_index].store(SlotState::Ready, std::memory_order_release);
        writing_ = nullptr;
    }

    const Frame *acquire_latest()
    {
        for (int attempt = 0; attempt < 2; ++attempt) {
            std::size_t newest_slot = SlotCount;
            uint32_t newest_sequence = 0;
            for (std::size_t slot_index = 0; slot_index < SlotCount;
                 ++slot_index) {
                if (states_[slot_index].load(std::memory_order_acquire) !=
                    SlotState::Ready) {
                    continue;
                }
                const uint32_t sequence =
                    sequences_[slot_index].load(std::memory_order_relaxed);
                if (newest_slot == SlotCount ||
                    sequence_is_newer(sequence, newest_sequence)) {
                    newest_slot = slot_index;
                    newest_sequence = sequence;
                }
            }
            if (newest_slot == SlotCount) {
                return nullptr;
            }

            SlotState expected = SlotState::Ready;
            if (states_[newest_slot].compare_exchange_strong(
                    expected, SlotState::Reading, std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                return &frames_[newest_slot];
            }
        }
        return nullptr;
    }

    void release(const Frame *frame)
    {
        const std::size_t slot_index = index_of(frame);
        if (slot_index == SlotCount) {
            return;
        }
        SlotState expected = SlotState::Reading;
        static_cast<void>(states_[slot_index].compare_exchange_strong(
            expected, SlotState::Free, std::memory_order_release,
            std::memory_order_relaxed));
    }

    void drain()
    {
        for (std::size_t remaining_slots = SlotCount; remaining_slots > 0;
             --remaining_slots) {
            const Frame *frame = acquire_latest();
            if (frame == nullptr) {
                break;
            }
            release(frame);
        }
    }

private:
    enum class SlotState : uint8_t {
        Free,
        Writing,
        Ready,
        Reading,
    };

    static bool sequence_is_newer(uint32_t candidate, uint32_t reference)
    {
        return static_cast<int32_t>(candidate - reference) > 0;
    }

    std::size_t index_of(const Frame *frame) const
    {
        for (std::size_t slot_index = 0; slot_index < SlotCount; ++slot_index) {
            if (frame == &frames_[slot_index]) {
                return slot_index;
            }
        }
        return SlotCount;
    }

    std::array<Frame, SlotCount> frames_{};
    std::array<std::atomic<SlotState>, SlotCount> states_{};
    std::array<std::atomic<uint32_t>, SlotCount> sequences_{};
    Frame *writing_ = nullptr;
    uint32_t last_published_sequence_ = 0;
};

}
