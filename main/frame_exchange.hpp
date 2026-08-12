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
        for (std::size_t i = 0; i < SlotCount; ++i) {
            states_[i].store(SlotState::Free, std::memory_order_relaxed);
            sequences_[i].store(0, std::memory_order_relaxed);
        }
    }

    LatestFrameExchange(const LatestFrameExchange &) = delete;
    LatestFrameExchange &operator=(const LatestFrameExchange &) = delete;

    Frame *begin_write()
    {
        if (writing_ != nullptr) {
            return nullptr;
        }
        for (std::size_t i = 0; i < SlotCount; ++i) {
            SlotState expected = SlotState::Free;
            if (states_[i].compare_exchange_strong(
                    expected, SlotState::Writing, std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                writing_ = &frames_[i];
                return writing_;
            }
        }

        std::size_t oldest = SlotCount;
        uint32_t oldest_sequence = 0;
        for (std::size_t i = 0; i < SlotCount; ++i) {
            if (states_[i].load(std::memory_order_acquire) != SlotState::Ready) {
                continue;
            }
            const uint32_t sequence =
                sequences_[i].load(std::memory_order_relaxed);
            if (oldest == SlotCount || newer(oldest_sequence, sequence)) {
                oldest = i;
                oldest_sequence = sequence;
            }
        }
        if (oldest == SlotCount) {
            return nullptr;
        }

        SlotState expected = SlotState::Ready;
        if (!states_[oldest].compare_exchange_strong(
                expected, SlotState::Writing, std::memory_order_acq_rel,
                std::memory_order_relaxed)) {
            return nullptr;
        }
        writing_ = &frames_[oldest];
        return writing_;
    }

    void publish(Frame *frame)
    {
        const std::size_t index = index_of(frame);
        if (frame != writing_ || index == SlotCount) {
            return;
        }
        sequences_[index].store(frame->sequence, std::memory_order_relaxed);
        states_[index].store(SlotState::Ready, std::memory_order_release);
        writing_ = nullptr;
    }

    const Frame *acquire_latest()
    {
        for (int attempt = 0; attempt < 2; ++attempt) {
            std::size_t newest = SlotCount;
            uint32_t newest_sequence = 0;
            for (std::size_t i = 0; i < SlotCount; ++i) {
                if (states_[i].load(std::memory_order_acquire) != SlotState::Ready) {
                    continue;
                }
                const uint32_t sequence =
                    sequences_[i].load(std::memory_order_relaxed);
                if (newest == SlotCount || newer(sequence, newest_sequence)) {
                    newest = i;
                    newest_sequence = sequence;
                }
            }
            if (newest == SlotCount) {
                return nullptr;
            }

            SlotState expected = SlotState::Ready;
            if (states_[newest].compare_exchange_strong(
                    expected, SlotState::Reading, std::memory_order_acq_rel,
                    std::memory_order_relaxed)) {
                return &frames_[newest];
            }
        }
        return nullptr;
    }

    void release(const Frame *frame)
    {
        const std::size_t index = index_of(frame);
        if (index == SlotCount) {
            return;
        }
        SlotState expected = SlotState::Reading;
        static_cast<void>(states_[index].compare_exchange_strong(
            expected, SlotState::Free, std::memory_order_release,
            std::memory_order_relaxed));
    }

    void drain()
    {
        for (std::size_t i = 0; i < SlotCount; ++i) {
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

    static bool newer(uint32_t candidate, uint32_t reference)
    {
        return static_cast<int32_t>(candidate - reference) > 0;
    }

    std::size_t index_of(const Frame *frame) const
    {
        for (std::size_t i = 0; i < SlotCount; ++i) {
            if (frame == &frames_[i]) {
                return i;
            }
        }
        return SlotCount;
    }

    std::array<Frame, SlotCount> frames_{};
    std::array<std::atomic<SlotState>, SlotCount> states_{};
    std::array<std::atomic<uint32_t>, SlotCount> sequences_{};
    Frame *writing_ = nullptr;
};

}  // namespace fluid_demo
