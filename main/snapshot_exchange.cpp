#include "snapshot_exchange.hpp"

#include <cstddef>
#include <cstdint>

namespace fluid_demo {

SnapshotExchange::SnapshotExchange() : write_slot_(nullptr) {
    for (auto &state : states_) {
        state.store(SlotState::kFree);
    }
    for (auto &sequence : sequences_) {
        sequence.store(0);
    }
}

ParticleFrame *SnapshotExchange::begin_write() {
    // Protocol guard: a begin_write without an intervening publish would
    // leak the in-flight slot. Reject instead of corrupting the pool.
    if (write_slot_ != nullptr) {
        return nullptr;
    }

    // Pass 1: prefer an untouched FREE slot. Only the producer can claim a
    // FREE slot, so this never races with the consumer.
    for (size_t i = 0; i < kSlotCount; ++i) {
        SlotState expected = SlotState::kFree;
        if (states_[i].compare_exchange_strong(expected, SlotState::kWriting)) {
            write_slot_ = &slots_[i];
            return write_slot_;
        }
    }

    // Pass 2: no FREE slot; reclaim the stale READY slot with the lowest
    // sequence so the consumer keeps the freshest frame. The consumer may be
    // CAS-ing the same slot to READING concurrently, so retry the scan.
    constexpr int kMaxAttempts = 4;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        size_t best = kSlotCount;
        uint32_t best_seq = 0;
        bool have_best = false;
        for (size_t i = 0; i < kSlotCount; ++i) {
            if (states_[i].load() == SlotState::kReady) {
                const uint32_t seq = sequences_[i].load();
                if (!have_best || seq < best_seq) {
                    best = i;
                    best_seq = seq;
                    have_best = true;
                }
            }
        }
        if (!have_best) {
            return nullptr;  // everything is READING or WRITING: no claimable slot
        }
        SlotState expected = SlotState::kReady;
        if (states_[best].compare_exchange_strong(expected, SlotState::kWriting)) {
            write_slot_ = &slots_[best];
            return write_slot_;
        }
        // Consumer stole this slot mid-reclaim; retry with a fresh scan.
    }
    return nullptr;
}

void SnapshotExchange::publish(ParticleFrame *slot) {
    const int idx = index_of(slot);
    if (idx < 0) {
        return;  // foreign or interior pointer: never touch it
    }
    // The frame data was written before this release store, so a consumer
    // that observes READY also observes the complete frame.
    sequences_[idx].store(slot->sequence);
    SlotState expected = SlotState::kWriting;
    if (!states_[idx].compare_exchange_strong(expected, SlotState::kReady)) {
        return;  // not in WRITING state: not ours to publish
    }
    write_slot_ = nullptr;
}

const ParticleFrame *SnapshotExchange::acquire_latest() {
    constexpr int kMaxAttempts = 4;
    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        size_t best = kSlotCount;
        uint32_t best_seq = 0;
        bool have_best = false;
        for (size_t i = 0; i < kSlotCount; ++i) {
            if (states_[i].load() == SlotState::kReady) {
                const uint32_t seq = sequences_[i].load();
                if (!have_best || seq > best_seq) {
                    best = i;
                    best_seq = seq;
                    have_best = true;
                }
            }
        }
        if (!have_best) {
            return nullptr;  // no published frame yet
        }
        SlotState expected = SlotState::kReady;
        if (states_[best].compare_exchange_strong(expected, SlotState::kReading)) {
            return &slots_[best];
        }
        // Producer reclaimed this slot concurrently; retry.
    }
    return nullptr;
}

void SnapshotExchange::release(const ParticleFrame *slot) {
    const int idx = index_of(slot);
    if (idx < 0) {
        return;  // foreign or interior pointer
    }
    SlotState expected = SlotState::kReading;
    if (!states_[idx].compare_exchange_strong(expected, SlotState::kFree)) {
        return;  // not held by the consumer: leave untouched
    }
}

int SnapshotExchange::index_of(const ParticleFrame *slot) const {
    if (slot == nullptr) {
        return -1;
    }
    const uintptr_t addr = reinterpret_cast<uintptr_t>(slot);
    const uintptr_t base = reinterpret_cast<uintptr_t>(slots_.data());
    const uintptr_t end = base + kSlotCount * sizeof(ParticleFrame);
    if (addr < base || addr >= end) {
        return -1;
    }
    const size_t offset = static_cast<size_t>(addr - base) / sizeof(ParticleFrame);
    // Reject interior pointers: the address must land exactly on a slot start.
    if (offset >= kSlotCount || slot != &slots_[offset]) {
        return -1;
    }
    return static_cast<int>(offset);
}

}  // namespace fluid_demo
