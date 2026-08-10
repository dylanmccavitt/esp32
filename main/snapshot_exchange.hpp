#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "app_types.hpp"

namespace fluid_demo {

// Triple-slot, lock-free producer/consumer exchange for ParticleFrame.
//
// Ownership model (two roles, one instance each):
//   producer (physics task):  begin_write() -> fill slot -> publish(slot)
//   consumer (render task):   acquire_latest() -> render -> release(slot)
//
// Exactly one WRITING slot (the producer's in-flight frame), at most one
// READING slot (the consumer's held frame) and the remaining slot(s) are
// FREE or READY, so the producer can always claim a slot without ever
// touching one the consumer is holding. The producer may reclaim a stale
// READY slot when no FREE slot exists, but never a READING slot. The
// consumer selects the most recently published (highest sequence) frame.
// No allocation, no blocking, no copies; the only synchronisation is
// seq_cst state transitions on three byte atomics.
class SnapshotExchange {
public:
    SnapshotExchange();

    SnapshotExchange(const SnapshotExchange &) = delete;
    SnapshotExchange &operator=(const SnapshotExchange &) = delete;

    // --- Producer side (single caller) ---

    // Claims a slot for writing: prefers a FREE slot, otherwise reclaims the
    // stale READY slot with the lowest sequence. Never reclaims a READING
    // slot. Returns nullptr only if no slot can be claimed (cannot happen
    // with one producer and one consumer; still handled defensively).
    ParticleFrame *begin_write();

    // Validates that `slot` came from this exchange and is in WRITING state,
    // then publishes it: the frame becomes visible as the newest READY frame.
    // Invalid or non-WRITING slots are rejected and left untouched.
    void publish(ParticleFrame *slot);

    // --- Consumer side (single caller) ---

    // Returns the newest READY frame and marks it READING, or nullptr when no
    // READY frame exists. The returned pointer remains owned by the caller
    // until release() is called with it.
    const ParticleFrame *acquire_latest();

    // Returns a READING slot to the FREE pool. Validates the pointer; invalid
    // or non-READING slots are rejected and left untouched.
    void release(const ParticleFrame *slot);

private:
    enum class SlotState : uint8_t {
        kFree,
        kWriting,
        kReady,
        kReading,
    };

    static constexpr size_t kSlotCount = 3;

    // Slot index for a pointer that points exactly at one of the slots,
    // else -1 (also rejects interior pointers into a slot).
    int index_of(const ParticleFrame *slot) const;

    // Producer-owned bookkeeping (touched only by begin_write/publish).
    ParticleFrame *write_slot_;

    std::array<ParticleFrame, kSlotCount> slots_{};
    std::array<std::atomic<SlotState>, kSlotCount> states_{};
    // Sequence metadata is atomic because producer and consumer inspect READY
    // slots before either side wins the state CAS; reading frame payload there
    // would otherwise race a producer that has just reclaimed the slot.
    std::array<std::atomic<uint32_t>, kSlotCount> sequences_{};
};

}  // namespace fluid_demo
