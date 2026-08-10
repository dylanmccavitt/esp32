#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace fluid_demo {

// Fixed simulation capacity shared by every module (fluid, snapshot exchange,
// renderer). All buffers are sized from these constants; nothing allocates at
// runtime.
constexpr size_t kMaxParticles = 768;
// Tuned startup count for 30 Hz physics on the 240x240 board. Buffers retain
// headroom for profiling, but steady-state allocation never changes.
constexpr size_t kInitialParticles = 216;
constexpr size_t kMinParticles = 128;

// A point in simulation box coordinates. The box preserves the physical ratio:
// x[-0.5,0.5], y[-0.5,0.5], z[0,0.812]. MotionFilter supplies apparent gravity.
struct Vec3 {
    float x;
    float y;
    float z;
};

// One particle as shipped to the renderer: position plus a speed-derived color
// scalar (velocity palette). Layout is owned by the fluid lane.
struct RenderParticle {
    float x;
    float y;
    float z;
    float speed;
};

// Snapshot of one physics frame, exchanged between the physics task and the
// render task through the snapshot exchange. No pointers, no allocation: the
// particle array is inline and always capacity-sized.
struct ParticleFrame {
    uint32_t sequence = 0;       // monotonic producer sequence number
    uint32_t reset_epoch = 0;    // fluid reset epoch, echoed for debug telemetry
    uint16_t count = 0;          // active particle count (<= kMaxParticles)
    float particle_radius = 0.0f;
    std::array<RenderParticle, kMaxParticles> particles{};
};

}  // namespace fluid_demo
