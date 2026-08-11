#pragma once

#include "app_types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace fluid_demo {

// Aggregate statistics surfaced to the app for once-per-second telemetry.
struct FluidStats {
    uint64_t candidate_checks = 0;    // neighbor pairs examined since the last reset
    uint32_t nonfinite_resets = 0;    // deterministic resets forced by non-finite state (cumulative, survives reset())
    float last_density_error = 0.0f;  // max |rho/rho0 - 1| over the latest step
    float max_density_error = 0.0f;   // running max since the last reset
};

// True 3D Position Based Fluids (Macklin & Mueller, "Position Based Fluids",
// 2013) with fixed capacity and zero runtime allocation. All particle state is
// stored in SoA arrays sized kMaxParticles; the object is intended to live at
// global scope (never on a task stack) and to be moved by none.
//
// Geometry contract: box x[-0.5,0.5], y[-0.5,0.5], z[0,0.812]. Spacing,
// kernel radius h and rendered radius scale with particle count so supported
// counts pack a comparable fluid volume. The rest density rho0 is
// calibrated against that lattice (self term plus simple-cubic neighbor shells,
// unit mass, exact 3D Poly6), so a perfect rest lattice has exactly C = 0.
//
// PBF solve: 2 Jacobi iterations at the caller's fixed dt; a 3D counting grid
// (cell size h) is rebuilt from the current predicted positions at the start of
// each iteration. Unordered pairs are gathered by a 14-cell half-stencil
// traversal (own cell with j > i plus the 13 lexicographically forward cells,
// no capped lists), so backward cells are never scanned. Poly6 density, exact
// Spiky gradient, scorr surface term, symmetric lambda denominator (squared
// self-gradient + neighbor gradient squares).
class Fluid {
public:
    Fluid() = default;
    Fluid(const Fluid&) = delete;
    Fluid& operator=(const Fluid&) = delete;

    // Configure the particle count (supported range [kMinParticles,
    // kMaxParticles]) and lay down a fresh deterministic lattice. Returns false
    // for an unsupported count.
    bool init(uint16_t particle_count);

    // Rebuild the centered volumetric lattice with the current count and
    // parameters, zero all velocities, clear statistics. Deterministic. Bumps
    // the reset epoch, which is echoed into every frame via fill_frame.
    void reset();

    // Advance one physics step at fixed dt. Gravity follows apparent_accel (the
    // motion filter normalizes it to the simulation gravity magnitude). Returns
    // false if the step was skipped (invalid dt) or if non-finite state forced
    // a deterministic reset (stat.nonfinite_resets incremented). Never writes
    // outside the fixed buffers.
    bool step(const Vec3& apparent_accel, float dt);

    // Copy the current state into a renderer frame. `sequence` is supplied by
    // the caller so the producer owns the numbering; reset epoch is filled from
    // the internal counter.
    void fill_frame(ParticleFrame& out, uint32_t sequence) const;

    uint16_t count() const { return count_; }
    uint32_t reset_epoch() const { return reset_epoch_; }
    float particle_radius() const { return radius_; }
    const FluidStats& stats() const { return stats_; }

private:
    // ---- fixed-capacity SoA state ----
    std::array<float, kMaxParticles> x_{}, y_{}, z_{};        // current positions
    std::array<float, kMaxParticles> px_{}, py_{}, pz_{};     // positions at step start
    std::array<float, kMaxParticles> vx_{}, vy_{}, vz_{};     // velocities

    // ---- per-step scratch (reused, never allocated) ----
    std::array<float, kMaxParticles> rho_{};                  // densities
    std::array<float, kMaxParticles> lambda_{};
    std::array<float, kMaxParticles> gsx_{}, gsy_{}, gsz_{};  // summed neighbor gradients
    std::array<float, kMaxParticles> gsq_{};                  // summed |grad|^2
    std::array<float, kMaxParticles> dx_{}, dy_{}, dz_{};     // accumulated deltas
    std::array<float, kMaxParticles> wnx_{}, wny_{}, wnz_{};  // accumulated wall normals

    // ---- neighbor counting grid ----
    // Bounded above by the largest grid any supported count can produce
    // (count=768 -> h~0.159 -> <= 8x8x7 cells); 4096 is a safe ceiling.
    static constexpr size_t kMaxGridCells = 4096;
    std::array<uint32_t, kMaxGridCells> cell_count_{};
    std::array<uint32_t, kMaxGridCells + 1> cell_start_{};
    std::array<uint16_t, kMaxParticles> cell_order_{};
    int32_t gx_ = 0, gy_ = 0, gz_ = 0;                        // cells per axis

    // ---- configuration ----
    uint16_t count_ = 0;
    uint32_t reset_epoch_ = 0;
    float spacing_ = 0.0f;
    float h_ = 0.0f;              // kernel support radius
    float radius_ = 0.0f;         // rendered particle radius (0.30 * spacing)
    float rho0_ = 0.0f;
    float inv_rho0_ = 0.0f;
    float poly6_k_ = 0.0f;        // 315 / (64 * pi * h^9)
    float spiky_grad_k_ = 0.0f;   // 45 / (pi * h^6), |grad W| = k * (h - r)^2
    float spiky_center_mag_ = 0.0f;  // 45 / (pi * h^4), center-limit gradient
    float epsilon_ = 0.0f;        // 0.2 / h^2, lambda regularization
    float lambda_cap_ = 0.0f;     // 0.5 * h^2
    float scorr_k_ = 0.0f;        // 0.03 * h^2
    float w_scorr_ref_ = 0.0f;    // Poly6(0.3 * h)
    float delta_cap_ = 0.0f;      // 0.25 * spacing

    FluidStats stats_{};

    // ---- internal helpers ----
    uint32_t cell_index(float x, float y, float z) const;
    void rebuild_grid();
    void clamp_walls();
    void compute_densities_and_lambdas();
    void compute_and_apply_deltas();
    void apply_wall_reflection();
    void velocity_cap(float cap);
};

}  // namespace fluid_demo
