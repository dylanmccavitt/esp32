#include "fluid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace fluid_demo {
namespace {

// Physical-aspect simulation volume. The LCD active area is 27.72 mm square;
// the enclosed device is 22.5 mm deep. The renderer puts z=0 at the display.
constexpr float kBoxXMin = -0.5f;
constexpr float kBoxXMax = 0.5f;
constexpr float kBoxYMin = -0.5f;
constexpr float kBoxYMax = 0.5f;
constexpr float kBoxZMin = 0.0f;
constexpr float kBoxZMax = 0.812f;

constexpr float kPi = 3.14159265358979323846f;
constexpr float kSqrt3Inv = 0.5773502691896258f;  // 1 / sqrt(3), cube-diagonal unit vector

constexpr float kWallRestitution = 0.05f;   // normal reflection coefficient
constexpr float kWallTangential = 0.98f;    // tangential friction coefficient
constexpr float kVelocityDamping = 0.995f;  // light bulk damping at the fixed 30 Hz step
constexpr int kJacobiIterations = 2;

inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline bool finite3(const Vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// 3D Poly6 kernel: 315 / (64 * pi * h^9) * (h^2 - r^2)^3, zero beyond h.
inline float poly6(float r2, float h, float poly6_k) {
    const float h2 = h * h;
    if (r2 >= h2) return 0.0f;
    const float q = h2 - r2;
    return poly6_k * (q * q * q);
}

// Spiky gradient magnitude |dW/dr| = 45 / (pi * h^6) * (h - r)^2, zero beyond h.
// The full gradient is -mag * r_hat for r_ij = p_i - p_j (see callers).
inline float spiky_grad_mag(float r, float h, float spiky_k) {
    if (r >= h) return 0.0f;
    const float s = h - r;
    return spiky_k * (s * s);
}

inline float pow4(float q) { return q * q * q * q; }

// Deterministic, antisymmetric unit fallback direction for exact r == 0 pairs:
// zero_dir(a, b) == -zero_dir(b, a). Coincident particles push apart along a
// cube-diagonal direction chosen from the (unordered) index pair, so the choice
// is stable across passes and independent of neighbor iteration order.
void zero_dir(uint32_t a, uint32_t b, float& ox, float& oy, float& oz) {
    if (a > b) {
        zero_dir(b, a, ox, oy, oz);
        ox = -ox;
        oy = -oy;
        oz = -oz;
        return;
    }
    const uint32_t h = (a * 0x9E3779B9u) ^ (b * 0x85EBCA6Bu) ^ ((a ^ b) * 0xC2B2AE35u);
    ox = (h & 1u) ? kSqrt3Inv : -kSqrt3Inv;
    oy = (h & 2u) ? kSqrt3Inv : -kSqrt3Inv;
    oz = (h & 4u) ? kSqrt3Inv : -kSqrt3Inv;
}

// Half stencil for unordered pair traversal: the 13 lexicographically forward
// (dz, dy, dx) cell offsets. A pair in the same cell is taken with j > i; a
// pair in distinct adjacent cells is taken from the cell whose offset to the
// other is in this list, so every unordered pair is visited exactly once
// without scanning the 13 backward cells at all.
constexpr int8_t kForwardCells[13][3] = {
    // {dx, dy, dz}
    {1, 0, 0},
    {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
    {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
    {-1, 0, 1}, {0, 0, 1}, {1, 0, 1},
    {-1, 1, 1}, {0, 1, 1}, {1, 1, 1},
};

// Largest integer lattice dimension n such that (n - 1) * s fits inside `span`
// (the whole lattice stays within the wall bounds on that axis).
int lattice_max_dim(float s, float span) {
    int n = static_cast<int>(std::floor(span / s)) + 1;
    while (n > 1 && (static_cast<float>(n - 1) * s > span + 1.0e-4f)) {
        --n;
    }
    return n;
}

}  // namespace

bool Fluid::init(uint16_t particle_count) {
    if (particle_count < kMinParticles || particle_count > kMaxParticles) {
        return false;
    }
    count_ = particle_count;

    // Count-scaled geometry keeps the occupied fluid volume stable while the
    // active count changes. The 216-particle baseline forms a 6x6x6 volume.
    const float ratio = static_cast<float>(kInitialParticles) / static_cast<float>(count_);
    spacing_ = 0.14f * std::cbrt(ratio);
    h_ = spacing_ * (0.20f / 0.11f);
    radius_ = 0.30f * spacing_;

    // Kernel and solve constants.
    const float h2 = h_ * h_;
    const float h6 = h2 * h2 * h2;
    const float h9 = h6 * h2 * h_;
    poly6_k_ = 315.0f / (64.0f * kPi * h9);
    spiky_grad_k_ = 45.0f / (kPi * h6);
    spiky_center_mag_ = spiky_grad_k_ * h2;  // = 45 / (pi * h^4)
    epsilon_ = 0.2f / h2;
    lambda_cap_ = 0.5f * h2;
    scorr_k_ = 0.03f * h2;
    w_scorr_ref_ = poly6(0.09f * h2, h_, poly6_k_);  // W(0.3 * h)
    delta_cap_ = 0.25f * spacing_;

    // Rest density: self term plus the simple-cubic neighbor shells, unit mass,
    // exact 3D Poly6. Shells are enumerated directly (offsets (i,j,k) * spacing
    // with |offset| < h), which is equivalent to summing rings by radius.
    float rho0 = poly6(0.0f, h_, poly6_k_);  // j == i self contribution
    const int k = static_cast<int>(std::ceil(h_ / spacing_));
    for (int i = -k; i <= k; ++i) {
        for (int j = -k; j <= k; ++j) {
            for (int l = -k; l <= k; ++l) {
                if (i == 0 && j == 0 && l == 0) continue;
                const float r2 = static_cast<float>(i * i + j * j + l * l) * spacing_ * spacing_;
                if (r2 >= h_ * h_) continue;
                rho0 += poly6(r2, h_, poly6_k_);
            }
        }
    }
    rho0_ = rho0;
    inv_rho0_ = 1.0f / rho0_;

    // Neighbor grid: cells of size h, one extra cell per axis absorbs float
    // rounding at the upper wall; particle-to-cell lookups clamp anyway.
    gx_ = static_cast<int32_t>(std::ceil((kBoxXMax - kBoxXMin) / h_)) + 1;
    gy_ = static_cast<int32_t>(std::ceil((kBoxYMax - kBoxYMin) / h_)) + 1;
    gz_ = static_cast<int32_t>(std::ceil((kBoxZMax - kBoxZMin) / h_)) + 1;

    reset();
    return true;
}

void Fluid::reset() {
    if (count_ == 0) {
        return;
    }

    // Deterministic centered volumetric lattice: bounded integer dimension
    // search over all triples that fit the box (every dimension > 1, span
    // (n-1)*spacing inside the walls). Prefer the smallest capacity overshoot,
    // then the most cubic triple, then the fixed scan order.
    const int nx_max = lattice_max_dim(spacing_, kBoxXMax - kBoxXMin);
    const int ny_max = lattice_max_dim(spacing_, kBoxYMax - kBoxYMin);
    const int nz_max = lattice_max_dim(spacing_, kBoxZMax - kBoxZMin);

    int dim_x = 0, dim_y = 0, dim_z = 0;
    int best_fill = std::numeric_limits<int>::max();
    int best_bal = std::numeric_limits<int>::max();
    for (int cz = nz_max; cz >= 2; --cz) {
        for (int cy = ny_max; cy >= 2; --cy) {
            for (int cx = nx_max; cx >= 2; --cx) {
                const int64_t product = static_cast<int64_t>(cx) * cy * cz;
                if (product < count_) continue;
                const int fill = static_cast<int>(product - count_);
                const int bal = std::max(cx, std::max(cy, cz)) - std::min(cx, std::min(cy, cz));
                if (fill < best_fill || (fill == best_fill && bal < best_bal)) {
                    best_fill = fill;
                    best_bal = bal;
                    dim_x = cx;
                    dim_y = cy;
                    dim_z = cz;
                }
            }
        }
    }
    if (dim_x == 0) {  // cannot occur for supported counts; keep state untouched
        return;
    }

    // Build the occupied set in inversion-symmetric pairs, nearest the box
    // center first. This keeps partially filled lattices centered instead of
    // dropping every unused site from one end of the final plane.
    const int capacity = dim_x * dim_y * dim_z;
    if (capacity < count_ || capacity > static_cast<int>(kMaxGridCells)) {
        return;
    }
    for (int i = 0; i < capacity; ++i) {
        cell_count_[i] = 0;  // pair-used marker; grid rebuild overwrites it
    }

    auto point_score = [&](int linear) {
        const int cx = linear % dim_x;
        const int cy = (linear / dim_x) % dim_y;
        const int cz = linear / (dim_x * dim_y);
        const float ox = (static_cast<float>(cx) - 0.5f * static_cast<float>(dim_x - 1)) * spacing_;
        const float oy = (static_cast<float>(cy) - 0.5f * static_cast<float>(dim_y - 1)) * spacing_;
        const float oz = (static_cast<float>(cz) - 0.5f * static_cast<float>(dim_z - 1)) * spacing_;
        return ox * ox + oy * oy + oz * oz;
    };

    const float x_start = kBoxXMin + 0.5f * ((kBoxXMax - kBoxXMin) - static_cast<float>(dim_x - 1) * spacing_);
    const float y_start = kBoxYMin + 0.5f * ((kBoxYMax - kBoxYMin) - static_cast<float>(dim_y - 1) * spacing_);
    const float z_start = kBoxZMin + 0.5f * ((kBoxZMax - kBoxZMin) - static_cast<float>(dim_z - 1) * spacing_);
    size_t idx = 0;
    auto place = [&](int linear) {
        const int cx = linear % dim_x;
        const int cy = (linear / dim_x) % dim_y;
        const int cz = linear / (dim_x * dim_y);
        x_[idx] = x_start + static_cast<float>(cx) * spacing_;
        y_[idx] = y_start + static_cast<float>(cy) * spacing_;
        z_[idx] = z_start + static_cast<float>(cz) * spacing_;
        vx_[idx] = vy_[idx] = vz_[idx] = 0.0f;
        px_[idx] = py_[idx] = pz_[idx] = 0.0f;
        ++idx;
    };

    if ((count_ & 1u) != 0u) {
        int best = 0;
        float best_score = point_score(0);
        for (int linear = 1; linear < capacity; ++linear) {
            const float score = point_score(linear);
            if (score < best_score) {
                best = linear;
                best_score = score;
            }
        }
        place(best);
        cell_count_[std::min(best, capacity - 1 - best)] = 1;
    }

    while (idx < count_) {
        int best_rep = -1;
        float best_score = std::numeric_limits<float>::max();
        for (int rep = 0; rep < capacity / 2; ++rep) {
            if (cell_count_[rep] != 0) continue;
            const float score = point_score(rep);
            if (score < best_score) {
                best_rep = rep;
                best_score = score;
            }
        }
        if (best_rep < 0) {
            return;  // impossible when capacity >= count_; protects the arrays
        }
        cell_count_[best_rep] = 1;
        place(best_rep);
        place(capacity - 1 - best_rep);
    }

    ++reset_epoch_;
    // Per-run statistics are cleared, but the non-finite reset count is
    // cumulative so telemetry can track stability across resets.
    const uint32_t nf_resets = stats_.nonfinite_resets;
    stats_ = FluidStats{};
    stats_.nonfinite_resets = nf_resets;
}

bool Fluid::step(const Vec3& apparent_accel, float dt) {
    if (count_ == 0) {
        return false;
    }
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return false;
    }

    // Transient accelerometer noise must not corrupt the simulation: drop
    // gravity for this step, keep the state untouched.
    Vec3 g = apparent_accel;
    if (!finite3(g)) {
        g = Vec3{0.0f, 0.0f, 0.0f};
    }

    const float inv_dt = 1.0f / dt;
    const float vel_cap = 0.5f * h_ / dt;

    for (size_t i = 0; i < count_; ++i) {
        wnx_[i] = 0.0f;
        wny_[i] = 0.0f;
        wnz_[i] = 0.0f;
    }

    // Snapshot positions, integrate gravity into velocity, predict.
    for (size_t i = 0; i < count_; ++i) {
        px_[i] = x_[i];
        py_[i] = y_[i];
        pz_[i] = z_[i];

        vx_[i] += g.x * dt;
        vy_[i] += g.y * dt;
        vz_[i] += g.z * dt;
        x_[i] += vx_[i] * dt;
        y_[i] += vy_[i] * dt;
        z_[i] += vz_[i] * dt;
    }
    clamp_walls();

    // Jacobi constraint solve: per iteration, grid -> densities/lambdas ->
    // deltas -> positions, then clamp all six walls.
    for (int it = 0; it < kJacobiIterations; ++it) {
        rebuild_grid();
        compute_densities_and_lambdas();
        compute_and_apply_deltas();
    }

    // Recover velocity from the net displacement; reflect against contacted
    // walls (restitution 0.05 normal, 0.98 tangential).
    for (size_t i = 0; i < count_; ++i) {
        vx_[i] = (x_[i] - px_[i]) * inv_dt;
        vy_[i] = (y_[i] - py_[i]) * inv_dt;
        vz_[i] = (z_[i] - pz_[i]) * inv_dt;
    }
    apply_wall_reflection();
    for (size_t i = 0; i < count_; ++i) {
        vx_[i] *= kVelocityDamping;
        vy_[i] *= kVelocityDamping;
        vz_[i] *= kVelocityDamping;
    }

    // rho_ from the final Jacobi iteration is retained for a cheap density
    // diagnostic. Avoiding an extra full neighbor pass keeps the 30 Hz budget.
    velocity_cap(vel_cap);

    float max_err = 0.0f;
    for (size_t i = 0; i < count_; ++i) {
        const float err = std::fabs(rho_[i] * inv_rho0_ - 1.0f);
        if (err > max_err) max_err = err;
    }
    stats_.last_density_error = max_err;
    if (max_err > stats_.max_density_error) {
        stats_.max_density_error = max_err;
    }

    // Non-finite state requests a deterministic reset.
    for (size_t i = 0; i < count_; ++i) {
        const bool ok = std::isfinite(x_[i]) && std::isfinite(y_[i]) && std::isfinite(z_[i]) &&
                        std::isfinite(vx_[i]) && std::isfinite(vy_[i]) && std::isfinite(vz_[i]);
        if (!ok) {
            ++stats_.nonfinite_resets;
            reset();
            return false;
        }
    }
    return true;
}

void Fluid::fill_frame(ParticleFrame& out, uint32_t sequence) const {
    out.sequence = sequence;
    out.reset_epoch = reset_epoch_;
    out.count = count_;
    out.particle_radius = radius_;
    for (size_t i = 0; i < count_; ++i) {
        RenderParticle& p = out.particles[i];
        p.x = x_[i];
        p.y = y_[i];
        p.z = z_[i];
        p.speed = std::sqrt(vx_[i] * vx_[i] + vy_[i] * vy_[i] + vz_[i] * vz_[i]);
    }
    for (size_t i = count_; i < kMaxParticles; ++i) {
        out.particles[i] = RenderParticle{0.0f, 0.0f, 0.0f, 0.0f};
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

uint32_t Fluid::cell_index(float x, float y, float z) const {
    int32_t ix = static_cast<int32_t>((x - kBoxXMin) / h_);
    int32_t iy = static_cast<int32_t>((y - kBoxYMin) / h_);
    int32_t iz = static_cast<int32_t>((z - kBoxZMin) / h_);
    ix = ix < 0 ? 0 : (ix >= gx_ ? gx_ - 1 : ix);
    iy = iy < 0 ? 0 : (iy >= gy_ ? gy_ - 1 : iy);
    iz = iz < 0 ? 0 : (iz >= gz_ ? gz_ - 1 : iz);
    return static_cast<uint32_t>((iz * gy_ + iy) * gx_ + ix);
}

void Fluid::rebuild_grid() {
    const uint32_t cells = static_cast<uint32_t>(gx_) * static_cast<uint32_t>(gy_) * static_cast<uint32_t>(gz_);
    for (uint32_t c = 0; c < cells; ++c) {
        cell_count_[c] = 0;
    }
    for (size_t i = 0; i < count_; ++i) {
        ++cell_count_[cell_index(x_[i], y_[i], z_[i])];
    }
    uint32_t acc = 0;
    for (uint32_t c = 0; c < cells; ++c) {
        cell_start_[c] = acc;
        acc += cell_count_[c];
    }
    cell_start_[cells] = acc;
    // cell_count_ is now exhausted; reuse it as the fill cursor.
    for (uint32_t c = 0; c < cells; ++c) {
        cell_count_[c] = cell_start_[c];
    }
    for (size_t i = 0; i < count_; ++i) {
        cell_order_[cell_count_[cell_index(x_[i], y_[i], z_[i])]++] = static_cast<uint16_t>(i);
    }
}

void Fluid::clamp_walls() {
    for (size_t i = 0; i < count_; ++i) {
        // Inward normals accumulate so velocity reflection knows which walls
        // were contacted this step (corner contacts sum to a diagonal).
        if (x_[i] < kBoxXMin) {
            x_[i] = kBoxXMin;
            wnx_[i] += 1.0f;
        } else if (x_[i] > kBoxXMax) {
            x_[i] = kBoxXMax;
            wnx_[i] -= 1.0f;
        }
        if (y_[i] < kBoxYMin) {
            y_[i] = kBoxYMin;
            wny_[i] += 1.0f;
        } else if (y_[i] > kBoxYMax) {
            y_[i] = kBoxYMax;
            wny_[i] -= 1.0f;
        }
        if (z_[i] < kBoxZMin) {
            z_[i] = kBoxZMin;
            wnz_[i] += 1.0f;
        } else if (z_[i] > kBoxZMax) {
            z_[i] = kBoxZMax;
            wnz_[i] -= 1.0f;
        }
    }
}

void Fluid::compute_densities_and_lambdas() {
    const float h2 = h_ * h_;
    const float w0 = poly6(0.0f, h_, poly6_k_);

    for (size_t i = 0; i < count_; ++i) {
        rho_[i] = w0;  // self term
        gsx_[i] = 0.0f;
        gsy_[i] = 0.0f;
        gsz_[i] = 0.0f;
        gsq_[i] = 0.0f;
    }

    // One pass over unordered pairs via the half stencil: the own cell
    // contributes j > i, each forward cell contributes every resident, so
    // every pair is visited exactly once and both particles accumulate
    // symmetrically. Particle i's position and sums live in locals so the
    // inner loop never re-reads memory the j-side writes could alias.
    uint64_t cand_total = 0;
    for (size_t i = 0; i < count_; ++i) {
        const float xi = x_[i];
        const float yi = y_[i];
        const float zi = z_[i];
        float rho_i = 0.0f;
        float gsx_i = 0.0f, gsy_i = 0.0f, gsz_i = 0.0f;
        float gsq_i = 0.0f;
        uint32_t cand = 0;

        int32_t cx = static_cast<int32_t>((xi - kBoxXMin) / h_);
        int32_t cy = static_cast<int32_t>((yi - kBoxYMin) / h_);
        int32_t cz = static_cast<int32_t>((zi - kBoxZMin) / h_);
        cx = cx < 0 ? 0 : (cx >= gx_ ? gx_ - 1 : cx);
        cy = cy < 0 ? 0 : (cy >= gy_ ? gy_ - 1 : cy);
        cz = cz < 0 ? 0 : (cz >= gz_ ? gz_ - 1 : cz);

        const auto pair_with = [&](uint32_t j) {
            ++cand;
            const float dxp = xi - x_[j];
            const float dyp = yi - y_[j];
            const float dzp = zi - z_[j];
            const float r2 = dxp * dxp + dyp * dyp + dzp * dzp;
            if (r2 >= h2) return;

            float gx, gy, gz;
            if (r2 == 0.0f) {
                // Exact coincidence: deterministic antisymmetric fallback
                // direction with center-limit Spiky magnitude; both
                // particles see a full W(0) density.
                zero_dir(i, j, gx, gy, gz);
                const float m = spiky_center_mag_;
                gx = -m * gx;
                gy = -m * gy;
                gz = -m * gz;
                rho_i += w0;
                rho_[j] += w0;
            } else {
                const float r = std::sqrt(r2);
                const float inv_r = 1.0f / r;
                // r_hat points from j to i; dW/dr < 0 so the gradient of
                // W(r_ij) points from i toward j.
                const float m = spiky_grad_mag(r, h_, spiky_grad_k_);
                gx = -m * (dxp * inv_r);
                gy = -m * (dyp * inv_r);
                gz = -m * (dzp * inv_r);
                const float w = poly6(r2, h_, poly6_k_);
                rho_i += w;
                rho_[j] += w;
            }
            gsx_i += gx;
            gsy_i += gy;
            gsz_i += gz;
            gsx_[j] -= gx;
            gsy_[j] -= gy;
            gsz_[j] -= gz;
            const float m2 = gx * gx + gy * gy + gz * gz;
            gsq_i += m2;
            gsq_[j] += m2;
        };

        // Own cell: unordered pairs via j > i.
        const uint32_t self_base =
            (static_cast<uint32_t>(cz) * static_cast<uint32_t>(gy_) +
             static_cast<uint32_t>(cy)) * static_cast<uint32_t>(gx_) +
            static_cast<uint32_t>(cx);
        for (uint32_t k = cell_start_[self_base]; k < cell_start_[self_base + 1]; ++k) {
            const uint32_t j = cell_order_[k];
            if (j <= i) continue;
            pair_with(j);
        }
        // Forward cells: every resident pairs with i exactly once.
        for (const auto &f : kForwardCells) {
            const int32_t nx = cx + f[0];
            if (nx < 0 || nx >= gx_) continue;
            const int32_t ny = cy + f[1];
            if (ny < 0 || ny >= gy_) continue;
            const int32_t nz = cz + f[2];
            if (nz < 0 || nz >= gz_) continue;
            const uint32_t base = (static_cast<uint32_t>(nz) * static_cast<uint32_t>(gy_) +
                                   static_cast<uint32_t>(ny)) * static_cast<uint32_t>(gx_) +
                                  static_cast<uint32_t>(nx);
            for (uint32_t k = cell_start_[base]; k < cell_start_[base + 1]; ++k) {
                pair_with(cell_order_[k]);
            }
        }

        rho_[i] += rho_i;
        gsx_[i] += gsx_i;
        gsy_[i] += gsy_i;
        gsz_[i] += gsz_i;
        gsq_[i] += gsq_i;
        cand_total += cand;
    }
    stats_.candidate_checks += cand_total;

    // Constraint error C = rho/rho0 - 1 clamped to [-0.05, 1]; lambda
    // denominator is the squared self-gradient plus the neighbor gradient
    // squares (both scaled by 1/rho0^2), regularized by epsilon and capped.
    for (size_t i = 0; i < count_; ++i) {
        const float c = clampf(rho_[i] * inv_rho0_ - 1.0f, -0.05f, 1.0f);
        const float gsum2 = gsx_[i] * gsx_[i] + gsy_[i] * gsy_[i] + gsz_[i] * gsz_[i];
        const float denom = (gsum2 + gsq_[i]) * inv_rho0_ * inv_rho0_ + epsilon_;
        lambda_[i] = clampf(-c / denom, -lambda_cap_, lambda_cap_);
    }
}


void Fluid::compute_and_apply_deltas() {
    const float h2 = h_ * h_;
    const float w0 = poly6(0.0f, h_, poly6_k_);
    const float inv_rho0 = inv_rho0_;
    const float cap2 = delta_cap_ * delta_cap_;

    for (size_t i = 0; i < count_; ++i) {
        dx_[i] = 0.0f;
        dy_[i] = 0.0f;
        dz_[i] = 0.0f;
    }

    // Same half-stencil traversal as the density pass: own cell with j > i,
    // then the 13 forward cells in full. Particle i's accumulators are locals.
    uint64_t cand_total = 0;
    for (size_t i = 0; i < count_; ++i) {
        const float xi = x_[i];
        const float yi = y_[i];
        const float zi = z_[i];
        const float lambda_i = lambda_[i];
        float dxi = 0.0f, dyi = 0.0f, dzi = 0.0f;
        uint32_t cand = 0;

        int32_t cx = static_cast<int32_t>((xi - kBoxXMin) / h_);
        int32_t cy = static_cast<int32_t>((yi - kBoxYMin) / h_);
        int32_t cz = static_cast<int32_t>((zi - kBoxZMin) / h_);
        cx = cx < 0 ? 0 : (cx >= gx_ ? gx_ - 1 : cx);
        cy = cy < 0 ? 0 : (cy >= gy_ ? gy_ - 1 : cy);
        cz = cz < 0 ? 0 : (cz >= gz_ ? gz_ - 1 : cz);

        const auto pair_with = [&](uint32_t j) {
            ++cand;
            const float dxp = xi - x_[j];
            const float dyp = yi - y_[j];
            const float dzp = zi - z_[j];
            const float r2 = dxp * dxp + dyp * dyp + dzp * dzp;
            if (r2 >= h2) return;

            float gx, gy, gz;
            float s;  // scorr surface term (negative)
            if (r2 == 0.0f) {
                zero_dir(i, j, gx, gy, gz);
                const float m = spiky_center_mag_;
                gx = -m * gx;
                gy = -m * gy;
                gz = -m * gz;
                s = -scorr_k_ * pow4(w0 / w_scorr_ref_);
            } else {
                const float r = std::sqrt(r2);
                const float inv_r = 1.0f / r;
                const float m = spiky_grad_mag(r, h_, spiky_grad_k_);
                gx = -m * (dxp * inv_r);
                gy = -m * (dyp * inv_r);
                gz = -m * (dzp * inv_r);
                const float w = poly6(r2, h_, poly6_k_);
                s = -scorr_k_ * pow4(w / w_scorr_ref_);
            }
            const float coeff = (lambda_i + lambda_[j] + s) * inv_rho0;
            dxi += coeff * gx;
            dyi += coeff * gy;
            dzi += coeff * gz;
            dx_[j] -= coeff * gx;
            dy_[j] -= coeff * gy;
            dz_[j] -= coeff * gz;
        };

        const uint32_t self_base =
            (static_cast<uint32_t>(cz) * static_cast<uint32_t>(gy_) +
             static_cast<uint32_t>(cy)) * static_cast<uint32_t>(gx_) +
            static_cast<uint32_t>(cx);
        for (uint32_t k = cell_start_[self_base]; k < cell_start_[self_base + 1]; ++k) {
            const uint32_t j = cell_order_[k];
            if (j <= i) continue;
            pair_with(j);
        }
        for (const auto &f : kForwardCells) {
            const int32_t nx = cx + f[0];
            if (nx < 0 || nx >= gx_) continue;
            const int32_t ny = cy + f[1];
            if (ny < 0 || ny >= gy_) continue;
            const int32_t nz = cz + f[2];
            if (nz < 0 || nz >= gz_) continue;
            const uint32_t base = (static_cast<uint32_t>(nz) * static_cast<uint32_t>(gy_) +
                                   static_cast<uint32_t>(ny)) * static_cast<uint32_t>(gx_) +
                                  static_cast<uint32_t>(nx);
            for (uint32_t k = cell_start_[base]; k < cell_start_[base + 1]; ++k) {
                pair_with(cell_order_[k]);
            }
        }

        dx_[i] += dxi;
        dy_[i] += dyi;
        dz_[i] += dzi;
        cand_total += cand;
    }
    stats_.candidate_checks += cand_total;

    // Cap each delta, apply to positions, clamp all six walls.
    for (size_t i = 0; i < count_; ++i) {
        const float d2 = dx_[i] * dx_[i] + dy_[i] * dy_[i] + dz_[i] * dz_[i];
        if (d2 > cap2) {
            const float scale = delta_cap_ / std::sqrt(d2);
            dx_[i] *= scale;
            dy_[i] *= scale;
            dz_[i] *= scale;
        }
        x_[i] += dx_[i];
        y_[i] += dy_[i];
        z_[i] += dz_[i];
    }
    clamp_walls();
}

void Fluid::apply_wall_reflection() {
    for (size_t i = 0; i < count_; ++i) {
        const float nx = wnx_[i];
        const float ny = wny_[i];
        const float nz = wnz_[i];
        const float n2 = nx * nx + ny * ny + nz * nz;
        if (n2 <= 0.0f) continue;
        const float inv = 1.0f / std::sqrt(n2);
        const float unx = nx * inv;
        const float uny = ny * inv;
        const float unz = nz * inv;
        const float vn = vx_[i] * unx + vy_[i] * uny + vz_[i] * unz;
        if (vn < 0.0f) {  // still pressing into the wall
            const float vtx = vx_[i] - vn * unx;
            const float vty = vy_[i] - vn * uny;
            const float vtz = vz_[i] - vn * unz;
            vx_[i] = -kWallRestitution * vn * unx + kWallTangential * vtx;
            vy_[i] = -kWallRestitution * vn * uny + kWallTangential * vty;
            vz_[i] = -kWallRestitution * vn * unz + kWallTangential * vtz;
        }
    }
}


void Fluid::velocity_cap(float cap) {
    const float cap2 = cap * cap;
    for (size_t i = 0; i < count_; ++i) {
        const float v2 = vx_[i] * vx_[i] + vy_[i] * vy_[i] + vz_[i] * vz_[i];
        if (v2 > cap2) {
            const float scale = cap / std::sqrt(v2);
            vx_[i] *= scale;
            vy_[i] *= scale;
            vz_[i] *= scale;
        }
    }
}

}  // namespace fluid_demo
