#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
#include "frame_exchange.hpp"
#include "motion.hpp"

namespace fluid_demo {

/// Fixed-capacity particle simulation with immutable update-to-render frames.
class FluidBoxApp final : public App {
public:
    FluidBoxApp() = default;
    ~FluidBoxApp() override;
    FluidBoxApp(const FluidBoxApp &) = delete;
    FluidBoxApp &operator=(const FluidBoxApp &) = delete;

    esp_err_t setup_once() override;

    esp_err_t enter() override;

    bool on_motion(const MotionTick &tick) override;

    ShellAction handle_event(AppEvent) override;

    const LauncherVisual *launcher_visual() const override;

    esp_err_t update(float dt = kPhysicsDt) override;

    bool render(DisplayFrame &frame) override;

    AppStats stats() override;

    void leave() override;

    void request_fluid_reset();

private:
    static constexpr int kGridW = 240;
    static constexpr int kGridH = 240;
    static constexpr int kParticleCount = 3000;
    static_assert(kGridW * kGridH <= UINT16_MAX);

    struct FluidFrame {
        uint32_t sequence = 0;
        uint32_t epoch = 0;
        uint16_t count = 0;
        std::array<uint16_t, kParticleCount> cells{};
        std::array<uint8_t, kParticleCount> colors{};
    };
    static_assert(sizeof(FluidFrame) <= 9020);

    // Particle state byte: bits 0-2 base shade (0..5), bits 3-5 displayed
    // glow level, bit 6 ASLEEP. Grid byte: 0 empty, else
    // (level << 3) | (shade + 1) with bits 6-7 = saturating kick counter
    // harvested one frame later; the raster masks & 0x3F so kick bits are
    // invisible.
    static constexpr uint8_t kStateShadeMask = 0x07;
    static constexpr int kStateGlowShift = 3;
    static constexpr uint8_t kStateGlowMask = 0x38;
    static constexpr uint8_t kStateAsleep = 0x40;
    static constexpr uint8_t kStateOverlap = 0x80;

    struct GravityState {
        enum class Mode : uint8_t {
            Flat,
            Tilted,
        };

        struct Frame {
            float x;
            float y;
            float acceleration_units;
            bool wake_all;
        };

        Frame resolve(float sgx, float sgy, float sgz);

        Mode mode = Mode::Flat;
        float x = 0.0f;
        float y = 1.0f;
        float wake_anchor_x = 0.0f;
        float wake_anchor_y = 1.0f;
    };

    void reset_particles();
    uint32_t step_particles(float sgx, float sgy, float sgz);
    void fill_frame(FluidFrame &frame);
    void draw_frame(const FluidFrame &frame, uint16_t *buf, int y0, int rows);
    inline uint32_t rnd();

    MotionFilter filter_;

    struct SharedMotion {
        Vec3 apparent_accel{0.0f, 0.0f, 9.0f};
        Vec3 raw_accel{0.0f, 0.0f, 0.0f};
        bool valid{false};
    };
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_;

    std::atomic<bool> reset_requested_{false};

    // Telemetry atomics (stats() may be called from another lane).
    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint64_t> walk_steps_{0};
    std::atomic<uint32_t> governor_hits_{0};
    std::atomic<uint32_t> physics_us_{0};

    // Particle structure-of-arrays arena and occupancy grid.
    void *arena_ = nullptr;
    uint16_t *px_ = nullptr;
    uint16_t *py_ = nullptr;
    int16_t *vx_ = nullptr;
    int16_t *vy_ = nullptr;
    uint8_t *pstate_ = nullptr;
    uint8_t *prest_ = nullptr;
    uint8_t *grid_ = nullptr;

    uint32_t rng_ = 0x2545F491u;
    uint32_t frame_parity_ = 0;   ///< alternates sweep direction per frame
    uint32_t awake_count_ = 0;    ///< particles not ASLEEP after last step
    uint32_t rest_gate_frames_ = 0;
    GravityState gravity_;
    uint32_t sequence_ = 0;
    LatestFrameExchange<FluidFrame, 3> frames_;

    /// Wire-order (pre-swapped) RGB565 palette indexed by the grid byte
    /// (level << 3 | shade + 1) & 0x3F; shade-0 slots unused.
    uint16_t shade_wire_[64] = {};

    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;

    bool setup_done_ = false;
};

extern FluidBoxApp s_fluid_app;

}  // namespace fluid_demo
