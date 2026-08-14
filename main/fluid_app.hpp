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

class FluidBoxApp final : public App {
public:
    FluidBoxApp() = default;
    ~FluidBoxApp() override;
    FluidBoxApp(const FluidBoxApp &) = delete;
    FluidBoxApp &operator=(const FluidBoxApp &) = delete;

    esp_err_t setup_once() override;
    esp_err_t enter() override;
    bool on_motion(const MotionTick &tick) override;
    void on_plus_press() override;
    const LauncherVisual *launcher_visual() const override;
    esp_err_t update(float dt = kPhysicsDt) override;
    bool render(DisplayFrame &frame) override;
    AppStats stats() override;
    void leave() override;

private:
    static constexpr int kGridWidth = 240;
    static constexpr int kGridHeight = 240;
    static constexpr int kParticleCount = 3000;
    static_assert(kGridWidth * kGridHeight <= UINT16_MAX);

    struct FluidFrame {
        uint16_t count = 0;
        std::array<uint16_t, kParticleCount> cells{};
        std::array<uint8_t, kParticleCount> colors{};
    };
    static_assert(sizeof(FluidFrame) <= 9020);

    static constexpr uint8_t kStateShadeMask = 0x07;
    static constexpr int kStateGlowShift = 3;
    static constexpr uint8_t kStateGlowMask = 0x38;
    static constexpr uint8_t kStateAsleep = 0x40;
    static constexpr uint8_t kStateOverlap = 0x80;
    static constexpr uint8_t kGridColorMask = 0x3F;
    static constexpr uint8_t kGridKickIncrement = 0x40;
    static constexpr uint8_t kGridKickMask = 0xC0;
    static constexpr uint8_t kWakeOnlyKickCount = 3;
    static constexpr uint8_t kMaxImpactKickCount = kWakeOnlyKickCount - 1;
    static constexpr int kGridKickShift = 6;
    static constexpr int kShadeCount = 6;
    static constexpr int kGlowLevelCount = 8;

    struct GravityState {
        enum class Mode : uint8_t {
            Flat,
            Tilted,
        };

        struct ResolvedGravity {
            float direction_x;
            float direction_y;
            float acceleration_units;
            bool wake_all;
        };

        ResolvedGravity resolve(float screen_gravity_x, float screen_gravity_y,
                                float screen_gravity_z);

        Mode mode = Mode::Flat;
        float direction_x = 0.0f;
        float direction_y = 1.0f;
        float wake_anchor_x = 0.0f;
        float wake_anchor_y = 1.0f;
    };

    void reset_particles();
    uint32_t step_particles(float screen_gravity_x, float screen_gravity_y,
                            float screen_gravity_z);
    void fill_frame(FluidFrame &frame);
    void draw_frame(const FluidFrame &frame, uint16_t *pixels, int stripe_y,
                    int stripe_rows);
    uint32_t next_random();

    MotionFilter motion_filter_;

    struct SharedMotion {
        Vec3 apparent_acceleration{0.0f, 0.0f, 9.0f};
        Vec3 raw_acceleration{0.0f, 0.0f, 0.0f};
        bool available{false};
    };
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_;

    std::atomic<bool> reset_requested_{false};

    std::atomic<uint32_t> epoch_{0};
    std::atomic<uint64_t> walk_steps_{0};
    std::atomic<uint32_t> governor_hits_{0};
    std::atomic<uint32_t> physics_us_{0};

    void *particle_arena_ = nullptr;
    uint16_t *particle_x_ = nullptr;
    uint16_t *particle_y_ = nullptr;
    int16_t *particle_velocity_x_ = nullptr;
    int16_t *particle_velocity_y_ = nullptr;
    uint8_t *particle_state_ = nullptr;
    uint8_t *particle_rest_frames_ = nullptr;
    uint8_t *occupancy_grid_ = nullptr;

    uint32_t random_state_ = 0x2545F491u;
    uint32_t frame_parity_ = 0;
    uint32_t awake_count_ = 0;
    uint32_t rest_gate_frames_ = 0;
    GravityState gravity_;
    LatestFrameExchange<FluidFrame, 3> frames_;

    uint16_t wire_palette_[64] = {};

    uint32_t frame_us_ = 0;
    uint32_t raster_us_ = 0;

    bool setup_done_ = false;
};

extern FluidBoxApp s_fluid_app;

}
