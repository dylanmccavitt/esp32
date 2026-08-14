#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
#include "attitude.hpp"
#include "frame_exchange.hpp"

namespace fluid_demo {

class OrientCubeApp final : public App {
public:
    OrientCubeApp() = default;
    ~OrientCubeApp() override = default;
    OrientCubeApp(const OrientCubeApp &) = delete;
    OrientCubeApp &operator=(const OrientCubeApp &) = delete;

    esp_err_t setup_once() override;
    esp_err_t enter() override;
    bool on_motion(const MotionTick &tick) override;
    const LauncherVisual *launcher_visual() const override;
    void on_plus_press() override;
    esp_err_t update(float dt = kPhysicsDt) override;
    bool render(DisplayFrame &frame) override;
    AppStats stats() override;
    void leave() override;

private:
    struct CubeFrame {
        float rotation_matrix[9] = {
            1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        };
    };

    struct SharedMotion {
        Vec3 apparent_acceleration{0.0f, 0.0f, 6.0f};
        Vec3 raw_acceleration{0.0f, 0.0f, 0.0f};
        float rotation_matrix[9] = {
            1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
        };
        float pitch = 0.0f;
        float roll = 0.0f;
        float yaw = 0.0f;
    };

    static void raster_stripe(const float projected_vertices[8][3],
                              const float camera_vertices[8][3],
                              const int face_order[6],
                              const bool face_visible[6], uint16_t *pixels,
                              int width, int stripe_y, int stripe_rows);

    // Launch and PLUS define the relative attitude identity.
    AttitudeFilter attitude_filter_{AttitudeFilter::ReferenceMode::Relative};
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_{};
    std::atomic<bool> reset_requested_{false};

    uint32_t epoch_ = 0;
    LatestFrameExchange<CubeFrame, 3> frames_;

    std::atomic<uint32_t> published_epoch_{0};
    std::atomic<uint32_t> physics_us_{0};
    std::atomic<uint32_t> nonfinite_resets_{0};
    uint32_t raster_us_ = 0;
    uint32_t frame_us_ = 0;
    bool setup_done_ = false;
};

extern OrientCubeApp s_orient_cube_app;

}
