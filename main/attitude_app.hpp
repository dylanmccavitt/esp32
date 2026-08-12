#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"

#include "app_shell.hpp"
#include "attitude.hpp"
#include "frame_exchange.hpp"

namespace fluid_demo {

class AttitudeApp final : public App {
public:
    AttitudeApp() = default;
    ~AttitudeApp() override = default;
    AttitudeApp(const AttitudeApp &) = delete;
    AttitudeApp &operator=(const AttitudeApp &) = delete;

    esp_err_t setup_once() override;
    esp_err_t enter() override;
    bool on_motion(const MotionTick &tick) override;
    const LauncherVisual *launcher_visual() const override;
    ShellAction handle_event(AppEvent event) override;
    esp_err_t update(float dt = kPhysicsDt) override;
    bool render(DisplayFrame &frame) override;
    AppStats stats() override;
    void leave() override;

private:
    struct HorizonFrame {
        uint32_t sequence = 0;
        uint32_t epoch = 0;
        float up_x = 0.0f;
        float up_y = 1.0f;
        float up_z = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        bool ladder_ok = true;
    };

    struct SharedMotion {
        Vec3 apparent{0.0f, 0.0f, 6.0f};
        Vec3 raw{0.0f, 0.0f, 0.0f};
        float up_x = 0.0f;
        float up_y = 1.0f;
        float up_z = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float gyro_abs = 0.0f;
        bool valid = false;
    };

    void fill_snapshot(HorizonFrame &snapshot);
    static void raster_stripe(const HorizonFrame &horizon, uint16_t *pixels, int width,
                              int y0, int rows);

    AttitudeFilter filter_;
    portMUX_TYPE motion_mux_ = portMUX_INITIALIZER_UNLOCKED;
    SharedMotion motion_{};
    std::atomic<bool> reset_requested_{false};

    uint32_t sequence_ = 0;
    uint32_t epoch_ = 0;
    LatestFrameExchange<HorizonFrame, 3> frames_;

    std::atomic<uint32_t> published_epoch_{0};
    std::atomic<uint32_t> physics_us_{0};
    std::atomic<uint32_t> nonfinite_resets_{0};
    uint32_t raster_us_ = 0;
    uint32_t frame_us_ = 0;
    bool setup_done_ = false;
};

extern AttitudeApp s_attitude_app;

}  // namespace fluid_demo
