#include "orient_cube_app.hpp"

#include <cmath>
#include <cstdint>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "common_math.hpp"
#include "draw.hpp"
#include "launcher_icons.hpp"

namespace fluid_demo {

namespace {

constexpr char kTag[] = "orient_cube";
constexpr int kPanelWidth = 240;
constexpr int kPanelHeight = 240;

constexpr uint16_t kBackground = rgb24(0x101418);
constexpr uint16_t kBand = rgb24(0x294542);
constexpr uint16_t kMuted = rgb24(0x7B8A7B);
constexpr uint16_t kAccent = rgb24(0xF7A252);
constexpr uint16_t kFront = rgb24(0x2951B5);
constexpr uint16_t kFrontInset = rgb24(0x105584);
constexpr uint16_t kFrontGlass = rgb24(0x42BACE);
constexpr uint16_t kBack = rgb24(0x39494A);
constexpr uint16_t kBackMark = rgb24(0x636563);
constexpr uint16_t kRight = rgb24(0x4A9273);
constexpr uint16_t kLeft = rgb24(0x5A555A);
constexpr uint16_t kTop = rgb24(0xC6C7C6);
constexpr uint16_t kUsb = rgb24(0xC69600);
constexpr uint16_t kUsbMark = rgb24(0xFFDF00);
constexpr uint16_t kEdge = rgb24(0x000000);

constexpr LauncherVisual kLauncherVisual{
    kBackground, kBand, kMuted, kAccent, kIconCube,
};

constexpr int kCubeVertexCount = 8;
constexpr int kCubeFaceCount = 6;
constexpr int kVerticesPerFace = 4;
constexpr float kCubeHalfExtent = 1.0f;
constexpr float kCameraDistance = 5.5f;
constexpr float kFocalLength = 400.0f;
constexpr float kScreenCenter = 120.0f;
constexpr float kMinimumCameraDepth = 0.35f;
constexpr float kLightX = 0.42f;
constexpr float kLightY = 0.74f;
constexpr float kLightZ = 0.53f;

constexpr int kFaceVertexIndices[kCubeFaceCount][kVerticesPerFace] = {
    {0, 1, 2, 3}, {5, 4, 7, 6}, {1, 5, 6, 2},
    {4, 0, 3, 7}, {3, 2, 6, 7}, {4, 5, 1, 0},
};
constexpr uint16_t kFaceColors[kCubeFaceCount] = {
    kFront, kBack, kRight, kLeft, kTop, kUsb,
};

void copy_matrix(float *destination, const float *source)
{
    for (int element_index = 0; element_index < 9; ++element_index) {
        destination[element_index] = source[element_index];
    }
}

void project_vertices(const float rotation_matrix[9],
                      float camera_vertices[kCubeVertexCount][3],
                      float projected_vertices[kCubeVertexCount][3])
{
    const float model_vertices[kCubeVertexCount][3] = {
        {-kCubeHalfExtent, -kCubeHalfExtent, kCubeHalfExtent},
        {kCubeHalfExtent, -kCubeHalfExtent, kCubeHalfExtent},
        {kCubeHalfExtent, kCubeHalfExtent, kCubeHalfExtent},
        {-kCubeHalfExtent, kCubeHalfExtent, kCubeHalfExtent},
        {-kCubeHalfExtent, -kCubeHalfExtent, -kCubeHalfExtent},
        {kCubeHalfExtent, -kCubeHalfExtent, -kCubeHalfExtent},
        {kCubeHalfExtent, kCubeHalfExtent, -kCubeHalfExtent},
        {-kCubeHalfExtent, kCubeHalfExtent, -kCubeHalfExtent},
    };

    for (int vertex_index = 0; vertex_index < kCubeVertexCount;
         ++vertex_index) {
        const float camera_x =
            rotation_matrix[0] * model_vertices[vertex_index][0] +
            rotation_matrix[1] * model_vertices[vertex_index][1] +
            rotation_matrix[2] * model_vertices[vertex_index][2];
        const float camera_y =
            rotation_matrix[3] * model_vertices[vertex_index][0] +
            rotation_matrix[4] * model_vertices[vertex_index][1] +
            rotation_matrix[5] * model_vertices[vertex_index][2];
        const float camera_z =
            rotation_matrix[6] * model_vertices[vertex_index][0] +
            rotation_matrix[7] * model_vertices[vertex_index][1] +
            rotation_matrix[8] * model_vertices[vertex_index][2];

        camera_vertices[vertex_index][0] = camera_x;
        camera_vertices[vertex_index][1] = camera_y;
        camera_vertices[vertex_index][2] = camera_z;

        float camera_depth = kCameraDistance - camera_z;
        if (camera_depth < kMinimumCameraDepth) {
            camera_depth = kMinimumCameraDepth;
        }
        const float perspective_scale = kFocalLength / camera_depth;
        projected_vertices[vertex_index][0] =
            kScreenCenter + camera_x * perspective_scale;
        projected_vertices[vertex_index][1] =
            kScreenCenter - camera_y * perspective_scale;
        projected_vertices[vertex_index][2] = camera_z;
    }
}

float face_light(const float camera_vertices[kCubeVertexCount][3],
                 int face_index)
{
    const int first_vertex = kFaceVertexIndices[face_index][0];
    const int second_vertex = kFaceVertexIndices[face_index][1];
    const int third_vertex = kFaceVertexIndices[face_index][2];
    const float first_edge_x =
        camera_vertices[second_vertex][0] - camera_vertices[first_vertex][0];
    const float first_edge_y =
        camera_vertices[second_vertex][1] - camera_vertices[first_vertex][1];
    const float first_edge_z =
        camera_vertices[second_vertex][2] - camera_vertices[first_vertex][2];
    const float second_edge_x =
        camera_vertices[third_vertex][0] - camera_vertices[first_vertex][0];
    const float second_edge_y =
        camera_vertices[third_vertex][1] - camera_vertices[first_vertex][1];
    const float second_edge_z =
        camera_vertices[third_vertex][2] - camera_vertices[first_vertex][2];

    float normal_x =
        first_edge_y * second_edge_z - first_edge_z * second_edge_y;
    float normal_y =
        first_edge_z * second_edge_x - first_edge_x * second_edge_z;
    float normal_z =
        first_edge_x * second_edge_y - first_edge_y * second_edge_x;
    const float normal_length = std::sqrt(
        normal_x * normal_x + normal_y * normal_y + normal_z * normal_z);
    if (normal_length < 1e-5f) {
        return 0.35f;
    }
    normal_x /= normal_length;
    normal_y /= normal_length;
    normal_z /= normal_length;
    float light_dot =
        normal_x * kLightX + normal_y * kLightY + normal_z * kLightZ;
    if (light_dot < 0.0f) {
        light_dot = 0.0f;
    }
    return 0.48f + 0.52f * light_dot;
}

float face_area(const float projected_vertices[kCubeVertexCount][3],
                int face_index)
{
    float signed_area = 0.0f;
    for (int vertex_offset = 0; vertex_offset < kVerticesPerFace;
         ++vertex_offset) {
        const int current_vertex =
            kFaceVertexIndices[face_index][vertex_offset];
        const int next_vertex =
            kFaceVertexIndices[face_index]
                              [(vertex_offset + 1) % kVerticesPerFace];
        signed_area += projected_vertices[current_vertex][0] *
                           projected_vertices[next_vertex][1] -
                       projected_vertices[next_vertex][0] *
                           projected_vertices[current_vertex][1];
    }
    return signed_area;
}

void face_quad(const float projected_vertices[kCubeVertexCount][3],
               int face_index, float quad_coordinates[kVerticesPerFace * 2])
{
    for (int vertex_offset = 0; vertex_offset < kVerticesPerFace;
         ++vertex_offset) {
        const int vertex_index = kFaceVertexIndices[face_index][vertex_offset];
        quad_coordinates[2 * vertex_offset] =
            projected_vertices[vertex_index][0];
        quad_coordinates[2 * vertex_offset + 1] =
            projected_vertices[vertex_index][1];
    }
}

void scale_quad(const float quad_coordinates[kVerticesPerFace * 2], float scale,
                float scaled_coordinates[kVerticesPerFace * 2])
{
    const float center_x = 0.25f * (quad_coordinates[0] + quad_coordinates[2] +
                                    quad_coordinates[4] + quad_coordinates[6]);
    const float center_y = 0.25f * (quad_coordinates[1] + quad_coordinates[3] +
                                    quad_coordinates[5] + quad_coordinates[7]);
    for (int vertex_offset = 0; vertex_offset < kVerticesPerFace;
         ++vertex_offset) {
        scaled_coordinates[2 * vertex_offset] =
            center_x + scale * (quad_coordinates[2 * vertex_offset] - center_x);
        scaled_coordinates[2 * vertex_offset + 1] =
            center_y +
            scale * (quad_coordinates[2 * vertex_offset + 1] - center_y);
    }
}

void draw_edges(const float quad_coordinates[kVerticesPerFace * 2],
                uint16_t *pixels, int width, int stripe_y, int stripe_rows)
{
    for (int vertex_offset = 0; vertex_offset < kVerticesPerFace;
         ++vertex_offset) {
        const int next_vertex_offset = (vertex_offset + 1) % kVerticesPerFace;
        fill_segment(
            pixels, width, stripe_y, stripe_rows,
            static_cast<int>(quad_coordinates[2 * vertex_offset] + 0.5f),
            static_cast<int>(quad_coordinates[2 * vertex_offset + 1] + 0.5f),
            static_cast<int>(quad_coordinates[2 * next_vertex_offset] + 0.5f),
            static_cast<int>(quad_coordinates[2 * next_vertex_offset + 1] +
                             0.5f),
            1, kEdge);
    }
}
}

OrientCubeApp s_orient_cube_app;

const LauncherVisual *OrientCubeApp::launcher_visual() const
{
    return &kLauncherVisual;
}

esp_err_t OrientCubeApp::setup_once()
{
    if (setup_done_) {
        return ESP_OK;
    }
    attitude_filter_.reset();
    epoch_ = 1;
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    setup_done_ = true;
    return ESP_OK;
}

esp_err_t OrientCubeApp::enter()
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    frames_.drain();
    attitude_filter_.reset();
    portENTER_CRITICAL(&motion_mux_);
    copy_matrix(motion_.rotation_matrix, attitude_filter_.rotation_matrix());
    portEXIT_CRITICAL(&motion_mux_);
    return ESP_OK;
}

void OrientCubeApp::leave() {}

void OrientCubeApp::on_plus_press()
{
    reset_requested_.store(true, std::memory_order_release);
    attitude_filter_.request_align();
}

bool OrientCubeApp::on_motion(const MotionTick &tick)
{
    const bool physical_sample_valid =
        tick.fresh && std::isfinite(tick.dt) && tick.dt > 0.0f &&
        finite_vec(tick.accel_mps2) && finite_vec(tick.gyro_rads);
    const bool had_reference = attitude_filter_.aligned();
    bool override_accepted =
        tick.override_active && had_reference &&
        attitude_filter_.apply_override(tick.apparent_accel);

    bool physical_sample_accepted = false;
    if (physical_sample_valid) {
        physical_sample_accepted =
            override_accepted ||
            attitude_filter_.update(tick.accel_mps2, tick.gyro_rads, tick.dt);
    }
    if (tick.override_active && !override_accepted &&
        physical_sample_accepted && attitude_filter_.aligned()) {
        override_accepted =
            attitude_filter_.apply_override(tick.apparent_accel);
    }

    if (!physical_sample_accepted && !override_accepted) {
        return false;
    }

    portENTER_CRITICAL(&motion_mux_);
    motion_.apparent_acceleration =
        override_accepted ? tick.apparent_accel
                          : attitude_filter_.mapped_acceleration();
    copy_matrix(motion_.rotation_matrix, attitude_filter_.rotation_matrix());
    motion_.pitch = attitude_filter_.pitch();
    motion_.roll = attitude_filter_.roll();
    motion_.yaw = attitude_filter_.yaw();
    if (physical_sample_valid) {
        motion_.raw_acceleration = tick.accel_mps2;
    }
    portEXIT_CRITICAL(&motion_mux_);
    nonfinite_resets_.store(attitude_filter_.nonfinite_resets(),
                            std::memory_order_relaxed);
    return physical_sample_accepted;
}

esp_err_t OrientCubeApp::update(float dt)
{
    if (!setup_done_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!std::isfinite(dt) || dt <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t update_start_us = esp_timer_get_time();
    if (reset_requested_.exchange(false, std::memory_order_acq_rel)) {
        ++epoch_;
        if (epoch_ == 0u) {
            epoch_ = 1u;
        }
        ESP_LOGI(kTag, "cube align epoch=%" PRIu32, epoch_);
    }

    CubeFrame *snapshot = frames_.begin_write();
    if (snapshot != nullptr) {
        portENTER_CRITICAL(&motion_mux_);
        copy_matrix(snapshot->rotation_matrix, motion_.rotation_matrix);
        portEXIT_CRITICAL(&motion_mux_);
        frames_.publish(snapshot);
    }
    published_epoch_.store(epoch_, std::memory_order_relaxed);
    physics_us_.store(
        static_cast<uint32_t>(esp_timer_get_time() - update_start_us),
        std::memory_order_relaxed);
    return ESP_OK;
}

AppStats OrientCubeApp::stats()
{
    AppStats result{};
    result.count = 1;
    result.epoch = published_epoch_.load(std::memory_order_relaxed);
    result.nonfinite_resets = nonfinite_resets_.load(std::memory_order_relaxed);
    result.physics_us = physics_us_.load(std::memory_order_relaxed);
    portENTER_CRITICAL(&motion_mux_);
    result.raw[0] = motion_.raw_acceleration.x;
    result.raw[1] = motion_.raw_acceleration.y;
    result.raw[2] = motion_.raw_acceleration.z;
    result.apparent[0] = motion_.apparent_acceleration.x;
    result.apparent[1] = motion_.apparent_acceleration.y;
    result.apparent[2] = motion_.apparent_acceleration.z;
    result.pitch = motion_.pitch;
    result.roll = motion_.roll;
    result.yaw = motion_.yaw;
    portEXIT_CRITICAL(&motion_mux_);
    result.raster_us = raster_us_;
    result.frame_us = frame_us_;
    return result;
}

void OrientCubeApp::raster_stripe(
    const float projected_vertices[kCubeVertexCount][3],
    const float camera_vertices[kCubeVertexCount][3],
    const int face_order[kCubeFaceCount],
    const bool face_visible[kCubeFaceCount], uint16_t *pixels, int width,
    int stripe_y, int stripe_rows)
{
    for (int local_y = 0; local_y < stripe_rows; ++local_y) {
        uint16_t *row = pixels + local_y * width;
        for (int screen_x = 0; screen_x < width; ++screen_x) {
            row[screen_x] = kBackground;
        }
    }

    for (int order_index = 0; order_index < kCubeFaceCount; ++order_index) {
        const int face_index = face_order[order_index];
        if (!face_visible[face_index]) {
            continue;
        }

        float quad_coordinates[kVerticesPerFace * 2];
        face_quad(projected_vertices, face_index, quad_coordinates);
        const float light = face_light(camera_vertices, face_index);
        const uint16_t fill_color =
            shade_rgb565(kFaceColors[face_index], light);
        float expanded_quad[kVerticesPerFace * 2];
        scale_quad(quad_coordinates, 1.03f, expanded_quad);
        fill_convex_quad(pixels, width, stripe_y, stripe_rows, expanded_quad,
                         fill_color);

        const float projected_area =
            std::fabs(face_area(projected_vertices, face_index));
        if (projected_area <= 1200.0f) {
            continue;
        }

        float inset_quad[kVerticesPerFace * 2];
        if (face_index == 0) {
            scale_quad(quad_coordinates, 0.70f, inset_quad);
            fill_convex_quad(pixels, width, stripe_y, stripe_rows, inset_quad,
                             shade_rgb565(kFrontInset, light));
            scale_quad(quad_coordinates, 0.52f, inset_quad);
            fill_convex_quad(pixels, width, stripe_y, stripe_rows, inset_quad,
                             shade_rgb565(kFrontGlass, light * 0.90f));
        } else if (face_index == 1) {
            scale_quad(quad_coordinates, 0.28f, inset_quad);
            fill_convex_quad(pixels, width, stripe_y, stripe_rows, inset_quad,
                             shade_rgb565(kBackMark, light));
        } else if (face_index == 5) {
            scale_quad(quad_coordinates, 0.40f, inset_quad);
            fill_convex_quad(pixels, width, stripe_y, stripe_rows, inset_quad,
                             shade_rgb565(kUsbMark, light));
        }
    }

    for (int order_index = 0; order_index < kCubeFaceCount; ++order_index) {
        const int face_index = face_order[order_index];
        if (!face_visible[face_index]) {
            continue;
        }
        float quad_coordinates[kVerticesPerFace * 2];
        face_quad(projected_vertices, face_index, quad_coordinates);
        draw_edges(quad_coordinates, pixels, width, stripe_y, stripe_rows);
    }
}

bool OrientCubeApp::render(DisplayFrame &frame)
{
    if (!setup_done_ || frame.transport == nullptr ||
        frame.width != kPanelWidth || frame.height != kPanelHeight ||
        frame.stripe_rows <= 0 || frame.stripe_count <= 0 ||
        frame.stripe[0] == nullptr || frame.stripe[1] == nullptr ||
        frame.ops.wait_previous == nullptr ||
        frame.ops.latch_capture == nullptr || frame.ops.submit == nullptr ||
        frame.ops.finish == nullptr || frame.ops.capture_copy_us == nullptr) {
        ESP_LOGW(kTag, "render rejected invalid display frame");
        return false;
    }

    const CubeFrame *snapshot = frames_.acquire_latest();
    if (snapshot == nullptr) {
        return false;
    }

    float camera_vertices[kCubeVertexCount][3];
    float projected_vertices[kCubeVertexCount][3];
    project_vertices(snapshot->rotation_matrix, camera_vertices,
                     projected_vertices);
    bool face_is_visible[kCubeFaceCount];
    float face_depths[kCubeFaceCount];
    int face_order[kCubeFaceCount];
    for (int face_index = 0; face_index < kCubeFaceCount; ++face_index) {
        face_is_visible[face_index] =
            face_area(projected_vertices, face_index) < 0.0f;
        face_depths[face_index] =
            0.25f * (projected_vertices[kFaceVertexIndices[face_index][0]][2] +
                     projected_vertices[kFaceVertexIndices[face_index][1]][2] +
                     projected_vertices[kFaceVertexIndices[face_index][2]][2] +
                     projected_vertices[kFaceVertexIndices[face_index][3]][2]);
        face_order[face_index] = face_index;
    }
    for (int order_index = 1; order_index < kCubeFaceCount; ++order_index) {
        const int face_index = face_order[order_index];
        const float selected_depth = face_depths[face_index];
        int insertion_index = order_index;
        while (insertion_index > 0 &&
               face_depths[face_order[insertion_index - 1]] > selected_depth) {
            face_order[insertion_index] = face_order[insertion_index - 1];
            --insertion_index;
        }
        face_order[insertion_index] = face_index;
    }

    const int64_t frame_start_us = esp_timer_get_time();
    uint32_t total_raster_us = 0;
    esp_err_t error = frame.ops.wait_previous(frame.transport);
    if (error == ESP_OK) {
        frame.ops.latch_capture(frame.transport);
        for (int stripe_index = 0; stripe_index < frame.stripe_count;
             ++stripe_index) {
            const int stripe_y = stripe_index * frame.stripe_rows;
            const int stripe_rows =
                min_int(frame.stripe_rows, frame.height - stripe_y);
            if (stripe_rows <= 0) {
                break;
            }
            uint16_t *stripe_pixels = frame.stripe[stripe_index & 1];
            const int64_t raster_start_us = esp_timer_get_time();
            raster_stripe(projected_vertices, camera_vertices, face_order,
                          face_is_visible, stripe_pixels, frame.width, stripe_y,
                          stripe_rows);
            const int pixel_count = frame.width * stripe_rows;
            for (int pixel_index = 0; pixel_index < pixel_count;
                 ++pixel_index) {
                stripe_pixels[pixel_index] =
                    __builtin_bswap16(stripe_pixels[pixel_index]);
            }
            total_raster_us +=
                static_cast<uint32_t>(esp_timer_get_time() - raster_start_us);
            error = frame.ops.submit(frame.transport, stripe_index, stripe_y,
                                     stripe_rows, stripe_pixels);
            if (error != ESP_OK) {
                break;
            }
        }
        if (error == ESP_OK) {
            error = frame.ops.finish(frame.transport);
        }
    }

    frames_.release(snapshot);
    if (error != ESP_OK) {
        ESP_LOGW(kTag, "render transport failed: %s", esp_err_to_name(error));
        return false;
    }

    frame_us_ = static_cast<uint32_t>(esp_timer_get_time() - frame_start_us);
    raster_us_ = total_raster_us + frame.ops.capture_copy_us(frame.transport);
    return true;
}
}
