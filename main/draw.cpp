#include "draw.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fluid_demo {

void fill_disc(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
               int center_x, int center_y, int radius, uint16_t color)
{
    if (pixels == nullptr || width <= 0 || stripe_rows <= 0 || radius < 0) {
        return;
    }
    const int top = std::max(stripe_y, center_y - radius);
    const int bottom = std::min(stripe_y + stripe_rows - 1, center_y + radius);
    const int left = std::max(0, center_x - radius);
    const int right = std::min(width - 1, center_x + radius);
    const int radius_squared = radius * radius;
    for (int screen_y = top; screen_y <= bottom; ++screen_y) {
        uint16_t *row = pixels + (screen_y - stripe_y) * width;
        const int vertical_offset = screen_y - center_y;
        for (int screen_x = left; screen_x <= right; ++screen_x) {
            const int horizontal_offset = screen_x - center_x;
            if (horizontal_offset * horizontal_offset +
                    vertical_offset * vertical_offset <=
                radius_squared) {
                row[screen_x] = color;
            }
        }
    }
}

void fill_rect(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
               int left, int top, int right, int bottom, uint16_t color)
{
    if (pixels == nullptr || width <= 0 || stripe_rows <= 0 || left >= right ||
        top >= bottom) {
        return;
    }
    const int clipped_left = std::max(0, left);
    const int clipped_right = std::min(width, right);
    const int clipped_top = std::max(stripe_y, top);
    const int clipped_bottom = std::min(stripe_y + stripe_rows, bottom);
    if (clipped_left >= clipped_right || clipped_top >= clipped_bottom) {
        return;
    }
    for (int screen_y = clipped_top; screen_y < clipped_bottom; ++screen_y) {
        uint16_t *row = pixels + (screen_y - stripe_y) * width;
        for (int screen_x = clipped_left; screen_x < clipped_right;
             ++screen_x) {
            row[screen_x] = color;
        }
    }
}

void fill_segment(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
                  int start_x, int start_y, int end_x, int end_y, int radius,
                  uint16_t color)
{
    const int horizontal_delta = end_x - start_x;
    const int vertical_delta = end_y - start_y;
    const int extent =
        std::max(std::abs(horizontal_delta), std::abs(vertical_delta));
    const int sample_spacing = std::max(1, radius);
    const int sample_count = std::max(1, extent / sample_spacing);
    for (int sample_index = 0; sample_index <= sample_count; ++sample_index) {
        fill_disc(pixels, width, stripe_y, stripe_rows,
                  start_x + (horizontal_delta * sample_index) / sample_count,
                  start_y + (vertical_delta * sample_index) / sample_count,
                  radius, color);
    }
}

void fill_convex(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
                 const float *coordinates, int vertex_count, uint16_t color)
{
    if (pixels == nullptr || coordinates == nullptr || width <= 0 ||
        stripe_rows <= 0 || vertex_count < 3 || vertex_count > 8) {
        return;
    }

    float minimum_y = coordinates[1];
    float maximum_y = coordinates[1];
    for (int vertex_index = 1; vertex_index < vertex_count; ++vertex_index) {
        const float vertex_y = coordinates[2 * vertex_index + 1];
        if (vertex_y < minimum_y) {
            minimum_y = vertex_y;
        }
        if (vertex_y > maximum_y) {
            maximum_y = vertex_y;
        }
    }
    const int top = std::max(stripe_y, static_cast<int>(std::floor(minimum_y)));
    const int bottom = std::min(stripe_y + stripe_rows - 1,
                                static_cast<int>(std::ceil(maximum_y)));
    if (top > bottom) {
        return;
    }

    for (int screen_y = top; screen_y <= bottom; ++screen_y) {
        const float scanline_y = static_cast<float>(screen_y) + 0.5f;
        float minimum_intersection_x = 1.0e9f;
        float maximum_intersection_x = -1.0e9f;
        int intersection_count = 0;
        for (int vertex_index = 0; vertex_index < vertex_count;
             ++vertex_index) {
            const int next_vertex_index = (vertex_index + 1) % vertex_count;
            const float start_x = coordinates[2 * vertex_index];
            const float start_y = coordinates[2 * vertex_index + 1];
            const float end_x = coordinates[2 * next_vertex_index];
            const float end_y = coordinates[2 * next_vertex_index + 1];
            const bool misses_scanline =
                (start_y < scanline_y && end_y < scanline_y) ||
                (start_y >= scanline_y && end_y >= scanline_y);
            if (misses_scanline) {
                continue;
            }
            const float vertical_delta = end_y - start_y;
            if (std::fabs(vertical_delta) < 1e-6f) {
                continue;
            }
            const float interpolation = (scanline_y - start_y) / vertical_delta;
            if (interpolation < 0.0f || interpolation > 1.0f) {
                continue;
            }
            const float intersection_x =
                start_x + interpolation * (end_x - start_x);
            if (intersection_x < minimum_intersection_x) {
                minimum_intersection_x = intersection_x;
            }
            if (intersection_x > maximum_intersection_x) {
                maximum_intersection_x = intersection_x;
            }
            ++intersection_count;
        }
        if (intersection_count < 2 ||
            maximum_intersection_x < minimum_intersection_x) {
            continue;
        }
        int left = static_cast<int>(std::floor(minimum_intersection_x + 0.5f));
        int right = static_cast<int>(std::floor(maximum_intersection_x + 0.5f));
        if (left < 0) {
            left = 0;
        }
        if (right >= width) {
            right = width - 1;
        }
        if (left > right) {
            continue;
        }
        uint16_t *row = pixels + (screen_y - stripe_y) * width;
        for (int screen_x = left; screen_x <= right; ++screen_x) {
            row[screen_x] = color;
        }
    }
}

void fill_triangle(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
                   float x0, float y0, float x1, float y1, float x2, float y2,
                   uint16_t color)
{
    if (pixels == nullptr || width <= 0 || stripe_rows <= 0) {
        return;
    }
    const float signed_double_area =
        (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (std::fabs(signed_double_area) < 0.25f) {
        return;
    }
    const float minimum_x = std::min(std::min(x0, x1), x2);
    const float maximum_x = std::max(std::max(x0, x1), x2);
    const float minimum_y = std::min(std::min(y0, y1), y2);
    const float maximum_y = std::max(std::max(y0, y1), y2);
    const int top = std::max(stripe_y, static_cast<int>(std::floor(minimum_y)));
    const int bottom = std::min(stripe_y + stripe_rows - 1,
                                static_cast<int>(std::ceil(maximum_y)));
    const int left = std::max(0, static_cast<int>(std::floor(minimum_x)));
    const int right =
        std::min(width - 1, static_cast<int>(std::ceil(maximum_x)));
    if (top > bottom || left > right) {
        return;
    }

    const float edge_01_x = y0 - y1;
    const float edge_01_y = x1 - x0;
    const float edge_12_x = y1 - y2;
    const float edge_12_y = x2 - x1;
    const float edge_20_x = y2 - y0;
    const float edge_20_y = x0 - x2;
    const float orientation = signed_double_area > 0.0f ? 1.0f : -1.0f;
    for (int screen_y = top; screen_y <= bottom; ++screen_y) {
        const float pixel_y = static_cast<float>(screen_y) + 0.5f;
        uint16_t *row = pixels + (screen_y - stripe_y) * width;
        for (int screen_x = left; screen_x <= right; ++screen_x) {
            const float pixel_x = static_cast<float>(screen_x) + 0.5f;
            const float edge_12_value =
                orientation *
                (edge_12_x * (pixel_x - x1) + edge_12_y * (pixel_y - y1));
            const float edge_20_value =
                orientation *
                (edge_20_x * (pixel_x - x2) + edge_20_y * (pixel_y - y2));
            const float edge_01_value =
                orientation *
                (edge_01_x * (pixel_x - x0) + edge_01_y * (pixel_y - y0));
            if (edge_12_value >= -0.35f && edge_20_value >= -0.35f &&
                edge_01_value >= -0.35f) {
                row[screen_x] = color;
            }
        }
    }
}

void fill_convex_quad(uint16_t *pixels, int width, int stripe_y,
                      int stripe_rows, const float *coordinates, uint16_t color)
{
    fill_convex(pixels, width, stripe_y, stripe_rows, coordinates, 4, color);
}

uint16_t shade_rgb565(uint16_t color, float light)
{
    light = std::clamp(light, 0.0f, 1.0f);
    const int red = std::min(
        31, static_cast<int>(static_cast<float>((color >> 11) & 0x1F) * light +
                             0.5f));
    const int green = std::min(
        63, static_cast<int>(static_cast<float>((color >> 5) & 0x3F) * light +
                             0.5f));
    const int blue = std::min(
        31, static_cast<int>(static_cast<float>(color & 0x1F) * light + 0.5f));
    return static_cast<uint16_t>((red << 11) | (green << 5) | blue);
}

}
