#include "draw.hpp"

#include <cmath>
#include <cstdint>

namespace fluid_demo {
namespace {

inline int min_int(int a, int b) { return a < b ? a : b; }
inline int max_int(int a, int b) { return a > b ? a : b; }
inline int abs_int(int value) { return value < 0 ? -value : value; }

}  // namespace

void fill_disc(uint16_t *pixels, int width, int y0, int rows, int cx, int cy,
               int radius, uint16_t color)
{
    if (pixels == nullptr || width <= 0 || rows <= 0 || radius < 0) {
        return;
    }
    const int top = max_int(y0, cy - radius);
    const int bottom = min_int(y0 + rows - 1, cy + radius);
    const int left = max_int(0, cx - radius);
    const int right = min_int(width - 1, cx + radius);
    const int radius_squared = radius * radius;
    for (int y = top; y <= bottom; ++y) {
        uint16_t *row = pixels + (y - y0) * width;
        const int dy = y - cy;
        for (int x = left; x <= right; ++x) {
            const int dx = x - cx;
            if (dx * dx + dy * dy <= radius_squared) {
                row[x] = color;
            }
        }
    }
}

void fill_segment(uint16_t *pixels, int width, int y0, int rows, int x0, int y0_screen,
                  int x1, int y1, int radius, uint16_t color)
{
    const int dx = x1 - x0;
    const int dy = y1 - y0_screen;
    const int extent = max_int(abs_int(dx), abs_int(dy));
    const int stride = max_int(1, radius);
    const int samples = max_int(1, extent / stride);
    for (int sample = 0; sample <= samples; ++sample) {
        fill_disc(pixels, width, y0, rows, x0 + (dx * sample) / samples,
                  y0_screen + (dy * sample) / samples, radius, color);
    }
}

void fill_convex(uint16_t *pixels, int width, int y0, int rows, const float *xy,
                 int count, uint16_t color)
{
    if (pixels == nullptr || xy == nullptr || width <= 0 || rows <= 0 || count < 3 ||
        count > 8) {
        return;
    }

    float y_min = xy[1];
    float y_max = xy[1];
    for (int i = 1; i < count; ++i) {
        const float y = xy[2 * i + 1];
        if (y < y_min) {
            y_min = y;
        }
        if (y > y_max) {
            y_max = y;
        }
    }
    const int top = max_int(y0, static_cast<int>(std::floor(y_min)));
    const int bottom = min_int(y0 + rows - 1, static_cast<int>(std::ceil(y_max)));
    if (top > bottom) {
        return;
    }

    for (int y = top; y <= bottom; ++y) {
        const float scan = static_cast<float>(y) + 0.5f;
        float x_min = 1.0e9f;
        float x_max = -1.0e9f;
        int hits = 0;
        for (int i = 0; i < count; ++i) {
            const int j = (i + 1) % count;
            const float x0 = xy[2 * i];
            const float y0e = xy[2 * i + 1];
            const float x1 = xy[2 * j];
            const float y1e = xy[2 * j + 1];
            const bool skip = (y0e < scan && y1e < scan) || (y0e >= scan && y1e >= scan);
            if (skip) {
                continue;
            }
            const float dy = y1e - y0e;
            if (std::fabs(dy) < 1e-6f) {
                continue;
            }
            const float t = (scan - y0e) / dy;
            if (t < 0.0f || t > 1.0f) {
                continue;
            }
            const float x = x0 + t * (x1 - x0);
            if (x < x_min) {
                x_min = x;
            }
            if (x > x_max) {
                x_max = x;
            }
            ++hits;
        }
        if (hits < 2 || x_max < x_min) {
            continue;
        }
        int left = static_cast<int>(std::floor(x_min + 0.5f));
        int right = static_cast<int>(std::floor(x_max + 0.5f));
        if (left < 0) {
            left = 0;
        }
        if (right >= width) {
            right = width - 1;
        }
        if (left > right) {
            continue;
        }
        uint16_t *row = pixels + (y - y0) * width;
        for (int x = left; x <= right; ++x) {
            row[x] = color;
        }
    }
}

void fill_convex_quad(uint16_t *pixels, int width, int y0, int rows, const float *xy,
                      uint16_t color)
{
    fill_convex(pixels, width, y0, rows, xy, 4, color);
}

}  // namespace fluid_demo
