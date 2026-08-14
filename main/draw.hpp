#pragma once

#include <cstdint>

namespace fluid_demo {

void fill_disc(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
               int center_x, int center_y, int radius, uint16_t color);

void fill_rect(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
               int left, int top, int right, int bottom, uint16_t color);

void fill_segment(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
                  int start_x, int start_y, int end_x, int end_y, int radius,
                  uint16_t color);

void fill_convex(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
                 const float *coordinates, int vertex_count, uint16_t color);

void fill_convex_quad(uint16_t *pixels, int width, int stripe_y,
                      int stripe_rows, const float *coordinates,
                      uint16_t color);

void fill_triangle(uint16_t *pixels, int width, int stripe_y, int stripe_rows,
                   float x0, float y0, float x1, float y1, float x2, float y2,
                   uint16_t color);

uint16_t shade_rgb565(uint16_t color, float light);

constexpr uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    return static_cast<uint16_t>(
        (static_cast<uint16_t>(red & 0xF8) << 8) |
        (static_cast<uint16_t>(green & 0xFC) << 3) |
        (static_cast<uint16_t>(blue) >> 3));
}

constexpr uint16_t rgb24(uint32_t hex)
{
    return rgb565(static_cast<uint8_t>(hex >> 16),
                  static_cast<uint8_t>(hex >> 8), static_cast<uint8_t>(hex));
}

}
