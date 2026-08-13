#pragma once

#include <cstdint>

namespace fluid_demo {

void fill_disc(uint16_t *pixels, int width, int y0, int rows, int cx, int cy,
               int radius, uint16_t color);

void fill_segment(uint16_t *pixels, int width, int y0, int rows, int x0, int y0_screen,
                  int x1, int y1, int radius, uint16_t color);

/// Convex n-gon in screen pixels, clipped to the current stripe. `xy` is
/// interleaved x,y with `count` vertices (3..8).
void fill_convex(uint16_t *pixels, int width, int y0, int rows, const float *xy,
                 int count, uint16_t color);

void fill_convex_quad(uint16_t *pixels, int width, int y0, int rows, const float *xy,
                      uint16_t color);

void fill_triangle(uint16_t *pixels, int width, int y0, int rows, float x0, float y0s,
                   float x1, float y1, float x2, float y2, uint16_t color);

uint16_t shade_rgb565(uint16_t color, float light);

constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(r & 0xF8) << 8) |
                                 (static_cast<uint16_t>(g & 0xFC) << 3) |
                                 (static_cast<uint16_t>(b) >> 3));
}

constexpr uint16_t rgb24(uint32_t hex)
{
    return rgb565(static_cast<uint8_t>(hex >> 16), static_cast<uint8_t>(hex >> 8),
                  static_cast<uint8_t>(hex));
}

}  // namespace fluid_demo
