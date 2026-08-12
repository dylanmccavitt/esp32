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

}  // namespace fluid_demo
