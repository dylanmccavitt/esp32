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

}  // namespace fluid_demo
