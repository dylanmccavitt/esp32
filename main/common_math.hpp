#pragma once

#include <cmath>
#include <cstdint>

#include "app_types.hpp"

namespace fluid_demo {

constexpr int min_int(int left, int right)
{
    return left < right ? left : right;
}

constexpr int max_int(int left, int right)
{
    return left > right ? left : right;
}

constexpr int abs_int(int value)
{
    return value < 0 ? -value : value;
}

constexpr float clamp_float(float value, float lower, float upper)
{
    return value < lower ? lower : (value > upper ? upper : value);
}

constexpr int32_t clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    return value < lower ? lower : (value > upper ? upper : value);
}

inline bool finite_vec(const Vec3 &value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z);
}

}
