#pragma once

#include <cstdint>

#include "esp_err.h"

#include "app_shell.hpp"

namespace fluid_demo {

struct RegistryEntry {
    const char *id;
    const char *label;
    App *app;
};

const RegistryEntry *registry();

// Packed dispatch layout shared by all runtime lanes.
constexpr uint32_t kAppIndexMask = 0x0Fu;
constexpr uint32_t kAppModeShift = 4;
constexpr uint32_t kAppModeMask = 0x3u << kAppModeShift;
constexpr uint32_t kAppGenShift = kAppModeShift + 2;
constexpr uint32_t kNoAppIndex = 0x0Fu;

const char *runtime_mode_name();
/// Copy the latest render-lane telemetry for the current run generation.
/// Returns false until that generation has completed a rendered frame.
bool runtime_active_stats(AppStats &stats);

[[noreturn]] void runtime_run();

}  // namespace fluid_demo
