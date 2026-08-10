#pragma once

#include <cstdint>

#include "esp_err.h"

#include "app_shell.hpp"

namespace fluid_demo {

/// One compile-time registered app. The registry is built by the coordinator
/// (runtime.cpp) so it can bind &s_fluid_app without creating a header cycle;
/// it contains exactly one entry in S4: {fluid_box, "Fluid Box"}.
struct RegistryEntry {
    const char *id;   ///< Stable machine id — also the @DEV MODE name.
    const char *label;  ///< Human-readable label.
    App *app;         ///< Lifecycle-managed app instance.
};

/// The compile-time app registry (defined in runtime.cpp). Bounded by
/// kAppIndexMask: the packed selection stores the index in the low 4 bits.
const RegistryEntry *registry();

/// Packed active selection: one std::atomic<uint32_t> read exactly once per
/// lane iteration, so a lane can never observe a torn (app, generation)
/// pair. Low 4 bits = selected registry index; high 28 bits = monotonic
/// quiesce/run generation bumped once per committed transition. AppMode stays
/// coordinator-side only — a lane derives everything it needs (run the app vs
/// park without app callbacks) from a single acquire load.
constexpr uint32_t kAppIndexMask = 0x0Fu;
constexpr uint32_t kAppGenShift = 4;
/// No registered app selected: the shell's launcher/idle state. S4 draws
/// nothing there (S5 renders the launcher); app callbacks never run for it.
constexpr uint32_t kNoAppIndex = 0x0Fu;

/// Coordinator requests. The coordinator on the ESP main task consumes a
/// small bounded queue of these and runs every transition to completion
/// synchronously; no caller ever performs a transition inline.
enum class RuntimeRequest : uint8_t {
    NextMode = 0,  ///< Toggle Fluid Box <-> launcher/idle (temporary S4 barrier exercise).
};

/// Enqueue a request for the coordinator. Never blocks and never performs the
/// transition synchronously. ESP_OK when queued; ESP_ERR_NO_MEM when the
/// bounded queue is full (the requester is expected to report the backpressure).
esp_err_t runtime_enqueue_request(RuntimeRequest request);

/// Enter the runtime: complete boot wiring (board -> display -> console ->
/// Fluid setup -> enter), create the three pinned lanes exactly once, then run
/// the coordinator loop on the ESP main task forever. Never returns.
[[noreturn]] void runtime_run();

}  // namespace fluid_demo
