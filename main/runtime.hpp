#pragma once

#include <cstdint>

#include "esp_err.h"

#include "app_shell.hpp"

namespace fluid_demo {

/// One compile-time registered app. The registry is built by the coordinator
/// (runtime.cpp) so it can bind &s_fluid_app without creating a header cycle;
/// it currently contains {fluid_box, "Fluid Box"}.
struct RegistryEntry {
    const char *id;   ///< Stable machine id — also the @DEV MODE name.
    const char *label;  ///< Human-readable label.
    App *app;         ///< Lifecycle-managed app instance.
};

/// The compile-time app registry (defined in runtime.cpp). Bounded by
/// kAppIndexMask: the packed selection stores the index in the low 4 bits.
const RegistryEntry *registry();

/// Packed active dispatch: one std::atomic<uint32_t> read exactly once per
/// lane iteration, so a lane can never observe a torn (app, mode,
/// generation) triple. Low 4 bits = selected registry index (kNoAppIndex
/// while no app runs); bits 4-5 = the AppMode published by the coordinator
/// (Transition while quiescing, Launcher or Running once committed — a lane
/// derives everything it needs from a single acquire load); high 26 bits =
/// monotonic quiesce/run generation bumped once per committed transition.
constexpr uint32_t kAppIndexMask = 0x0Fu;
constexpr uint32_t kAppModeShift = 4;
constexpr uint32_t kAppModeMask = 0x3u << kAppModeShift;
constexpr uint32_t kAppGenShift = kAppModeShift + 2;
/// No registered app selected: the shell's launcher stable mode. App
/// callbacks never run for it; the render lane draws the launcher there.
constexpr uint32_t kNoAppIndex = 0x0Fu;

/// Coordinator requests. The coordinator on the ESP main task consumes a
/// small bounded queue of these and runs every transition to completion
/// synchronously; no caller ever performs a transition inline. Requests
/// whose target equals the already committed side are no-ops.
enum class RuntimeRequest : uint8_t {
    Launch = 0,  ///< Launcher -> Fluid Box (PLUS from the launcher).
    Home = 1,    ///< Fluid Box -> launcher (short PWR while Fluid runs).
};

/// Enqueue a request for the coordinator. Never blocks and never performs the
/// transition synchronously. ESP_OK when queued; ESP_ERR_NO_MEM when the
/// bounded queue is full (the sensor lane retains and retries target intents).
esp_err_t runtime_enqueue_request(RuntimeRequest request);

/// Coordinator-side committed stable-mode name for host telemetry: "launcher"
/// or the active registry id ("fluid_box"). Safe from any task; no lifecycle
/// effect. Updated once per committed transition and at boot.
const char *runtime_mode_name();

/// Enter the runtime: complete boot wiring (board -> display -> console ->
/// Fluid setup), boot the launcher stable mode WITHOUT calling the Fluid
/// app's enter(), create the three pinned lanes exactly once, then run the
/// coordinator loop on the ESP main task forever. Never returns.
[[noreturn]] void runtime_run();

}  // namespace fluid_demo
