// app_main.cpp — fluid_box_3d application glue (startup wiring only).
//
// All lane, coordinator, registry and shell-service ownership lives in
// runtime.cpp. This file only hands control to the runtime, which boots the
// board (board -> DisplayService -> ConsoleService ->
// Fluid setup) on the ESP main task, creates the three pinned lanes exactly
// once (sensor core0 prio 7 100 Hz, physics core1 prio 8 30 Hz, render core0
// prio 5 30 Hz), then never returns: the main task becomes the coordinator
// loop that runs the generation-barrier transitions (quiesce -> 3-lane acks
// -> DisplayService::drain -> leave/enter -> run publish) on request.
//
// No tasks are recreated and no hardware is re-initialized after boot.

#include "runtime.hpp"

extern "C" void app_main(void)
{
    // Startup wiring only; the runtime owns the boot sequence, the lanes and
    // the coordinator loop, and never returns.
    fluid_demo::runtime_run();
}
