#pragma once

#include "app_shell.hpp"

namespace fluid_demo {

const char *runtime_mode_name();
bool runtime_active_stats(AppStats &stats);

[[noreturn]] void runtime_run();

}
