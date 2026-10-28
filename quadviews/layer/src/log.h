// SPDX-License-Identifier: MIT
// The layer's log: $XDG_STATE_HOME/qv-vulkan/qv-vulkan.log (default ~/.local/state/qv-vulkan), started anew by each
// process; also stderr when QV_VULKAN_LOG_STDERR=1.
#pragma once

namespace qv {

void logOpen();
void logf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

} // namespace qv
