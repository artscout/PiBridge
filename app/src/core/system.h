// SPDX-License-Identifier: MIT
// Small looks at the system: is a process running (/proc/*/comm), where is a helper tool.
#pragma once

#include <QString>
#include <QStringList>

namespace pibridge::app {

// True when a process whose /proc/<pid>/comm is `comm` exists (comm is cut at 15 characters by the kernel).
// `procRoot` is for tests.
bool processRunning(const QString& comm, const QString& procRoot = QStringLiteral("/proc"));
// The processes of an inside-out session a previous PiBridge started and left behind (pibridge-cam, pibridge-ctrltrack,
// pibridge-headtrack, winbridge.exe; e.g. PiBridge restarted while it ran): SIGTERM, SIGKILL after 2 s. Only while no
// vrserver runs (a running session's are never touched). Returns what was stopped ("" names = nothing).
QStringList stopLeftoverSession();

// The places a helper tool is looked for, in order: PATH, next to this binary, this build's tools/*/ dirs.
QStringList toolDirs();
// The tool's full path, or empty when it is in none of them.
QString findTool(const QString& name);

} // namespace pibridge::app
