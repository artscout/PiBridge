// SPDX-License-Identifier: MIT
// Small looks at the system: is a process running (/proc/*/comm), where is a helper tool.
#pragma once

#include <QString>
#include <QStringList>

namespace pibridge::app {

// True when a process whose /proc/<pid>/comm is `comm` exists (comm is cut at 15 characters by the kernel).
// `procRoot` is for tests.
bool processRunning(const QString& comm, const QString& procRoot = QStringLiteral("/proc"));

// The places a helper tool is looked for, in order: PATH, next to this binary, this build's tools/*/ dirs, the
// omarchyvr release build (~/gamedev/omarchyvr/rust/target/release).
QStringList toolDirs();
// The tool's full path, or empty when it is in none of them.
QString findTool(const QString& name);

} // namespace pibridge::app
