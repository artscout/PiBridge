// SPDX-License-Identifier: MIT
#include "paths.h"

#include <QDir>
#include <unistd.h>

namespace pibridge::app {

namespace {

QString configBase() {
    const QByteArray x = qgetenv("XDG_CONFIG_HOME");
    return x.isEmpty() ? QDir::homePath() + QStringLiteral("/.config") : QString::fromLocal8Bit(x);
}

} // namespace

QString configDir() {
    return configBase() + QStringLiteral("/pibridge");
}

QString confPath() {
    return configDir() + QStringLiteral("/pibridge.conf");
}

QString legacyConfPath() {
    return configBase() + QStringLiteral("/pimax-linux/pimax-linux.conf");
}

QStringList eyeFileDirs() {
    return {configDir(), configBase() + QStringLiteral("/pimax-linux"), QDir::homePath() + QStringLiteral("/.config/omarchyvr")};
}

QString statusDir() {
    const QByteArray x = qgetenv("XDG_RUNTIME_DIR");
    if (!x.isEmpty()) {
        return QString::fromLocal8Bit(x) + QStringLiteral("/pibridge");
    }
    return QStringLiteral("/tmp/pibridge-%1").arg(::getuid());
}

QString eyeProfileOut() {
    return configDir() + QStringLiteral("/eye-profile-crystal-super.bin");
}

} // namespace pibridge::app
