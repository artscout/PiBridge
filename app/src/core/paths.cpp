// SPDX-License-Identifier: MIT
#include "paths.h"

#include <QDir>
#include <QFileInfo>
#include <unistd.h>

namespace pibridge::app {

namespace {

QString configBase() {
    const QByteArray x = qgetenv("XDG_CONFIG_HOME");
    return x.isEmpty() ? QDir::homePath() + QStringLiteral("/.config") : QString::fromLocal8Bit(x);
}

} // namespace

QString installPrefix() {
    if (const QByteArray o = qgetenv("PIBRIDGE_PREFIX"); !o.isEmpty()) {
        return QString::fromLocal8Bit(o);
    }
    // relocatable: <prefix>/bin/pibridge -> <prefix>
    char buf[4096];
    const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n > 0) {
        buf[n] = '\0';
        QDir d = QFileInfo(QString::fromLocal8Bit(buf, static_cast<int>(n))).absoluteDir(); // <prefix>/bin
        if (d.dirName() == QLatin1String("bin")) {
            d.cdUp();
            return d.absolutePath();
        }
    }
    return QDir::homePath() + QStringLiteral("/.local");
}

QString dataDir() {
    return installPrefix() + QStringLiteral("/share/pibridge");
}

QString libDir() {
    return installPrefix() + QStringLiteral("/lib/pibridge");
}

QString defaultsDir() {
    return installPrefix() + QStringLiteral("/etc/pibridge");
}

QString configDir() {
    return configBase() + QStringLiteral("/pibridge");
}

QString confPath() {
    return configDir() + QStringLiteral("/pibridge.conf");
}

QString legacyConfPath() {
    return configBase() + QStringLiteral("/pimax-linux/pimax-linux.conf");
}

QString defaultsConfPath() {
    return defaultsDir() + QStringLiteral("/pibridge.conf");
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
