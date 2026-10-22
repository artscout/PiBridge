// SPDX-License-Identifier: MIT
#include "system.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>

namespace pibridge::app {

bool processRunning(const QString& comm, const QString& procRoot) {
    const QString want = comm.left(15);
    const QDir proc(procRoot);
    const QStringList pids = proc.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QString& pid : pids) {
        if (pid.isEmpty() || !pid.front().isDigit()) {
            continue;
        }
        QFile f(procRoot + QLatin1Char('/') + pid + QStringLiteral("/comm"));
        if (f.open(QIODevice::ReadOnly) && QString::fromLocal8Bit(f.readAll()).trimmed() == want) {
            return true;
        }
    }
    return false;
}

QStringList toolDirs() {
    QStringList dirs;
    if (QCoreApplication::instance()) {
        const QString app = QCoreApplication::applicationDirPath();
        dirs << app;
        // build tree: <build>/app/pibridge next to <build>/tools/<part>/<tool>
        const QDir tools(app + QStringLiteral("/../tools"));
        for (const QString& d : tools.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            dirs << QDir::cleanPath(tools.absoluteFilePath(d));
        }
    }
    return dirs;
}

QString findTool(const QString& name) {
    if (const QString p = QStandardPaths::findExecutable(name); !p.isEmpty()) {
        return p;
    }
    for (const QString& d : toolDirs()) {
        const QFileInfo fi(d + QLatin1Char('/') + name);
        if (fi.isFile() && fi.isExecutable()) {
            return fi.absoluteFilePath();
        }
    }
    return {};
}

} // namespace pibridge::app
