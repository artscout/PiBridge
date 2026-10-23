// SPDX-License-Identifier: MIT
#include "system.h"

#include <QCoreApplication>
#include <QThread>
#include <csignal>
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

QStringList stopLeftoverSession() {
    if (processRunning(QStringLiteral("vrserver"))) {
        return {};
    }
    static const QStringList comms = {QStringLiteral("pibridge-cam"), QStringLiteral("pibridge-ctrltr"),
                                      QStringLiteral("pibridge-headtr"), QStringLiteral("winbridge.exe")};
    const auto find = [] {
        QList<std::pair<pid_t, QString>> out;
        const QDir proc(QStringLiteral("/proc"));
        for (const QString& pid : proc.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            if (pid.isEmpty() || !pid.front().isDigit()) {
                continue;
            }
            QFile f(QStringLiteral("/proc/") + pid + QStringLiteral("/comm"));
            if (!f.open(QIODevice::ReadOnly)) {
                continue;
            }
            const QString comm = QString::fromLocal8Bit(f.readAll()).trimmed();
            if (comms.contains(comm)) {
                out.append({static_cast<pid_t>(pid.toInt()), comm});
            }
        }
        return out;
    };
    QStringList stopped;
    for (const auto& [pid, comm] : find()) {
        if (::kill(pid, SIGTERM) == 0) {
            stopped << comm;
        }
    }
    for (int i = 0; i < 20 && !find().isEmpty(); i++) {
        QThread::msleep(100);
    }
    for (const auto& [pid, comm] : find()) {
        ::kill(pid, SIGKILL);
    }
    return stopped;
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
