// SPDX-License-Identifier: MIT
#include "openxr_runtime.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace pibridge::app {

QString activeRuntimePath() {
    QString base = qEnvironmentVariable("XDG_CONFIG_HOME");
    if (base.isEmpty()) {
        base = QDir::homePath() + QStringLiteral("/.config");
    }
    return base + QStringLiteral("/openxr/1/active_runtime.json");
}

OpenXrRuntime activeRuntimeOf(const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        return OpenXrRuntime::None;
    }
    const QJsonObject rt = QJsonDocument::fromJson(f.readAll()).object().value(QStringLiteral("runtime")).toObject();
    if (rt.isEmpty()) {
        return OpenXrRuntime::Other;
    }
    const QString name = rt.value(QStringLiteral("name")).toString();
    if (rt.value(QStringLiteral("VALVE_runtime_is_steamvr")).toBool() || name == QLatin1String("SteamVR")) {
        return OpenXrRuntime::SteamVr;
    }
    if (name.contains(QLatin1String("Monado"), Qt::CaseInsensitive) ||
        rt.value(QStringLiteral("library_path")).toString().contains(QLatin1String("openxr_monado"))) {
        return OpenXrRuntime::Monado;
    }
    return OpenXrRuntime::Other;
}

QString steamVrRuntimeJsonFor(const QString& steamVrStartup) {
    if (steamVrStartup.isEmpty()) {
        return {};
    }
    QDir d = QFileInfo(steamVrStartup).dir(); // <runtime>/bin
    d.cdUp();
    const QString p = d.absoluteFilePath(QStringLiteral("steamxr_linux64.json"));
    return QFileInfo::exists(p) ? p : QString();
}

std::expected<void, QString> setActiveRuntime(const QString& path, const QString& target) {
    if (target.isEmpty() || !QFileInfo::exists(target)) {
        return std::unexpected(QStringLiteral("The runtime file %1 does not exist.").arg(target));
    }
    const QFileInfo cur(path);
    QDir().mkpath(cur.absolutePath());
    if (cur.exists() && !cur.isSymLink()) {
        const QString backup = path + QStringLiteral(".pibridge-backup");
        if (!QFileInfo::exists(backup) && !QFile::copy(path, backup)) {
            return std::unexpected(QStringLiteral("Could not keep a copy of %1.").arg(path));
        }
    }
    const QByteArray tmp = QFile::encodeName(path + QStringLiteral(".pibridge-tmp"));
    ::unlink(tmp.constData());
    if (::symlink(QFile::encodeName(target).constData(), tmp.constData()) != 0) {
        return std::unexpected(QStringLiteral("Could not create the link: %1.").arg(QString::fromLocal8Bit(std::strerror(errno))));
    }
    if (::rename(tmp.constData(), QFile::encodeName(path).constData()) != 0) {
        const int e = errno;
        ::unlink(tmp.constData());
        return std::unexpected(QStringLiteral("Could not replace %1: %2.").arg(path, QString::fromLocal8Bit(std::strerror(e))));
    }
    return {};
}

QString openXrRuntimeWord(OpenXrRuntime r) {
    switch (r) {
    case OpenXrRuntime::SteamVr: return QStringLiteral("SteamVR");
    case OpenXrRuntime::Monado: return QStringLiteral("Monado");
    case OpenXrRuntime::Other: return QStringLiteral("other");
    case OpenXrRuntime::None: break;
    }
    return QStringLiteral("none");
}

} // namespace pibridge::app
