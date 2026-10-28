// SPDX-License-Identifier: MIT
#include "vr_backend.h"

#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>

namespace pibridge::app {

namespace {

void copyTree(const QString& from, const QString& to) {
    QDir().mkpath(to);
    QDirIterator it(from, QDir::AllEntries | QDir::NoDotAndDotDot | QDir::Hidden, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString src = it.next();
        const QString dst = to + src.mid(from.size());
        if (it.fileInfo().isDir()) {
            QDir().mkpath(dst);
        } else {
            QFile::copy(src, dst);
        }
    }
}

} // namespace

VrBackend vrBackend(const Conf& conf) {
    return conf.value(kVrBackendKey, QString()).trimmed() == QLatin1String("monado") ? VrBackend::Monado : VrBackend::SteamVr;
}

QString vrBackendName(VrBackend b) {
    return b == VrBackend::Monado ? QStringLiteral("monado") : QStringLiteral("steamvr");
}

QString vrBackendWord(VrBackend b) {
    return b == VrBackend::Monado ? QStringLiteral("Monado") : QStringLiteral("SteamVR");
}

void exportVrBackend(VrBackend b) {
    qputenv("PIBRIDGE_BACKEND", vrBackendName(b).toUtf8());
}

VrBackend currentVrBackend() {
    return qEnvironmentVariable("PIBRIDGE_BACKEND") == QLatin1String("monado") ? VrBackend::Monado : VrBackend::SteamVr;
}

QStringList vrBackendArgs(VrBackend b) {
    return {QStringLiteral("--backend"), vrBackendName(b)};
}

QString vrBackendRuntimeDir(VrBackend b) {
    const QString rt = qEnvironmentVariable("XDG_RUNTIME_DIR", QStringLiteral("/tmp"));
    return rt + QStringLiteral("/pibridge/") + vrBackendName(b);
}

QString vrBackendRoomsDir(VrBackend b, const QString& home) {
    const QString dir = home + QStringLiteral("/.local/share/pibridge/rooms/") + vrBackendName(b);
    const QString old = home + QStringLiteral("/.local/share/pibridge/roommap");
    if (b == VrBackend::SteamVr && !QFileInfo(dir).isDir() && QFileInfo(old).isDir()) {
        copyTree(old, dir);
    }
    return dir;
}

} // namespace pibridge::app
