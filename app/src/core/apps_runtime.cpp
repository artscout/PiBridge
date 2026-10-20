// SPDX-License-Identifier: MIT
#include "apps_runtime.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>

namespace pibridge::app {

QString monadoSocketPath(const QString& runtimeDir) {
    return runtimeDir + QLatin1Char('/') + QLatin1String(kMonadoSocketName);
}

namespace {

// The folder of monado-service and up to four above it (a build tree), nearest first.
QStringList buildDirsOf(const QString& monadoService) {
    QStringList out;
    QDir d = QFileInfo(monadoService).absoluteDir();
    for (int i = 0; i <= 4; i++) {
        out << d.absolutePath();
        if (!d.cdUp()) {
            break;
        }
    }
    return out;
}

// <prefix> of an installed <prefix>/bin/monado-service; "" otherwise.
QString prefixOf(const QString& monadoService) {
    const QDir bin = QFileInfo(monadoService).absoluteDir();
    if (bin.dirName() != QLatin1String("bin")) {
        return {};
    }
    QDir p = bin;
    return p.cdUp() ? p.absolutePath() : QString();
}

} // namespace

QString monadoRuntimeJsonFor(const QString& monadoService) {
    if (monadoService.isEmpty()) {
        return {};
    }
    for (const QString& d : buildDirsOf(monadoService)) {
        const QString f = d + QStringLiteral("/openxr_monado-dev.json");
        if (QFileInfo(f).isFile()) {
            return f;
        }
    }
    if (const QString p = prefixOf(monadoService); !p.isEmpty()) {
        const QString f = p + QStringLiteral("/share/openxr/1/openxr_monado.json");
        if (QFileInfo(f).isFile()) {
            return f;
        }
    }
    return {};
}

QString libmonadoFor(const QString& monadoService) {
    if (monadoService.isEmpty()) {
        return {};
    }
    if (const QString json = monadoRuntimeJsonFor(monadoService); !json.isEmpty()) {
        QFile f(json);
        if (f.open(QIODevice::ReadOnly)) {
            const QJsonObject rt = QJsonDocument::fromJson(f.readAll()).object().value(QStringLiteral("runtime")).toObject();
            QString lib = rt.value(QStringLiteral("MND_libmonado_path")).toString();
            if (!lib.isEmpty() && QFileInfo(lib).isRelative()) {
                lib = QFileInfo(json).absoluteDir().absoluteFilePath(lib);
            }
            if (!lib.isEmpty() && QFileInfo(lib).isFile()) {
                return lib;
            }
        }
    }
    for (const QString& d : buildDirsOf(monadoService)) {
        const QString f = d + QStringLiteral("/src/xrt/targets/libmonado/libmonado.so");
        if (QFileInfo(f).isFile()) {
            return f;
        }
    }
    if (const QString p = prefixOf(monadoService); !p.isEmpty()) {
        for (const char* sub : {"/lib/libmonado.so", "/lib64/libmonado.so"}) {
            const QString f = p + QLatin1String(sub);
            if (QFileInfo(f).isFile()) {
                return f;
            }
        }
    }
    return {};
}

MonadoReadiness evaluateMonadoReady(const MonadoInputs& in) {
    MonadoReadiness r;
    r.runtimeJson = in.runtimeJson;
    if (in.runtimeJson.isEmpty()) {
        r.step = MonadoStep::NoRuntimeFile;
        r.words = QStringLiteral("The Monado build has no openxr_monado-dev.json next to monado-service, so an app "
                                 "cannot find Monado.");
        return r;
    }
    switch (in.state) {
    case RecState::Idle:
        r.step = MonadoStep::Off;
        r.words = QStringLiteral("Tracking is off. Play starts it.");
        return r;
    case RecState::Starting:
    case RecState::RetryWait:
        r.step = MonadoStep::Starting;
        r.words = QStringLiteral("Starting tracking...");
        return r;
    case RecState::Stopping:
    case RecState::Finished:
    case RecState::Failed:
        r.step = MonadoStep::Failed;
        r.words = in.message.isEmpty() ? QStringLiteral("Tracking stopped.")
                                       : QStringLiteral("Tracking stopped: %1").arg(in.message);
        return r;
    case RecState::Recording: break;
    }
    if (!in.socket) {
        r.step = MonadoStep::NoSocket;
        r.words = QStringLiteral("Waiting for Monado...");
        return r;
    }
    if (!in.homeSettled) {
        r.step = MonadoStep::HomeStarting;
        r.words = QStringLiteral("Waiting for the home scene...");
        return r;
    }
    r.step = MonadoStep::Ready;
    r.words = QStringLiteral("Tracking runs (Monado).");
    return r;
}

QString steamBlockedWhy(AppTarget t) {
    if (t == AppTarget::SteamVr) {
        return {};
    }
    return QStringLiteral("Steam games run through SteamVR; stop \"Monado directly\" on the Development page first.");
}

} // namespace pibridge::app
