// SPDX-License-Identifier: MIT
#include "chovr.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QStandardPaths>

#include "steamvr.h"

namespace pibridge::app {

namespace {

const QString kDriver = QStringLiteral("CustomHeadsetOpenVR");
const QString kGui = QStringLiteral("custom-headset-gui");

QJsonObject readObject(const QString& path, bool* ok) {
    QFile f(path);
    *ok = false;
    if (!f.open(QIODevice::ReadOnly)) {
        return {};
    }
    const QJsonDocument d = QJsonDocument::fromJson(f.readAll());
    *ok = d.isObject();
    return d.object();
}

bool executable(const QString& p) {
    const QFileInfo fi(p);
    return !p.isEmpty() && fi.isFile() && fi.isExecutable();
}

} // namespace

QString chovrSettingsFile() {
    QString base = qEnvironmentVariable("XDG_CONFIG_HOME");
    if (base.isEmpty()) {
        base = QDir::homePath() + QStringLiteral("/.config");
    }
    return base + QStringLiteral("/CustomHeadset/settings.json");
}

bool chovrDriverRegistered(const QString& vrpathFile) {
    bool ok = false;
    const QJsonObject o = readObject(vrpathFile, &ok);
    if (!ok) {
        return false;
    }
    for (const QJsonValue& v : o.value(QStringLiteral("external_drivers")).toArray()) {
        const QString p = QDir::cleanPath(v.toString());
        if (p == kDriver || p.endsWith(QLatin1Char('/') + kDriver)) {
            return true;
        }
    }
    const QString rt = openVrRuntimeDir(vrpathFile);
    return !rt.isEmpty() && QFileInfo(rt + QStringLiteral("/drivers/") + kDriver).isDir();
}

bool chovrHeadsetEnabled(const QString& settingsFile) {
    bool ok = false;
    const QJsonObject o = readObject(settingsFile, &ok);
    if (!ok) {
        return false;
    }
    bool anyHeadset = false;
    for (auto it = o.begin(); it != o.end(); ++it) {
        if (!it.value().isObject() || it.key() == QLatin1String("customShader")) {
            continue;
        }
        anyHeadset = true;
        const QJsonValue en = it.value().toObject().value(QStringLiteral("enable"));
        if (en.isUndefined() || en.toBool(false)) {
            return true;
        }
    }
    return !anyHeadset; // no headset entry: CHOVR's defaults
}

bool chovrActive(const QString& vrpathFile, const QString& settingsFile) {
    return chovrDriverRegistered(vrpathFile) && chovrHeadsetEnabled(settingsFile);
}

QString chovrGuiPath(const QString& confPath, const QString& home) {
    if (const QString p = QStandardPaths::findExecutable(kGui); !p.isEmpty()) {
        return p;
    }
    const QString candidates[] = {
        home + QStringLiteral("/.local/bin/") + kGui,
        confPath.startsWith(QLatin1String("~/")) ? home + confPath.mid(1) : confPath,
        home + QStringLiteral("/gamedev/ext/CustomHeadsetOpenVR/output/CustomHeadsetGUI/release/") + kGui,
    };
    for (const QString& c : candidates) {
        if (executable(c)) {
            return QFileInfo(c).absoluteFilePath();
        }
    }
    return {};
}

bool startChovrGui(Runner& runner, const QString& path) {
    return !path.isEmpty() && runner.startDetached(path, {});
}

bool chovrSetIpd(const QString& settingsFile, double mm) {
    bool ok = false;
    QJsonObject root = readObject(settingsFile, &ok); // missing or broken: start from an empty object
    QStringList keys;
    for (auto it = root.begin(); it != root.end(); ++it) {
        if (it.key().startsWith(QLatin1String("crystalSuper")) && it.value().isObject()) {
            keys << it.key();
        }
    }
    if (keys.isEmpty()) {
        keys << QStringLiteral("crystalSuperMicroOLED");
    }
    for (const QString& k : keys) {
        QJsonObject h = root.value(k).toObject();
        h[QStringLiteral("ipd")] = mm;
        h[QStringLiteral("hardwareIpd")] = false;
        root[k] = h;
    }
    // the old horizontal IPD offset (no longer set in PiBridge) must not shift the eyes on top of the IPD
    QJsonObject gen = root.value(QStringLiteral("generalHeadset")).toObject();
    gen.remove(QStringLiteral("horizontalIPDOffset"));
    root[QStringLiteral("generalHeadset")] = gen;
    QDir().mkpath(QFileInfo(settingsFile).absolutePath());
    QSaveFile f(settingsFile);
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    return f.commit();
}

} // namespace pibridge::app
