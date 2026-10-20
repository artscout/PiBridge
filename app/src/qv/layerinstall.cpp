// SPDX-License-Identifier: MIT
// Copied from artscout-qv-vulkan-linux gui/src/layerinstall.cpp (MIT).
#include "layerinstall.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QTextStream>

#include "settings.h"

namespace qvgui {

namespace {

const QString kName = QStringLiteral("XR_APILAYER_ARTSCOUT_quad_views_vulkan");
const QString kSub = QStringLiteral("openxr/1/api_layers/implicit.d");

QString userDataDir() {
    const QByteArray x = qgetenv("XDG_DATA_HOME");
    return x.isEmpty() ? QDir::homePath() + "/.local/share" : QString::fromLocal8Bit(x);
}

} // namespace

Install findInstall() {
    Install i;
    const QString dir = userDataDir() + "/" + kSub;
    const QString on = dir + "/" + kName + ".json", off = on + ".disabled";
    if (QFileInfo::exists(on)) {
        i.userManifest = on;
        i.userInstalled = i.userEnabled = true;
    } else if (QFileInfo::exists(off)) {
        i.userManifest = off;
        i.userInstalled = true;
    } else {
        i.userManifest = on;
    }
    // The other places the OpenXR loader looks (XDG_DATA_DIRS and XDG_CONFIG_DIRS, /etc).
    QStringList dirs;
    const QByteArray dataDirs = qgetenv("XDG_DATA_DIRS");
    for (const auto& d : QString::fromLocal8Bit(dataDirs.isEmpty() ? "/usr/local/share:/usr/share" : dataDirs)
                             .split(':', Qt::SkipEmptyParts)) {
        dirs << d + "/" + kSub;
    }
    const QByteArray cfgDirs = qgetenv("XDG_CONFIG_DIRS");
    for (const auto& d :
         QString::fromLocal8Bit(cfgDirs.isEmpty() ? "/etc/xdg" : cfgDirs).split(':', Qt::SkipEmptyParts)) {
        dirs << d + "/" + kSub;
    }
    dirs << "/etc/" + kSub;
    for (const auto& d : dirs) {
        const QString f = d + "/" + kName + ".json";
        if (QFileInfo::exists(f)) {
            i.systemManifest = f;
            break;
        }
    }
    return i;
}

bool setEnabled(const Install& i, bool on, QString& error) {
    if (!i.userInstalled) {
        error = QObject::tr("The layer is not installed in your home directory (cmake --install).");
        return false;
    }
    const QString base = i.userManifest.endsWith(".disabled") ? i.userManifest.chopped(9) : i.userManifest;
    const QString from = on ? base + ".disabled" : base, to = on ? base : base + ".disabled";
    if (from == to || !QFileInfo::exists(from)) {
        return true;
    }
    QFile::remove(to);
    if (!QFile::rename(from, to)) {
        error = QObject::tr("Could not rename %1").arg(from);
        return false;
    }
    return true;
}

Status readStatus() {
    Status s;
    const QString path = QString::fromStdString((qv::stateDir() / "status.txt").string());
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return s;
    }
    QTextStream in(&f);
    while (!in.atEnd()) {
        const QString l = in.readLine();
        const qsizetype eq = l.indexOf('=');
        if (eq > 0) {
            s.values.insert(l.left(eq), l.mid(eq + 1));
        }
    }
    s.written = QFileInfo(path).lastModified();
    return s;
}

QString logPath() {
    return QString::fromStdString((qv::stateDir() / "qv-vulkan.log").string());
}

} // namespace qvgui
