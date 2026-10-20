// SPDX-License-Identifier: MIT
// Tests of core/chovr.*: is CustomHeadsetOpenVR the SteamVR driver (temp vrpath and settings files), where its
// configurator is (temp PATH and home), and that it is started detached through the FakeRunner.
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include "core/chovr.h"
#include "core/runner.h"
#include "pibridge_test.h"

using namespace pibridge::app;

namespace {

void writeAll(const QString& p, const QByteArray& b, bool exec = false) {
    QDir().mkpath(QFileInfo(p).absolutePath());
    QFile f(p);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        f.write(b);
    }
    f.close();
    if (exec) {
        f.setPermissions(f.permissions() | QFileDevice::ExeOwner | QFileDevice::ReadOwner);
    }
}

} // namespace

TEST(driver_registered_by_external_drivers) {
    QTemporaryDir t;
    const QString vp = t.path() + QStringLiteral("/openvrpaths.vrpath");
    CHECK(!chovrDriverRegistered(vp)); // missing
    writeAll(vp, "not json");
    CHECK(!chovrDriverRegistered(vp));
    writeAll(vp, R"({"runtime":["/nowhere/SteamVR"],"external_drivers":["/x/CustomHeadsetOpenVR/build/CustomHeadsetOpenVR/"]})");
    CHECK(chovrDriverRegistered(vp));
    writeAll(vp, R"({"runtime":["/nowhere/SteamVR"],"external_drivers":["/x/CustomHeadsetOpenVR/build/other", "/x/CustomHeadsetOpenVR2"]})");
    CHECK(!chovrDriverRegistered(vp));
    // or the driver dir in the runtime's drivers/
    const QString rt = t.path() + QStringLiteral("/SteamVR");
    QDir().mkpath(rt + QStringLiteral("/drivers/CustomHeadsetOpenVR"));
    writeAll(vp, QStringLiteral(R"({"runtime":["%1"]})").arg(rt).toUtf8());
    CHECK(chovrDriverRegistered(vp));
}

TEST(headset_enabled_rules) {
    QTemporaryDir t;
    const QString s = t.path() + QStringLiteral("/settings.json");
    CHECK(!chovrHeadsetEnabled(s)); // missing
    writeAll(s, "{");
    CHECK(!chovrHeadsetEnabled(s)); // broken
    writeAll(s, "{}");
    CHECK(chovrHeadsetEnabled(s)); // the defaults
    writeAll(s, R"({"crystalSuperMicroOLED":{"distortionProfile":"x"}})");
    CHECK(chovrHeadsetEnabled(s)); // no "enable": the default (on)
    writeAll(s, R"({"crystalSuperMicroOLED":{"enable":false},"dreamAir":{"enable":false},"customShader":{}})");
    CHECK(!chovrHeadsetEnabled(s)); // customShader is no headset
    writeAll(s, R"({"crystalSuperMicroOLED":{"enable":false},"dreamAir":{"enable":true}})");
    CHECK(chovrHeadsetEnabled(s));

    const QString vp = t.path() + QStringLiteral("/openvrpaths.vrpath");
    writeAll(vp, R"({"external_drivers":["/x/CustomHeadsetOpenVR"]})");
    CHECK(chovrActive(vp, s));
    CHECK(!chovrActive(vp, t.path() + QStringLiteral("/none.json")));
    CHECK(!chovrActive(t.path() + QStringLiteral("/none.vrpath"), s));
}

TEST(gui_lookup_order_and_start) {
    QTemporaryDir t;
    const QString home = t.path() + QStringLiteral("/home");
    const QString bin = t.path() + QStringLiteral("/bin");
    QDir().mkpath(bin);
    const QByteArray oldPath = qgetenv("PATH");
    qputenv("PATH", bin.toLocal8Bit());
    const QString build = home + QStringLiteral("/gamedev/ext/CustomHeadsetOpenVR/output/CustomHeadsetGUI/release/custom-headset-gui");
    const QString conf = t.path() + QStringLiteral("/opt/chovr-gui");
    const QString local = home + QStringLiteral("/.local/bin/custom-headset-gui");
    const QString inPath = bin + QStringLiteral("/custom-headset-gui");

    CHECK(chovrGuiPath({}, home).isEmpty());
    writeAll(build, "#!/bin/sh\n"); // not executable: does not count
    CHECK(chovrGuiPath({}, home).isEmpty());
    writeAll(build, "#!/bin/sh\n", true);
    CHECK(chovrGuiPath({}, home) == build);
    writeAll(conf, "#!/bin/sh\n", true);
    CHECK(chovrGuiPath(conf, home) == conf);
    writeAll(local, "#!/bin/sh\n", true);
    CHECK(chovrGuiPath(conf, home) == local);
    writeAll(inPath, "#!/bin/sh\n", true);
    CHECK(chovrGuiPath(conf, home) == inPath);
    qputenv("PATH", oldPath);

    FakeRunner r;
    CHECK(!startChovrGui(r, {}));
    CHECK(r.calls.empty());
    CHECK(startChovrGui(r, build));
    CHECK(r.calls.size() == 1 && r.calls[0].program == build && r.calls[0].args.isEmpty() && r.calls[0].detached);
}

PIBRIDGE_TEST_MAIN()
