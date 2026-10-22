// SPDX-License-Identifier: MIT
#include "rec_setup.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <algorithm>
#include <unistd.h>

#include "paths.h"
#include "roommap_link.h"
#include "steamvr.h"
#include "steamvr_reset.h"
#include "system.h"

namespace pibridge::app {

namespace {

QString envDir(const char* var, const QString& def) {
    const QString v = qEnvironmentVariable(var);
    return v.isEmpty() ? def : v;
}

QString expand(const QString& p, const QString& home) {
    if (p == QLatin1String("~")) {
        return home;
    }
    return p.startsWith(QLatin1String("~/")) ? home + p.mid(1) : p;
}

QByteArray readFile(const QString& p, qint64 max = 1 << 16) {
    QFile f(p);
    return f.open(QIODevice::ReadOnly) ? f.read(max) : QByteArray();
}

QString readLine(const QString& p) {
    return QString::fromLocal8Bit(readFile(p, 4096)).trimmed();
}

bool isExec(const QString& p) {
    const QFileInfo fi(p);
    return !p.isEmpty() && fi.isFile() && fi.isExecutable();
}

bool isFile(const QString& p) {
    return !p.isEmpty() && QFileInfo(p).isFile();
}

// The conf value if set (then only it counts), else the first candidate that passes `ok`.
RecPath pick(const Conf& conf, const char* key, const QString& label, bool required, const QStringList& candidates,
             const QString& home, bool (*ok)(const QString&)) {
    RecPath r;
    r.key = QString::fromLatin1(key);
    r.label = label;
    r.required = required;
    const QString set = conf.value(key, QString()).trimmed();
    if (!set.isEmpty()) {
        r.fromConf = true;
        const QString p = expand(set, home);
        if (ok(p)) {
            r.path = QFileInfo(p).absoluteFilePath();
        } else {
            r.wanted = p;
        }
        return r;
    }
    for (const QString& c : candidates) {
        if (ok(c)) {
            r.path = QFileInfo(c).absoluteFilePath();
            break;
        }
    }
    return r;
}

QStringList tool(const RecEnv& env, const QString& name, const QStringList& more) {
    QStringList c;
    if (env.which) {
        if (const QString p = env.which(name); !p.isEmpty()) {
            c << p;
        }
    }
    for (const QString& d : more) {
        c << d + QLatin1Char('/') + name;
    }
    return c;
}

} // namespace

RecEnv RecEnv::defaults() {
    RecEnv e;
    e.home = QDir::homePath();
    e.prefix = pibridge::app::installPrefix();
    e.configDir = pibridge::app::configDir();
    e.dataHome = envDir("XDG_DATA_HOME", e.home + QStringLiteral("/.local/share"));
    e.stateHome = envDir("XDG_STATE_HOME", e.home + QStringLiteral("/.local/state"));
    e.runtimeDir = envDir("XDG_RUNTIME_DIR", QStringLiteral("/run/user/%1").arg(::getuid()));
    e.vrpathFile = openVrPathsFile();
    e.which = [](const QString& n) { return QStandardPaths::findExecutable(n); };
    e.buildInfo = [](const QString& p) { return monadoBuildInfoAsync(p); }; // never blocks the UI thread
    e.caps = [](const QString& p) { return fileCaps(p); };
    e.findTool = [](const QString& n) { return pibridge::app::findTool(n); };
    e.roommapDir = roommapBaseDir();
    return e;
}

RecSetup resolveRecSetup(const Conf& conf, const RecEnv& env) {
    const QString& h = env.home;
    // The installed layout (core/paths.h): engine/plugins under <prefix>/lib/pibridge, read-only data under
    // <prefix>/share/pibridge. These come first in every lookup; the ~/gamedev/* dev-tree paths stay last so an
    // uninstalled development checkout still works.
    const QString lib = env.prefix + QStringLiteral("/lib/pibridge");
    const QString data = env.prefix + QStringLiteral("/share/pibridge");
    // PiBridge's own tools: PATH, then the install prefix, then the build tree next to the app (core/system.h)
    const auto ownTool = [&env](const QString& name) {
        QStringList c = tool(env, name, {env.prefix + QStringLiteral("/bin")});
        if (env.findTool) {
            if (const QString p = env.findTool(name); !p.isEmpty()) {
                c << p;
            }
        }
        return c;
    };
    RecSetup s;
    s.monado = pick(conf, reckey::kMonado, QStringLiteral("monado-service"), true,
                    tool(env, QStringLiteral("monado-service"),
                         {lib + QStringLiteral("/monado/bin"),
                          h + QStringLiteral("/gamedev/pibridge-monado-build/src/xrt/targets/service")}),
                    h, isExec);
    s.cam = pick(conf, reckey::kCam, QStringLiteral("pibridge-cam"), true, ownTool(QStringLiteral("pibridge-cam")), h,
                 isExec);
    s.ctrl = pick(conf, reckey::kCtrl, QStringLiteral("pibridge-ctrltrack"), true,
                  ownTool(QStringLiteral("pibridge-ctrltrack")), h, isExec);
    s.basalt = pick(conf, reckey::kBasalt, QStringLiteral("Basalt library (libbasalt.so)"), false,
                    {lib + QStringLiteral("/libbasalt.so"), h + QStringLiteral("/gamedev/ovr-build/basalt/lib/libbasalt.so"),
                     QStringLiteral("/usr/local/lib/libbasalt.so"), QStringLiteral("/usr/lib/libbasalt.so")},
                    h, isFile);
    // the Pimax SLAM engine (not redistributable: copied from Pimax Play at install): the installed plugin, else the
    // user's data dir, else the build in the source tree
    s.winbridge = pick(conf, reckey::kWinbridge, QStringLiteral("winbridge.exe"), false,
                       {lib + QStringLiteral("/winbridge/winbridge.exe"),
                        env.dataHome + QStringLiteral("/pibridge/winbridge/winbridge.exe"),
                        h + QStringLiteral("/gamedev/pibridge/winbridge/winbridge.exe")},
                       h, isFile);
    s.vitBridge = pick(conf, reckey::kVitBridge, QStringLiteral("VIT library (libpibridge_vit_winbridge.so)"), false,
                       {lib + QStringLiteral("/winbridge/libpibridge_vit_winbridge.so"),
                        env.dataHome + QStringLiteral("/pibridge/winbridge/libpibridge_vit_winbridge.so"),
                        h + QStringLiteral("/gamedev/pibridge/winbridge/libpibridge_vit_winbridge.so")},
                       h, isFile);
    s.wine = env.which ? env.which(QStringLiteral("wine")) : QString();
    {
        const QString kit = env.dataHome + QStringLiteral("/pibridge/windata");
        const bool calib = !QDir(kit + QStringLiteral("/slam"))
                                .entryList({QStringLiteral("device_calibration_*.xml")}, QDir::Files)
                                .isEmpty();
        if (isFile(kit + QStringLiteral("/dlls/pimax_slam.pi.dll")) &&
            isFile(kit + QStringLiteral("/slam/voc_GEN_8X4.dbow")) && calib) {
            s.winKit = kit;
            // the controllers' LED models (Pimax Play's ProgramData\Pimax\slam\H1\<controller>\)
            for (const QString& d : QDir(kit + QStringLiteral("/slam/H1")).entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
                if (isFile(kit + QStringLiteral("/slam/H1/") + d + QStringLiteral("/controller_params.json"))) {
                    s.winCtrlModels = true;
                    break;
                }
            }
        }
    }
    s.slamConfig = pick(conf, reckey::kSlamConfig, QStringLiteral("SLAM config (pimax-basalt.toml)"), false,
                        {env.configDir + QStringLiteral("/pimax-basalt.toml"),
                         data + QStringLiteral("/slam/pimax-basalt.toml"),
                         h + QStringLiteral("/gamedev/ext/slam/pimax-basalt.toml")},
                        h, isFile);
    // The lens profile (lens_profile, the Display page's selector):
    //   "native" = crystal-pimax-native.conf (the Pimax geometry, the eye axis set by a true rotation),
    //   else       crystal-chovr.conf        (CHOVR's camera-calibrated default, FOV 98x88 = what CHOVR renders).
    // The chosen profile is tried first, then the other, so a missing file still resolves. An explicit rec_lens_conf
    // path still wins. crystal.conf (the raw PVR dump, inflated 101x97 port) stays last as a legacy fallback.
    const QString lensProfile = conf.value(reckey::kLensProfile, QString()).trimmed();
    QStringList lensNames;
    if (lensProfile == QLatin1String("native")) {
        lensNames = {QStringLiteral("crystal-pimax-native.conf"), QStringLiteral("crystal-chovr.conf")};
    } else {
        lensNames = {QStringLiteral("crystal-chovr.conf"), QStringLiteral("crystal-pimax-native.conf")};
    }
    QStringList lensCandidates{env.configDir + QStringLiteral("/lens.conf")};
    for (const QString& n : lensNames) {
        lensCandidates << data + QStringLiteral("/lens/") + n << h + QStringLiteral("/gamedev/ext/lens/") + n;
    }
    lensCandidates << data + QStringLiteral("/lens/crystal.conf") << h + QStringLiteral("/gamedev/ext/lens/crystal.conf");
    s.lens = pick(conf, reckey::kLens, QStringLiteral("lens file"), false, lensCandidates, h, isFile);
    // the user's factory calibration first, then the installed default template
    s.calib = pick(conf, reckey::kCalib, QStringLiteral("camera calibration"), true,
                   {env.configDir + QStringLiteral("/headset-calibration.xml"),
                    data + QStringLiteral("/calib/crystal-calibration.xml"),
                    h + QStringLiteral("/gamedev/ext/calib/crystal-calibration.xml")},
                   h, isFile);

    const QString conn = conf.value(reckey::kConnector, QString()).trimmed();
    s.connectorFromConf = !conn.isEmpty();
    s.connector = s.connectorFromConf ? conn : pimaxConnector(env.drmRoot);
    const QString cam = conf.value(reckey::kCamDevice, QString()).trimmed();
    s.camDeviceFromConf = !cam.isEmpty();
    s.camDevice = s.camDeviceFromConf ? (QFileInfo::exists(cam) ? cam : QString())
                                      : pimaxCameraDevice(env.v4lRoot, env.devRoot);
    const QString dir = conf.value(reckey::kDir, QString()).trimmed();
    s.recordingsDir = dir.isEmpty() ? env.dataHome + QStringLiteral("/pibridge/recordings") : expand(dir, h);
    const QString steps = conf.value(reckey::kSteps, QString()).trimmed();
    s.stepsFile = steps.isEmpty() ? QString() : expand(steps, h);
    s.brokerSocket = env.runtimeDir + QStringLiteral("/omarchyvr-cam-broker.sock");
    for (const QString& w : conf.value(reckey::kEnv, QString()).split(QLatin1Char(' '), Qt::SkipEmptyParts)) {
        if (w.indexOf(QLatin1Char('=')) > 0) {
            s.extraEnv << w;
        }
    }
    s.headsetPresent = pimaxHeadsetPresent(env.usbRoot);
    s.io = ioSettings(conf);
    s.refreshHz = conf.refreshHz();
    {
        bool ok = false;
        const double ipd = conf.value(reckey::kIpdMm, QString()).trimmed().toDouble(&ok);
        s.ipdMm = ok && ipd >= 50.0 && ipd <= 80.0 ? ipd : reckey::kIpdDefaultMm;
    }
    s.eyeTracking = conf.eyeTracking();
    s.eyeCalib = env.configDir + QStringLiteral("/eye-calibration.toml");
    s.sessionsDir = env.stateHome + QStringLiteral("/pibridge/inside-out");
    s.roommap = env.findTool ? env.findTool(QStringLiteral("pibridge-roommap")) : QString();
    s.slamTool = env.findTool ? env.findTool(QStringLiteral("pibridge-slam")) : QString();
    s.roomName = roomName(conf);
    s.roomSaved = roomSaved(env.roommapDir, s.roomName);
    if (!s.monado.path.isEmpty()) {
        if (env.buildInfo) {
            s.build = env.buildInfo(s.monado.path);
        }
        if (env.caps) {
            s.caps = env.caps(s.monado.path);
        }
    }
    s.steamVrDriverDir = steamVrDriverDirFor(s.monado.path);
    s.steamVrPreload = rttimePreloadFor(s.monado.path);
    s.driverEnvFile = env.configDir + QStringLiteral("/steamvr-driver.env");
    if (!env.vrpathFile.isEmpty()) {
        if (const QString rt = openVrRuntimeDir(env.vrpathFile); !rt.isEmpty() &&
            QFileInfo(rt + QStringLiteral("/bin/vrstartup.sh")).isFile()) {
            s.steamVrStartup = rt + QStringLiteral("/bin/vrstartup.sh");
        }
    }
    return s;
}

RecSetup fakeRecSetup(const QString& recordingsDir) {
    const auto p = [](const char* key, const char* label, const char* path, bool required) {
        RecPath r;
        r.key = QString::fromLatin1(key);
        r.label = QString::fromLatin1(label);
        r.path = QString::fromLatin1(path);
        r.required = required;
        return r;
    };
    RecSetup s;
    s.monado = p(reckey::kMonado, "monado-service", "/fake/monado/monado-service", true);
    s.cam = p(reckey::kCam, "pibridge-cam", "/fake/bin/pibridge-cam", true);
    s.ctrl = p(reckey::kCtrl, "pibridge-ctrltrack", "/fake/bin/pibridge-ctrltrack", true);
    s.basalt = p(reckey::kBasalt, "Basalt library (libbasalt.so)", "/fake/basalt/libbasalt.so", false);
    s.winbridge = p(reckey::kWinbridge, "winbridge.exe", "/fake/pibridge/winbridge/winbridge.exe", false);
    s.vitBridge = p(reckey::kVitBridge, "VIT library (libpibridge_vit_winbridge.so)",
                    "/fake/pibridge/winbridge/libpibridge_vit_winbridge.so", false);
    s.wine = QStringLiteral("/fake/bin/wine");
    s.winKit = QStringLiteral("/fake/share/pibridge/windata");
    s.winCtrlModels = true;
    s.slamConfig = p(reckey::kSlamConfig, "SLAM config (pimax-basalt.toml)", "", false);
    s.lens = p(reckey::kLens, "lens file", "/fake/lens/crystal.conf", false);
    s.calib = p(reckey::kCalib, "camera calibration", "/fake/config/pibridge/headset-calibration.xml", true);
    s.connector = QStringLiteral("DP-6");
    s.camDevice = QStringLiteral("/dev/video0");
    s.recordingsDir = recordingsDir;
    s.brokerSocket = QStringLiteral("/fake/run/omarchyvr-cam-broker.sock");
    s.headsetPresent = true;
    s.sessionsDir = QFileInfo(recordingsDir).absolutePath() + QStringLiteral("/inside-out");
    s.build.checked = true; // a made-up PiBridge-linked build with every marker
    s.build.path = s.monado.path;
    s.build.kind = BuildKind::PiBridge;
    s.build.abiText = QStringLiteral("PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 0000000 (fake)");
    s.caps = CapState{true, false, false};
    s.steamVrStartup = QStringLiteral("/fake/SteamVR/bin/vrstartup.sh");
    s.steamVrDriverDir = QStringLiteral("/fake/monado/steamvr-monado");
    s.steamVrPreload = s.steamVrDriverDir + QStringLiteral("/bin/linux64/pimax_rttime_preload.so");
    s.driverEnvFile = QFileInfo(recordingsDir).absolutePath() + QStringLiteral("/steamvr-driver.env");
    s.roommap = QStringLiteral("/fake/bin/pibridge-roommap");
    s.slamTool = QStringLiteral("/fake/bin/pibridge-slam");
    s.roomName = QString::fromLatin1(kDefaultRoom);
    return s;
}

// ---------------------------------------------------------------- detection

QString edidVendor(const QByteArray& e) {
    if (e.size() < 10) {
        return {};
    }
    const unsigned v = (static_cast<unsigned char>(e[8]) << 8) | static_cast<unsigned char>(e[9]);
    QString s;
    for (int shift : {10, 5, 0}) {
        const unsigned c = (v >> shift) & 0x1f;
        if (c < 1 || c > 26) {
            return {};
        }
        s += QChar(char('A' + c - 1));
    }
    return s;
}

QString edidName(const QByteArray& e) {
    for (int off : {54, 72, 90, 108}) {
        if (e.size() < off + 18) {
            break;
        }
        if (e[off] == 0 && e[off + 1] == 0 && e[off + 2] == 0 && static_cast<unsigned char>(e[off + 3]) == 0xfc) {
            QByteArray t = e.mid(off + 5, 13);
            if (const qsizetype nl = t.indexOf('\n'); nl >= 0) {
                t.truncate(nl);
            }
            return QString::fromLatin1(t).trimmed();
        }
    }
    return {};
}

QString pimaxConnector(const QString& drmRoot) {
    const QDir d(drmRoot);
    QString best;
    bool bestConnected = false;
    QStringList entries = d.entryList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::System, QDir::Name);
    for (const QString& e : entries) {
        // card1-DP-6 -> DP-6 (render nodes and plain cardN have no '-')
        const qsizetype dash = e.indexOf(QLatin1Char('-'));
        if (!e.startsWith(QLatin1String("card")) || dash < 0) {
            continue;
        }
        const QByteArray edid = readFile(drmRoot + QLatin1Char('/') + e + QStringLiteral("/edid"));
        if (edid.isEmpty()) {
            continue;
        }
        const bool pimax = edidName(edid).contains(QLatin1String("pimax"), Qt::CaseInsensitive) ||
                           edidVendor(edid) == QLatin1String("PVR");
        if (!pimax) {
            continue;
        }
        const bool connected = readLine(drmRoot + QLatin1Char('/') + e + QStringLiteral("/status")) ==
                               QLatin1String("connected");
        if (best.isEmpty() || (connected && !bestConnected)) {
            best = e.mid(dash + 1);
            bestConnected = connected;
        }
    }
    return best;
}

QString pimaxCameraDevice(const QString& v4lRoot, const QString& devRoot) {
    const QDir d(v4lRoot);
    QStringList nodes = d.entryList({QStringLiteral("video*")}, QDir::Dirs | QDir::NoDotAndDotDot | QDir::System);
    std::sort(nodes.begin(), nodes.end(), [](const QString& a, const QString& b) {
        return a.mid(5).toInt() < b.mid(5).toInt();
    });
    for (const QString& n : nodes) {
        const QString base = v4lRoot + QLatin1Char('/') + n;
        if (!readLine(base + QStringLiteral("/name")).startsWith(QLatin1String("PimaxSensor"))) {
            continue;
        }
        const QString idx = readLine(base + QStringLiteral("/index"));
        if (!idx.isEmpty() && idx != QLatin1String("0")) {
            continue; // the metadata node of the same camera
        }
        return devRoot + QLatin1Char('/') + n;
    }
    return {};
}

bool pimaxHeadsetPresent(const QString& usbRoot) {
    const QDir d(usbRoot);
    for (const QString& e : d.entryList(QDir::Dirs | QDir::NoDotAndDotDot | QDir::System)) {
        const QString base = usbRoot + QLatin1Char('/') + e;
        if (readLine(base + QStringLiteral("/idVendor")) == QLatin1String("34a4") &&
            readLine(base + QStringLiteral("/idProduct")) != QLatin1String("0014")) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------- can it start?

RecSystem readRecSystem(const QString& procRoot) {
    RecSystem s;
    s.vrserver = processRunning(QStringLiteral("vrserver"), procRoot);
    s.monado = processRunning(QStringLiteral("monado-service"), procRoot);
    s.camBroker = processRunning(QStringLiteral("pibridge-cam"), procRoot);
    s.ctrltrack = processRunning(QStringLiteral("pibridge-ctrltrack"), procRoot);
    s.winbridge = processRunning(QStringLiteral("winbridge.exe"), procRoot);
    s.steamClient = steamClientRunning(procRoot);
    return s;
}

RecReadiness recReadiness(const RecSetup& s, const RecSystem& sys, RecUse use) {
    RecReadiness r;
    if (sys.vrserver && use == RecUse::Monado) {
        r.blocking << QStringLiteral("SteamVR is running. Close SteamVR first (it holds the headset).");
    }
    if (sys.monado) {
        r.blocking << QStringLiteral("Another Monado (monado-service) is running. Close it first.");
    }
    if (sys.camBroker) {
        r.blocking << QStringLiteral("The tracking cameras are in use by another pibridge-cam. Close it first.");
    }
    if (sys.ctrltrack) {
        r.blocking << QStringLiteral("Another pibridge-ctrltrack is running. Close it first.");
    }
    if (sys.winbridge && s.io.slam && s.io.slamEngine == SlamEngine::Pimax) {
        r.blocking << QStringLiteral("Another winbridge (Pimax head tracking) is running. Close it first.");
    }
    if (!s.headsetPresent) {
        r.blocking << QStringLiteral("No headset found. Connect the headset and switch it on.");
    }
    if (s.camDevice.isEmpty()) {
        r.blocking << (s.camDeviceFromConf
                           ? QStringLiteral("The camera device in rec_cam_device= (pibridge.conf) does not exist.")
                           : QStringLiteral("The headset's tracking cameras were not found (no PimaxSensor video "
                                            "device). Set rec_cam_device= in pibridge.conf if it has another name."));
    }
    if (s.connector.isEmpty() && use == RecUse::Monado) {
        r.blocking << QStringLiteral("The headset's display connector was not found (no Pimax screen under "
                                     "/sys/class/drm). Set rec_connector= in pibridge.conf (for example DP-6).");
    }
    for (const RecPath* p : s.paths()) {
        if (!p->path.isEmpty()) {
            continue;
        }
        const QString what = p->fromConf ? QStringLiteral("%1 not found at %2 (%3= in pibridge.conf).")
                                               .arg(p->label, p->wanted, p->key)
                                         : QStringLiteral("%1 not found. Set %2= in pibridge.conf.").arg(p->label, p->key);
        if (p == &s.winbridge || p == &s.vitBridge) {
            // only the Pimax engine needs them
            if (s.io.slam && s.io.slamEngine == SlamEngine::Pimax) {
                r.warnings << what + QStringLiteral(" Pimax head tracking stays off (3DoF).");
            }
        } else if (p->required) {
            r.blocking << what;
        } else if (p == &s.basalt) {
            r.warnings << what + QStringLiteral(" The live 6DoF tracker stays off; the recording still works.");
        } else if (p == &s.slamConfig) {
            r.warnings << what + QStringLiteral(" Monado uses its built-in SLAM settings.");
        } else {
            r.warnings << what + QStringLiteral(" The headset uses its default lens settings.");
        }
    }
    if (const QString b = buildBlockingReason(s.build); !b.isEmpty() && !s.monado.path.isEmpty()) {
        r.blocking << b;
    }
    if (use == RecUse::Monado && s.caps && s.caps->readable && !s.caps->ok() && !s.monado.path.isEmpty()) {
        r.warnings << QStringLiteral("monado-service may not use realtime priority (cap_sys_nice is missing): press "
                                     "Fix permissions under Monado build on the Development page.");
    }
    return r;
}

} // namespace pibridge::app
