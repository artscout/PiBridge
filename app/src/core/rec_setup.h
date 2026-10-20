// SPDX-License-Identifier: MIT
// "Record tracking" (Development page): what a recording needs and whether it can start.
//
// A recording runs, in order, the camera broker (`pimax-cam broker`), the controller tracker (`pimax-ctrltrack live`)
// and `monado-service` with the Pimax driver's 6DoF settings plus PIMAX_SLAM_RECORD=<folder>, which makes Monado
// write a EuRoC dataset (cameras + IMU) into <folder>/euroc_<datetime>/mav0 (core/rec_session.h). This file finds the
// programs and files for that (pibridge.conf keys first, then the usual places), the headset's display connector (the
// DRM connector whose EDID names Pimax), the tracking camera (/dev/videoN of "PimaxSensor"), and turns the system's
// state into plain-words reasons why the RECORD TRACKING button is disabled.
//
// Every lookup takes its roots (home, /sys trees, the PATH search) from RecEnv, so the tests use fake trees.
#pragma once

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <functional>
#include <optional>
#include <vector>

#include "conf.h"
#include "inside_out.h"
#include "monado_build.h"

namespace pibridge::app {

// The pibridge.conf keys of the recording (docs/DEVELOPMENT.md).
namespace reckey {
inline constexpr char kMonado[] = "rec_monado_service";
inline constexpr char kCam[] = "rec_pimax_cam";
inline constexpr char kCtrl[] = "rec_pimax_ctrltrack";
inline constexpr char kBasalt[] = "rec_basalt_lib";
inline constexpr char kWinbridge[] = "rec_winbridge";        // winbridge.exe (the Pimax SLAM engine under Wine)
inline constexpr char kVitBridge[] = "rec_vit_winbridge";    // libpibridge_vit_winbridge.so (Monado's VIT library for it)
inline constexpr char kSlamConfig[] = "rec_slam_config";
inline constexpr char kLens[] = "rec_lens_conf";
inline constexpr char kCalib[] = "rec_cam_calib";
inline constexpr char kConnector[] = "rec_connector";
inline constexpr char kCamDevice[] = "rec_cam_device";
inline constexpr char kDir[] = "rec_dir";
inline constexpr char kSteps[] = "rec_steps";
inline constexpr char kEnv[] = "rec_env";
} // namespace reckey

// Where the lookups look. defaults() = the real system.
struct RecEnv {
    QString home;        // ~
    QString configDir;   // $XDG_CONFIG_HOME/pibridge
    QString dataHome;    // $XDG_DATA_HOME (~/.local/share)
    QString stateHome;   // $XDG_STATE_HOME (~/.local/state)
    QString runtimeDir;  // $XDG_RUNTIME_DIR (/run/user/<uid>)
    QString drmRoot = QStringLiteral("/sys/class/drm");
    QString v4lRoot = QStringLiteral("/sys/class/video4linux");
    QString usbRoot = QStringLiteral("/sys/bus/usb/devices");
    QString devRoot = QStringLiteral("/dev");
    QString vrpathFile;  // ~/.config/openvr/openvrpaths.vrpath (SteamVR's runtime dir; the SteamVR + Monado start)
    // A program in PATH ("" when not there).
    std::function<QString(const QString&)> which;
    // What the chosen monado-service is (core/monado_build.h) and its cap_sys_nice; empty = not looked at (tests).
    std::function<MonadoBuildInfo(const QString&)> buildInfo;
    std::function<CapState(const QString&)> caps;
    // pibridge-roommap (core/system.h findTool) and the room-map dir (core/roommap_link.h); empty = none (tests)
    std::function<QString(const QString&)> findTool;
    QString roommapDir;
    static RecEnv defaults();
};

// One program or file the recording uses.
struct RecPath {
    QString key;      // the pibridge.conf key
    QString label;    // "monado-service"
    QString path;     // resolved, "" when not found
    bool required = true;
    bool fromConf = false;   // the conf names it (then only that path counts)
    QString wanted;          // the conf's value when it names a missing file
};

struct RecSetup {
    RecPath monado, cam, ctrl, basalt, slamConfig, lens, calib;
    // The Pimax SLAM engine (io_slam_engine=pimax, docs/WINBRIDGE.md): the server, Monado's VIT library for it, Wine,
    // and the kit copied from Pimax Play ($XDG_DATA_HOME/pibridge/windata: dlls/ + slam/ with a device_calibration_*.xml).
    RecPath winbridge, vitBridge;
    QString wine;                 // wine in PATH; "" when not found
    QString winKit;               // the kit folder; "" when it lacks the engine, its vocabulary or a calibration
    bool winCtrlModels = false;   // the kit has the controllers' LED models (slam/H1/*/controller_params.json)
    QString connector;            // "DP-6"; "" when not found
    bool connectorFromConf = false;
    QString camDevice;            // "/dev/video0"; "" when not found
    bool camDeviceFromConf = false;
    QString recordingsDir;        // rec_dir, else $XDG_DATA_HOME/pibridge/recordings
    QString stepsFile;            // rec_steps (a protocol JSON), "" = the built-in protocol
    QString brokerSocket;         // $XDG_RUNTIME_DIR/omarchyvr-cam-broker.sock (the driver's default broker socket)
    QStringList extraEnv;         // rec_env: KEY=value words added to monado-service's environment
    bool headsetPresent = false;  // a Pimax USB device (vendor 34a4) other than the camera
    IoSettings io;                // the inside-out settings (core/inside_out.h): the Pimax driver's environment
    QString roommap;              // pibridge-roommap (tools/roommap; "" when not found): the room map at the start
    QString roomName;             // io_room, default "home" (core/roommap_link.h)
    bool roomSaved = false;       // that room has a saved map: the session starts pibridge-roommap auto
    int refreshHz = 90;           // refresh_hz (PIMAX_FPS)
    QString sessionSlamConfig;    // PiBridge's generated Basalt config, set at the start ("" = slamConfig as it is)
    int sessionSlamCams = 0;      // cameras in that config's calibration (0 = not made or not known)
    QString slamTool;             // pibridge-slam (tools/slam; "" when not found): the four-camera calibration
    QString sessionsDir;          // the logs of inside-out sessions: $XDG_STATE_HOME/pibridge/inside-out
    MonadoBuildInfo build;        // what the monado-service is (markers, Rust or C++); unchecked = not looked at
    std::optional<CapState> caps; // its cap_sys_nice (nullopt = not looked at)
    QString steamVrStartup;       // <SteamVR runtime>/bin/vrstartup.sh ("" when SteamVR is not found)
    QString steamVrDriverDir;     // <the Monado build>/steamvr-monado ("" when the build has none)
    QString steamVrPreload;       // its bin/linux64/pimax_rttime_preload.so ("" when missing)
    QString driverEnvFile;        // $XDG_CONFIG_HOME/pibridge/steamvr-driver.env (PIMAX_STEAMVR_ENV_FILE)
    QStringList steamVrExtraEnv;  // added to SteamVR's launch environment (PATH with PiBridge's shims), set at the start

    std::vector<const RecPath*> paths() const {
        return {&monado, &cam, &ctrl, &basalt, &slamConfig, &lens, &calib, &winbridge, &vitBridge};
    }
};

// Resolves every item (see docs/DEVELOPMENT.md for the lookup order).
RecSetup resolveRecSetup(const Conf& conf, const RecEnv& env);
// A made-up setup for the fake mode and screenshots (nothing looked up).
RecSetup fakeRecSetup(const QString& recordingsDir);

// ---------------------------------------------------------------- detection

// The 3-letter PNP manufacturer id of an EDID ("PVR" for Pimax); "" when the block is too short.
QString edidVendor(const QByteArray& edid);
// The monitor name descriptor (tag 0xFC) of an EDID, trimmed; "".
QString edidName(const QByteArray& edid);
// The connector name ("DP-6") under drmRoot whose EDID's monitor name says Pimax (or vendor PVR); a connected one
// wins. "" when none.
QString pimaxConnector(const QString& drmRoot);
// /dev/videoN of the first "PimaxSensor" video node with index 0; "" when none.
QString pimaxCameraDevice(const QString& v4lRoot, const QString& devRoot);
// A USB device with idVendor 34a4 other than the camera (idProduct 0014).
bool pimaxHeadsetPresent(const QString& usbRoot);

// ---------------------------------------------------------------- can it start?

// The running processes that matter (read from /proc in the app, made up in the tests).
struct RecSystem {
    bool vrserver = false;   // SteamVR
    bool monado = false;     // another monado-service
    bool camBroker = false;  // a pimax-cam (another broker holds the camera)
    bool ctrltrack = false;  // a pimax-ctrltrack
    bool winbridge = false;  // a winbridge.exe (holds the port the session's would need)
    bool steamClient = true; // Steam's client runs (SteamVR + Monado needs it before SteamVR starts)
};
RecSystem readRecSystem(const QString& procRoot = QStringLiteral("/proc"));

struct RecReadiness {
    QStringList blocking;  // why it cannot start, in plain words (empty = it can)
    QStringList warnings;  // what is missing but not needed
    bool ready() const { return blocking.isEmpty(); }
};
// What the setup is checked for: a start of monado-service (Record tracking, Monado directly) or of SteamVR with
// driver_monado (no display connector needed: SteamVR's compositor picks the display; vrserver is checked by the
// caller with its own words; the realtime capability of monado-service does not matter).
enum class RecUse { Monado, SteamVr };
RecReadiness recReadiness(const RecSetup& s, const RecSystem& sys, RecUse use = RecUse::Monado);

} // namespace pibridge::app
