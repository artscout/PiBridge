// SPDX-License-Identifier: MIT
// Inside-out head tracking (docs/TRACKING.md): the settings of the Pimax driver in Monado (Basalt SLAM on the headset's
// tracking cameras, optical controller tracking), saved in pibridge.conf, and the ONE builder of the session environment
// that every place starting an inside-out Monado uses (Record tracking on the Development page, START TRACKING).
//
// Where the knobs come from (read there, not invented): ~/gamedev/ext/tools/crystal-6dof.sh (the session env of today,
// with the steamvr variant through ~/.config/omarchyvr/steamvr-driver.env and the LD_PRELOAD of pimax_rttime_preload.so),
// the Monado Pimax driver (forks/monado src/xrt/drivers/pimax: DEBUG_GET_ONCE_* options and getenv), omarchyvr's
// pimax-cam (headpose.rs HeadParams::from_env, ae.rs AeParams::from_env_model), pimax-hands (fuse.rs), and the study
// docs/research/slam-imu-intrinsics.md (factory accel bias yes, ka/na never, grid 30, camera AE on).
//
// Values Basalt reads from its own files (the detection grid, the accel and gyro bias priors) go into PiBridge's copy of
// the Basalt config under $XDG_CONFIG_HOME/pibridge/slam/, generated from the base config the session used so far (read
// only); SLAM_CONFIG then points there (generateSlamConfig()).
#pragma once

#include <QString>
#include <QStringList>
#include <array>
#include <optional>
#include <vector>

#include "conf.h"

namespace pibridge::app {

struct RecSetup;

// The pibridge.conf keys of the inside-out settings. Paths and the extra environment reuse the rec_* keys of the
// recorder (core/rec_setup.h); the refresh rate is the shared refresh_hz.
namespace iokey {
inline constexpr char kMode[] = "tracking_mode";          // lighthouse | inside-out
inline constexpr char kApps[] = "io_apps";                // steamvr | monado
inline constexpr char kHome[] = "io_home_scene";          // on | off             pibridge-home in "Monado directly"
inline constexpr char kSlam[] = "io_slam";                // on | off             PIMAX_SLAM
inline constexpr char kSlamEngine[] = "io_slam_engine";   // basalt | pimax       VIT_SYSTEM_LIBRARY_PATH (pimax: + winbridge)
inline constexpr char kGrid[] = "io_slam_grid";           // 30 | 40 | 50 (px)    Basalt optical_flow_detection_grid_size
inline constexpr char kSlamCams[] = "io_slam_cams";       // 2 | 4               PIMAX_SLAM_CAMS 0,1 | 0,1,2,3 + Basalt calib
inline constexpr char kAccelBias[] = "io_slam_accel_bias"; // factory | off      Basalt calib_accel_bias
inline constexpr char kGyroBias[] = "io_slam_gyro_bias";  // measured | factory | off   Basalt calib_gyro_bias
inline constexpr char kDriftHold[] = "io_drift_hold";     // on | off             PIMAX_SLAM_DRIFT_HOLD
inline constexpr char kStillHold[] = "io_still_hold";     // on | off             PIMAX_SLAM_STILL
inline constexpr char kHeadTilt[] = "io_head_tilt";       // on | off             PIMAX_HEAD_TILT
inline constexpr char kFuseAuto[] = "io_fuse_auto";       // on | off             PIMAX_FUSE_AUTO
inline constexpr char kSmooth[] = "io_slam_smooth";       // on | off             PIMAX_SLAM_SMOOTH
inline constexpr char kAnchor[] = "io_anchor";            // on | off             PIMAX_ANCHOR
inline constexpr char kAnchorGlide[] = "io_anchor_glide"; // on | off             PIMAX_ANCHOR_GLIDE
inline constexpr char kCamAe[] = "io_cam_ae";             // on | dry | off       PIMAX_CAM_AE
inline constexpr char kCamAeModel[] = "io_cam_ae_model";  // crystal-super | auto PIMAX_CAM_AE_MODEL
inline constexpr char kCamAeLaw[] = "io_cam_ae_law";      // new | old            PIMAX_CAM_AE_LAW
inline constexpr char kCtrlTrack[] = "io_ctrl_track";     // on | off             PIMAX_CTRL_TRACK + pimax-ctrltrack
inline constexpr char kCtrlAngle[] = "io_ctrl_angle";     // -20..70 (deg)        PIMAX_CTRL_ANGLE
inline constexpr char kCtrlLost[] = "io_ctrl_lost_s";     // 1..10 (s)            PIMAX_CTRL_TRACK_LOST_S
inline constexpr char kCtrlSteady[] = "io_ctrl_steady";   // on | off             PIMAX_CTRL_TRACK_HYST
} // namespace iokey

enum class TrackingMode { Lighthouse, InsideOut };
enum class IoApps { SteamVr, Monado };
enum class AccelBias { Factory, Off };
enum class GyroBias { Measured, Factory, Off };
enum class CamAe { On, Dry, Off };
// Who computes the head pose from the cameras and the IMU: Basalt (libbasalt.so) or Pimax's own SLAM engine
// (pimax_slam.pi.dll of Pimax Play, run under Wine by winbridge.exe; Monado reaches it through
// libpibridge_vit_winbridge.so, docs/WINBRIDGE.md).
enum class SlamEngine { Basalt, Pimax };

inline constexpr int kCtrlAngleMin = -20; // pimax_ctrl.c: PIMAX_CTRL_ANGLE -20..+70, default +30
inline constexpr int kCtrlAngleMax = 70;
inline constexpr int kCtrlLostMin = 1;
inline constexpr int kCtrlLostMax = 10;

struct IoSettings {
    TrackingMode mode = TrackingMode::Lighthouse; // the setup that works today stays the default
    IoApps apps = IoApps::SteamVr;
    bool homeScene = true; // pibridge-home in the headset while the session runs with apps through Monado directly
    bool slam = true;
    SlamEngine slamEngine = SlamEngine::Basalt; // the engine of today stays the default
    int grid = 30;
    // Head tracking cameras: 2 = the lower tracking pair (0, 1), 4 = also the upper side pair (2, 3). 2 until a live
    // take 4 vs 2 decides (docs/ROADMAP.md 1b, docs/TRACKING.md).
    int slamCams = 2;
    AccelBias accelBias = AccelBias::Factory;
    GyroBias gyroBias = GyroBias::Measured;
    bool driftHold = false; // Crystal Super: off (study 3: it takes slow steps and leans for drift, 273 vs 74 mm)
    bool stillHold = true;
    bool headTilt = true;
    bool fuseAuto = true;
    bool smooth = true;
    bool anchor = false;
    bool anchorGlide = true;
    CamAe camAe = CamAe::On;
    bool camAeCrystal = true; // PIMAX_CAM_AE_MODEL=crystal-super (false: detect from USB)
    bool camAeNewLaw = true;
    bool ctrlTrack = true;
    int ctrlAngle = 30;
    int ctrlLostS = 4;
    bool ctrlSteady = true;

    // The recommended values (the sources above); equal to a default-constructed IoSettings except the mode, which
    // has no recommendation (Lighthouse needs the faceplate and base stations, inside-out is experimental).
    static IoSettings recommended() { return {}; }
    bool operator==(const IoSettings&) const = default;
};

// Read from pibridge.conf (a missing or broken value: the default). Integers are clamped to their ranges, the grid to
// one of 30 / 40 / 50, the camera count to 2 or 4 (anything but 4 is 2).
IoSettings ioSettings(const Conf& conf);
// The (key, value) lines that store `s` (every inside-out key, mode and apps included).
std::vector<std::pair<std::string, std::string>> ioConfLines(const IoSettings& s);
// Every key the page writes on "Reset to recommended" (all io_* keys; not the mode, apps, the home scene, refresh_hz
// or the paths).
std::vector<std::pair<std::string, std::string>> ioRecommendedLines();

// ---------------------------------------------------------------- the environment

// What a session runs and with which environment (pure; tested).
struct IoSessionEnv {
    QStringList driver;   // the Pimax driver's settings: monado-service and SteamVR's driver_monado.so read the same
    QStringList monado;   // monado-service's environment: driver + the direct-mode compositor (+ the recorder)
    QStringList steamvr;  // SteamVR's launch environment (steamvr variant): VALVE_SKIP_RUNTIME_SAFETY, LD_PRELOAD, ...
    bool ctrltrack = true; // start pimax-ctrltrack next to the broker
    bool winbridge = false; // start winbridge.exe (Wine) next to the broker: the Pimax SLAM engine
    QStringList notes;    // what was left out and why, in plain words
};

// `slamConfig`: the SLAM_CONFIG to use ("" = none: the driver hands Basalt its own calibration and the built-in
// settings). Four head tracking cameras (PIMAX_SLAM_CAMS=0,1,2,3) only when Basalt gets a calibration with four:
// without a SLAM_CONFIG (the driver's own), or with s.sessionSlamCams == 4 (prepareSessionSlam made one); otherwise
// the lower pair and a note (Basalt aborts the process on a camera count that differs from its calibration's). `recordDir`: PIMAX_SLAM_RECORD ("" = no recording). `driverEnvFile`: the file SteamVR's driver_monado
// reads (PIMAX_STEAMVR_ENV_FILE), `preload`: pimax_rttime_preload.so (both only for the steamvr variant).
IoSessionEnv buildIoEnv(const IoSettings& io, const RecSetup& s, int refreshHz, const QString& slamConfig,
                        const QString& recordDir, const QString& driverEnvFile = {}, const QString& preload = {});
// The Pimax SLAM engine can run: winbridge.exe, libpibridge_vit_winbridge.so, Wine and the Pimax kit (core/rec_setup.h
// RecSetup::win*) are all there. `why`: what is missing, in plain words ("" when ready).
bool pimaxSlamReady(const RecSetup& s, QString* why = nullptr);
// The controllers come from Pimax's LED engine in the same winbridge (optical controller tracking on, the Pimax
// head engine running): PIMAX_CTRL_ENGINE=1 for the driver, and pimax-ctrltrack is not started.
bool pimaxCtrlEngine(const RecSetup& s);
// The text of the driver env file (KEY=VALUE lines, as crystal-6dof.sh writes ~/.config/omarchyvr/steamvr-driver.env).
QString driverEnvFileText(const IoSessionEnv& e);

// ---------------------------------------------------------------- Basalt config

// PiBridge's Basalt files: $XDG_CONFIG_HOME/pibridge/slam/{pimax-basalt.toml, pimax-basalt-config.json, pimax-basalt-calib.json}.
QString slamDir();

// The factory IMU terms of <SFConfig><Stateinit aBias="x y z" wBias="x y z">, used as is (body frame).
struct FactoryImu {
    std::optional<std::array<double, 3>> aBias; // m/s^2, nullopt when absent or implausible (> 1 m/s^2)
    std::optional<std::array<double, 3>> wBias; // rad/s, nullopt when absent or implausible (> 0.1 rad/s)
};
FactoryImu readFactoryImu(const QByteArray& xml);
// The driver's gyro bias cache text (raw IMU axes, deg/s; pimax_hmd.c bias_cache_save) in the body frame (raw x and z
// negated) in rad/s; nullopt when malformed or implausible (> 5 deg/s). Twin of pimax-cam slam.rs parse_gyro_bias_cache.
std::optional<std::array<double, 3>> parseGyroBiasCache(const QByteArray& text);
// $XDG_CACHE_HOME/omarchyvr/gyro-bias (else ~/.cache/omarchyvr/gyro-bias): the cache the Pimax driver writes.
QString gyroBiasCachePath();

struct SlamGenInput {
    QString baseToml;      // the base config (rec_slam_config); read only
    QString outDir;        // where PiBridge's copy goes
    int grid = 30;
    AccelBias accelBias = AccelBias::Factory;
    GyroBias gyroBias = GyroBias::Measured;
    QString factoryXml;    // the headset's factory calibration (rec_cam_calib)
    QString gyroCache;     // gyroBiasCachePath()
    // A Basalt calibration with more cameras (`pibridge-slam export --cams 0,1,2,3`): its T_imu_cam, intrinsics and
    // resolution replace the base's, which must be its first cameras (the same headset); "" = the base's cameras.
    // More than two cameras also turn on config.optical_flow_detection_nonoverlap (tools/slam export.h).
    QString camsCalib;
    // The IMU noise Basalt weighs the IMU with: true = measured (imuNoiseFile, else $PIMAX_SLAM_IMU or
    // ~/.config/omarchyvr/slam-imu.json, else the Crystal Super's own measurement built in); false = the base
    // calibration's (Pimax's Windows densities, about 100x larger). Measured: Basalt's vertical motion went from about
    // 0.8 of the real one to 0.94-0.99 (docs/research/head-tracking-improvements.md, "Scale").
    bool measuredImuNoise = true;
    QString imuNoiseFile;
    bool write = true;     // false: only work out what would be written (the page's preview)
};
struct SlamGenResult {
    bool ok = false;
    QString toml;          // the generated SLAM_CONFIG ("" when !ok)
    QString error;         // plain words when !ok
    std::array<double, 3> accel{};
    std::array<double, 3> gyro{};
    QString accelFrom;     // "factory calibration", "off", "base config (no factory value)"
    QString gyroFrom;      // "measured at rest (driver cache)", "factory calibration", "off", ...
    QString noiseFrom;     // where accel_noise_std / gyro_noise_std came from ("" = the base's, unchanged)
    int cams = 0;          // cameras in the generated calibration
};
// Reads the base toml, its cam-calib and config-path files, and writes the three files into outDir with: the
// detection grid; calib_accel_bias = [aBias, 0 x 6] (never the factory scale ka or misalignment na); calib_gyro_bias
// = [bias, 0 x 9]. Every other byte of the base files is kept. When a factory or measured value is missing, the
// next source is used (gyro: measured -> factory -> base config's value; accel: factory -> base config's value).
SlamGenResult generateSlamConfig(const SlamGenInput& in);
// The cam-calib and config-path files a Basalt toml names (relative paths resolved next to it); "" when absent.
struct SlamTomlFiles {
    QString calib;
    QString config;
};
SlamTomlFiles slamTomlFiles(const QString& toml);
// The number of cameras of a Basalt calibration text ("value0".intrinsics); 0 when it is not one.
int basaltCalibCams(const QByteArray& json);
// The arguments of `pibridge-slam export` that write a four-camera Basalt calibration of the headset (factoryXml) into
// outDir, with the VIO settings of baseConfig (tools/slam, docs/pibridge-slam.md).
QStringList fourCamExportArgs(const QString& outDir, const QString& factoryXml, const QString& baseConfig);

// Synthetic base files for the fake mode and the tests (made-up numbers, no headset's data): <dir>/base.toml with
// base-calib.json (calib_accel_bias / calib_gyro_bias 0) and base-config.json (grid 40), and <dir>/factory.xml with a
// <Stateinit aBias wBias ka na> line. Returns the toml path ("" when it could not be written).
QString writeSyntheticSlamBase(const QString& dir);

// ---------------------------------------------------------------- small helpers for the page

// The DRM connectors under drmRoot ("DP-6"), connected ones first, each once.
QStringList drmConnectors(const QString& drmRoot = QStringLiteral("/sys/class/drm"));
// pimax_rttime_preload.so of the Monado build that holds `monadoService` (its steamvr-monado/bin/linux64/ next to
// it or up to four folders above); "" when none.
QString rttimePreloadFor(const QString& monadoService);
// Plain words for the dashboard chip and the page: "LIGHTHOUSE" / "INSIDE-OUT".
QString modeWord(TrackingMode m);

} // namespace pibridge::app
