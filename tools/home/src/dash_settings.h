// SPDX-License-Identifier: MIT
// The headset settings of the VR dashboard (dash_view.h lays them out, xrhome.cpp feeds them). Every entry has a real
// receiver, read there and not invented:
//
//   applied at once
//     IPD               $XDG_RUNTIME_DIR/pibridge/ipd-request "MM" (libpibridge-hmd ipd.h): the lens motors, taken by
//                       the Pimax driver in Monado (pimax_ctrl.c pimax_ipd_poll_request) or by pibridge-keepalived. The
//                       same value goes to ipd_mm in pibridge.conf: the picture's eye distance (OXR_DEBUG_IPD_MM, set
//                       when the session starts) follows at the next start.
//     Volume            the headset's PipeWire output (libpibridge-hmd audio.h headset_sink, pactl); pibridge-keepalived
//                       keeps the volume across reconnects.
//     Controller angle  $XDG_RUNTIME_DIR/pimax-ctrl-angle (degrees; pimax_ctrl.c and pibridge-headtrack read it live) and
//                       io_ctrl_angle in pibridge.conf (PIMAX_CTRL_ANGLE at the next start).
//     Controllers shown $XDG_RUNTIME_DIR/pimax-ctrl-inhand-mode auto | always | off (Monado's Pimax driver only; this
//                       session only: no pibridge.conf key, the driver starts with "always").
//     Vibrate           $XDG_RUNTIME_DIR/pimax-haptic-request "SLOT AMP MS" (libpibridge-hmd haptics.h).
//   at the next start of the Tracking session (pibridge.conf, read by the app's buildIoEnv / rec_setup)
//     refresh_hz 72 | 90 (PIMAX_FPS), io_slam_engine pimax | basalt, io_ctrl_track on | off, io_cam_fps 50 | 60.
//
// Room setup (Monado only): pibridge-roomsetup --map --room <io_room> [--eye-height M] [--quick-direct]
// [--backend monado], with PIBRIDGE_BACKEND=monado in its environment. It is an OpenXR app of the same Monado: it
// takes the headset while it runs and the home is shown again when it ends (loop.h).
//
// Pure: the files, the volume and the process start go through Sinks (dash_sys.h has the real ones, the tests fakes).
#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::home::dash {

enum class Backend { SteamVr, Monado };
/// PIBRIDGE_BACKEND ("monado" | "steamvr") when set, else from the OpenXR runtime's name (Monado's says "Monado").
Backend detectBackend(const char* envBackend, std::string_view runtimeName);
const char* backendName(Backend b);

enum class Page { Headset, Tracking, Controllers, Room, Recording };
inline constexpr Page kPages[] = {Page::Headset, Page::Tracking, Page::Controllers, Page::Room, Page::Recording};
const char* pageTitle(Page p);

/// When a change takes effect, as the row says it.
enum class When { Now, Restart, NowAndRestart, Session, Action };

struct Item {
    enum class Kind { Stepper, Choice, Button, Text };
    std::string id;     // "ipd", "volume", ...; the boxes are "<id>:-", "<id>:+", "<id>:<choice>", "<id>"
    std::string label;
    std::string value;  // Stepper: the value in words ("63.5 mm"); Text: the line
    Kind kind = Kind::Text;
    When when = When::Now;
    std::vector<std::string> choices; // Choice
    int chosen = -1;
    bool enabled = true;
    bool pending = false; // a restart setting differs from what the running session started with
    bool primary = false; // Button: the page's main action
};
/// The words of a row's timing tag ("NOW", "AFTER RESTART", ...); "" for none.
const char* whenWords(When w, bool pending);

// pibridge.conf keys (the app's names: core/rec_setup.h, core/inside_out.h, core/roommap_link.h)
inline constexpr char kKeyIpd[] = "ipd_mm";
inline constexpr char kKeyRefresh[] = "refresh_hz";
inline constexpr char kKeySlamEngine[] = "io_slam_engine";
inline constexpr char kKeyCtrlTrack[] = "io_ctrl_track";
inline constexpr char kKeyCamFps[] = "io_cam_fps";
inline constexpr char kKeyCtrlAngle[] = "io_ctrl_angle";
inline constexpr char kKeyRoom[] = "io_room";
inline constexpr char kKeyBodyHeight[] = "room_body_height_m";
inline constexpr char kKeyEyeHeight[] = "room_eye_height_m";

inline constexpr double kIpdMin = 58.0, kIpdMax = 72.0, kIpdStep = 0.5, kIpdDefault = 63.0; // the motor's range
inline constexpr int kVolumeStep = 5;
inline constexpr int kAngleMin = -20, kAngleMax = 70, kAngleStep = 5, kAngleDefault = 52; // pimax_ctrl.c, ioSettings
inline constexpr double kEyeFromBody = 0.936; // core/roommap_link.h

/// The text with every live `key=` line set to `value` (a trailing " # comment" kept), or `key=value` appended when
/// there is none; commented lines are left alone; ends with a newline (the app's confSet rules, core/conf.h).
std::string confSet(std::string_view text, std::string_view key, std::string_view value);
/// The room setup's eye height: room_eye_height_m (1.0..2.3), else room_body_height_m (1.2..2.3) * 0.936.
std::optional<double> eyeHeight(std::string_view confText);
/// pibridge-roomsetup's usage text names --backend (the flag is new; an older binary refuses unknown options).
bool roomsetupTakesBackend(std::string_view usageText);

struct Paths {
    std::filesystem::path conf;       // pibridge.conf
    std::filesystem::path runtimeDir; // $XDG_RUNTIME_DIR (pimax-ctrl-angle, pimax-ctrl-inhand*, pimax-haptic-request)
    std::filesystem::path statusDir;  // $XDG_RUNTIME_DIR/pibridge (ipd-request)
    std::string roomsetup;            // pibridge-roomsetup; "" = not found
    std::string record;               // pibridge-record; "" = not found
    std::filesystem::path recordConf; // pibridge-record's settings (record.toml next to pibridge.conf)
};

struct Sinks {
    std::function<std::optional<std::string>(const std::filesystem::path&)> read;
    std::function<bool(const std::filesystem::path&, const std::string&)> write; // atomic (temporary file + rename)
    std::function<std::optional<int>()> volume;  // the headset output's volume in percent; nullopt: no headset output
    std::function<bool(int percent)> setVolume;  // false: no headset output
    std::function<bool()> recording;             // a video recording runs (pibridge-record's recorder process)
    /// Start `argv` detached with `env` added; the pid, or nullopt when it could not start.
    std::function<std::optional<long>(const std::vector<std::string>& argv, const std::vector<std::string>& env)> spawn;
    /// `pibridge-roomsetup --help`'s text (nullopt when it would not run).
    std::function<std::optional<std::string>(const std::string& program)> usage;
};

class Settings {
public:
    Settings(Backend b, Paths p, Sinks s);
    Backend backend() const { return backend_; }
    /// Read pibridge.conf, the live files and the volume. The first call also remembers the restart settings as the
    /// ones the running session started with (the home starts with the session).
    void refresh();
    std::vector<Item> items(Page p) const;
    /// A box was clicked ("ipd:+", "refresh:0", "roomsetup", ...): apply it and say what happened in plain words
    /// ("" for an unknown box).
    std::string act(const std::string& box);
    /// The room setup that was started from here: its pid while it runs; roomsetupEnded(code) when it has ended.
    std::optional<long> roomsetupPid() const { return roomsetupPid_; }
    void roomsetupEnded(int code);
    std::string roomName() const;
    /// A record.toml setting as pibridge-record reads it (lower case, no quotes), else `fallback`.
    std::string recordWord(std::string_view key, std::string_view fallback) const;
    /// A room setup can start from here now (Monado, pibridge-roomsetup found, none running).
    bool canRecord() const { return !paths_.record.empty(); }
    bool recordingNow() const { return sinks_.recording && sinks_.recording(); }
    bool canStartRoomSetup() const { return backend_ == Backend::Monado && !paths_.roomsetup.empty() && !roomsetupPid_; }

private:
    bool setRecord(std::string_view key, std::string_view value);
    struct Restart {
        double ipd = kIpdDefault; // the picture's eye distance (OXR_DEBUG_IPD_MM)
        int refresh = 90;
        bool basalt = false;
        bool ctrlTrack = true;
        int camFps = 60;
        bool operator==(const Restart&) const = default;
    };
    Restart restartOf(std::string_view conf) const;
    bool setConf(std::string_view key, std::string_view value);
    std::string startRoomSetup(bool quick);

    Backend backend_;
    Paths paths_;
    Sinks sinks_;
    std::string conf_;
    std::string recordConf_; // record.toml's text
    std::optional<Restart> started_;
    double ipd_ = kIpdDefault;
    std::optional<int> volume_;
    int angle_ = kAngleDefault;
    std::string inhand_ = "always";
    std::optional<long> roomsetupPid_;
    std::string roomsetupLast_; // the last room setup's end, in words
};

} // namespace pibridge::home::dash
