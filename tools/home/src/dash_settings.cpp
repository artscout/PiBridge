// SPDX-License-Identifier: MIT
#include "dash_settings.h"

#include <ctime>

#include "backgrounds.h"

#include "pibridge/hmd/haptics.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <system_error>

namespace pibridge::home::dash {

namespace {

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) {
        s.remove_prefix(1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n')) {
        s.remove_suffix(1);
    }
    return s;
}

// numbers in and out with a point whatever the locale (Qt's painter sets the process locale)
std::optional<double> number(std::string_view s) {
    s = trim(s);
    if (const std::size_t hash = s.find('#'); hash != std::string_view::npos) {
        s = trim(s.substr(0, hash));
    }
    double v = 0;
    const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (r.ec != std::errc() || r.ptr != s.data() + s.size() || !std::isfinite(v)) {
        return std::nullopt;
    }
    return v;
}

std::string fixed(double v, int digits) {
    char buf[32];
    const auto r = std::to_chars(buf, buf + sizeof buf, v, std::chars_format::fixed, digits);
    return std::string(buf, r.ptr);
}

// the value without a trailing " # comment" and quotes
std::string raw(std::string_view conf, std::string_view key) {
    std::string v(trim(bg::confValue(conf, key).value_or("")));
    if (const std::size_t hash = v.find(" #"); hash != std::string::npos) {
        v = std::string(trim(v.substr(0, hash)));
    }
    if (v.size() >= 2 && v.front() == '"' && v.back() == '"') {
        v = v.substr(1, v.size() - 2);
    }
    return v;
}

std::string word(std::string_view conf, std::string_view key) {
    std::string v = raw(conf, key);
    std::ranges::transform(v, v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v;
}

double ipdOf(std::string_view conf) {
    // rec_setup.cpp: 50..80 or the default
    const auto v = number(bg::confValue(conf, kKeyIpd).value_or(""));
    return v && *v >= 50.0 && *v <= 80.0 ? *v : kIpdDefault;
}

std::string angleWords(int deg) { return (deg > 0 ? "+" : "") + std::to_string(deg) + "\xC2\xB0"; }

const std::vector<std::string> kInhandValues{"always", "auto", "off"};
// the Recording page's choices, as record.toml spells them (tools/record/src/settings.cpp)
const std::vector<std::string> kRecEyes{"left", "both"};
const std::vector<std::string> kRecFps{"30", "60", "72"};
const std::vector<std::string> kRecQuality{"low", "medium", "high"};
const std::vector<std::string> kRecHeight{"1080", "1440", "2160"};
const std::vector<std::string> kRecCodec{"h264", "hevc"};
const std::vector<std::string> kRecOnOff{"on", "off"};

} // namespace

Backend detectBackend(const char* envBackend, std::string_view runtimeName) {
    if (envBackend != nullptr) {
        const std::string_view e = envBackend;
        if (e == "monado") {
            return Backend::Monado;
        }
        if (e == "steamvr") {
            return Backend::SteamVr;
        }
    }
    return runtimeName.find("Monado") != std::string_view::npos ? Backend::Monado : Backend::SteamVr;
}

const char* backendName(Backend b) { return b == Backend::Monado ? "Monado" : "SteamVR"; }

const char* pageTitle(Page p) {
    switch (p) {
    case Page::Headset: return "Headset";
    case Page::Tracking: return "Tracking";
    case Page::Controllers: return "Controllers";
    case Page::Room: return "Room";
    case Page::Recording: return "Recording";
    }
    return "";
}

const char* whenWords(When w, bool pending) {
    switch (w) {
    case When::Now: return "NOW";
    case When::Restart: return pending ? "AFTER RESTART (CHANGED)" : "AFTER RESTART";
    case When::NowAndRestart: return pending ? "LENSES NOW, PICTURE AFTER RESTART (CHANGED)" : "LENSES NOW, PICTURE AFTER RESTART";
    case When::Session: return "NOW, THIS SESSION";
    case When::Action: break;
    }
    return "";
}

std::string confSet(std::string_view text, std::string_view key, std::string_view value) {
    std::string out;
    bool found = false;
    std::size_t pos = 0;
    while (pos < text.size()) {
        const std::size_t nl = text.find('\n', pos);
        const std::string_view line = text.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos);
        pos = nl == std::string_view::npos ? text.size() : nl + 1;
        const std::string_view t = trim(line);
        const std::size_t eq = t.find('=');
        if (!t.empty() && t.front() != '#' && eq != std::string_view::npos && trim(t.substr(0, eq)) == key) {
            found = true;
            std::string next = std::string(key) + "=" + std::string(value);
            if (const std::size_t c = line.find(" #"); c != std::string_view::npos && c > line.find('=')) {
                next += std::string(line.substr(c)); // the trailing comment stays
            }
            out += next;
        } else {
            out += std::string(line);
        }
        out += '\n';
    }
    if (!found) {
        out += std::string(key) + "=" + std::string(value) + "\n";
    }
    return out;
}

std::optional<double> eyeHeight(std::string_view conf) {
    if (const auto e = number(bg::confValue(conf, kKeyEyeHeight).value_or("")); e && *e >= 1.0 && *e <= 2.3) {
        return e;
    }
    if (const auto b = number(bg::confValue(conf, kKeyBodyHeight).value_or("")); b && *b >= 1.2 && *b <= 2.3) {
        return *b * kEyeFromBody;
    }
    return std::nullopt;
}

bool roomsetupTakesBackend(std::string_view usageText) { return usageText.find("--backend") != std::string_view::npos; }

Settings::Settings(Backend b, Paths p, Sinks s) : backend_(b), paths_(std::move(p)), sinks_(std::move(s)) {}

Settings::Restart Settings::restartOf(std::string_view conf) const {
    Restart r;
    r.ipd = ipdOf(conf);
    r.refresh = word(conf, kKeyRefresh) == "72" ? 72 : 90;
    r.basalt = word(conf, kKeySlamEngine) == "basalt";
    r.ctrlTrack = bg::confFlag(conf, kKeyCtrlTrack, true);
    const auto fps = number(word(conf, kKeyCamFps));
    r.camFps = !fps || *fps >= 55 ? 60 : 50;
    return r;
}

void Settings::refresh() {
    conf_ = sinks_.read ? sinks_.read(paths_.conf).value_or("") : "";
    recordConf_ = sinks_.read && !paths_.recordConf.empty() ? sinks_.read(paths_.recordConf).value_or("") : "";
    if (!started_) {
        started_ = restartOf(conf_);
    }
    ipd_ = ipdOf(conf_);
    volume_ = sinks_.volume ? sinks_.volume() : std::nullopt;
    // the live file first (what the driver uses now), else the setting the session started with
    std::optional<double> deg;
    if (sinks_.read) {
        if (const auto t = sinks_.read(paths_.runtimeDir / "pimax-ctrl-angle")) {
            deg = number(*t);
        }
    }
    if (!deg) {
        deg = number(word(conf_, kKeyCtrlAngle));
    }
    angle_ = std::clamp(static_cast<int>(std::lround(deg.value_or(kAngleDefault))), kAngleMin, kAngleMax);
    // the driver's status file says the mode it runs ("mode=auto")
    if (sinks_.read) {
        if (const auto t = sinks_.read(paths_.runtimeDir / "pimax-ctrl-inhand")) {
            const std::size_t m = t->find("mode=");
            if (m != std::string::npos) {
                const std::string v(trim(std::string_view(*t).substr(m + 5, t->find('\n', m) - m - 5)));
                if (std::ranges::find(kInhandValues, v) != kInhandValues.end()) {
                    inhand_ = v;
                }
            }
        }
    }
}

std::string Settings::roomName() const {
    const std::string r = raw(conf_, kKeyRoom);
    return r.empty() ? std::string("home") : r;
}

std::vector<Item> Settings::items(Page p) const {
    std::vector<Item> v;
    const Restart now = restartOf(conf_);
    const Restart was = started_.value_or(now);
    const bool monado = backend_ == Backend::Monado;
    const auto choice = [&](std::string id, std::string label, std::vector<std::string> ch, int chosen, When w, bool pending) {
        Item it;
        it.id = std::move(id);
        it.label = std::move(label);
        it.kind = Item::Kind::Choice;
        it.choices = std::move(ch);
        it.chosen = chosen;
        it.when = w;
        it.pending = pending;
        v.push_back(std::move(it));
    };
    const auto text = [&](std::string line) {
        Item it;
        it.id = "text" + std::to_string(v.size());
        it.kind = Item::Kind::Text;
        it.value = std::move(line);
        it.when = When::Action;
        v.push_back(std::move(it));
    };
    switch (p) {
    case Page::Headset: {
        if (!paths_.record.empty()) {
            const bool rec = sinks_.recording && sinks_.recording();
            Item r;
            r.id = "record";
            r.label = rec ? "Stop recording" : "Record video";
            r.kind = Item::Kind::Button;
            r.value = rec ? "Recording now: ~/Videos/pimax" : "What you see, into ~/Videos/pimax";
            r.primary = true;
            r.enabled = true;
            r.when = When::Action;
            v.push_back(r);
        }
        Item ipd;
        ipd.id = "ipd";
        ipd.label = "IPD";
        ipd.kind = Item::Kind::Stepper;
        ipd.value = fixed(ipd_, 1) + " mm";
        ipd.when = When::NowAndRestart;
        ipd.pending = std::abs(now.ipd - was.ipd) > 0.01;
        v.push_back(ipd);
        Item vol;
        vol.id = "volume";
        vol.label = "Volume";
        vol.kind = Item::Kind::Stepper;
        vol.value = volume_ ? std::to_string(*volume_) + " %" : "no headset output";
        vol.enabled = volume_.has_value();
        vol.when = When::Now;
        v.push_back(vol);
        choice("refresh", "Refresh rate", {"72 Hz", "90 Hz"}, now.refresh == 72 ? 0 : 1, When::Restart, now.refresh != was.refresh);
        break;
    }
    case Page::Tracking:
        choice("slam", "Head tracking engine", {"Pimax", "Basalt"}, now.basalt ? 1 : 0, When::Restart, now.basalt != was.basalt);
        choice("ctrltrack", "Optical controller tracking", {"On", "Off"}, now.ctrlTrack ? 0 : 1, When::Restart,
               now.ctrlTrack != was.ctrlTrack);
        choice("camfps", "Camera rate", {"50 Hz", "60 Hz"}, now.camFps == 50 ? 0 : 1, When::Restart, now.camFps != was.camFps);
        text("Settings marked AFTER RESTART are saved in pibridge.conf; stop and start the Tracking session in PiBridge "
             "to use them.");
        break;
    case Page::Controllers: {
        Item a;
        a.id = "angle";
        a.label = "Controller angle";
        a.kind = Item::Kind::Stepper;
        a.value = angleWords(angle_);
        a.when = When::Now;
        v.push_back(a);
        const auto it = std::ranges::find(kInhandValues, inhand_);
        choice("inhand", "Controllers shown", {"Always", "When held", "Off"},
               static_cast<int>(it == kInhandValues.end() ? 0 : it - kInhandValues.begin()), When::Session, false);
        v.back().enabled = monado;
        if (!monado) {
            v.back().value = "Monado only";
        }
        choice("vibrate", "Test vibration", {"Left", "Right"}, -1, When::Action, false);
        break;
    }
    case Page::Recording: {
        // pibridge-record's settings (record.toml, docs/record.md); used from the next recording
        const bool rec = sinks_.recording && sinks_.recording();
        Item r;
        r.id = "record";
        r.label = rec ? "Stop recording" : "Record video";
        r.kind = Item::Kind::Button;
        r.value = rec ? "Recording now; changes apply to the next one" : "Into " + recordWord("dir", "~/Videos/pimax");
        r.primary = true;
        r.enabled = !paths_.record.empty();
        r.when = When::Action;
        v.push_back(r);
        const auto pick = [&](const char* id, const char* label, std::vector<std::string> shown,
                              const std::vector<std::string>& values, const char* key, const char* fallback) {
            const std::string cur = recordWord(key, fallback);
            const auto at = std::ranges::find(values, cur);
            choice(id, label, std::move(shown), at == values.end() ? -1 : static_cast<int>(at - values.begin()),
                   When::Action, false);
        };
        pick("rec_eyes", "Picture", {"One eye", "Both eyes"}, kRecEyes, "eyes", "left");
        pick("rec_fps", "Frame rate", {"30 fps", "60 fps", "72 fps"}, kRecFps, "fps", "30");
        pick("rec_quality", "Quality", {"Low", "Medium", "High"}, kRecQuality, "quality", "high");
        const auto mh = number(recordWord("max_height", "1080")).value_or(1080);
        choice("rec_height", "Resolution", {"1080p", "1440p", "4K"}, mh >= 2000 ? 2 : (mh >= 1300 ? 1 : 0), When::Action,
               false);
        pick("rec_codec", "Codec", {"H.264", "H.265"}, kRecCodec, "codec", "h264");
        pick("rec_audio", "Sound", {"On", "Off"}, kRecOnOff, "audio", "on");
        pick("rec_mic", "Microphone", {"On", "Off"}, kRecOnOff, "mic", "off");
        break;
    }
    case Page::Room: {
        Item rc;
        rc.id = "recenter";
        rc.label = "Recenter";
        rc.kind = Item::Kind::Button;
        rc.value = "Forward where you look, the centre where you stand";
        rc.primary = true;
        rc.enabled = monado;
        rc.when = When::Action;
        v.push_back(rc);
        text("Room: " + roomName());
        const auto eh = eyeHeight(conf_);
        text(eh ? "Eye height: " + fixed(*eh, 2) + " m (PiBridge's Room page)"
                : "Eye height: not entered (PiBridge's Room page); the setup measures the floor without it.");
        const bool can = monado && !paths_.roomsetup.empty() && !roomsetupPid_;
        Item full;
        full.id = "roomsetup";
        full.label = "Room setup";
        full.kind = Item::Kind::Button;
        full.value = "Map the room, the floor and the play area";
        full.primary = true;
        full.enabled = can;
        full.when = When::Action;
        v.push_back(full);
        full.primary = false;
        Item quick = full;
        quick.id = "roomquick";
        quick.label = "Quick setup";
        quick.value = "A circle around where you stand";
        quick.primary = false;
        v.push_back(quick);
        if (!monado) {
            text("Under SteamVR the room setup starts from PiBridge's Room page.");
        } else if (paths_.roomsetup.empty()) {
            text("pibridge-roomsetup was not found next to pibridge-home or in PATH.");
        } else if (roomsetupPid_) {
            text("The room setup is running in the headset.");
        } else if (!roomsetupLast_.empty()) {
            text(roomsetupLast_);
        }
        break;
    }
    }
    return v;
}

std::string Settings::recordWord(std::string_view key, std::string_view fallback) const {
    const std::string v = word(recordConf_, key);
    return v.empty() ? std::string(fallback) : v;
}

bool Settings::setRecord(std::string_view key, std::string_view value) {
    if (paths_.recordConf.empty()) {
        return false;
    }
    const std::string base = sinks_.read ? sinks_.read(paths_.recordConf).value_or("") : "";
    const std::string next = confSet(base, key, value);
    if (!sinks_.write || !sinks_.write(paths_.recordConf, next)) {
        return false;
    }
    recordConf_ = next;
    return true;
}

bool Settings::setConf(std::string_view key, std::string_view value) {
    const std::string base = sinks_.read ? sinks_.read(paths_.conf).value_or("") : ""; // another writer may have changed it
    const std::string next = confSet(base, key, value);
    if (!sinks_.write || !sinks_.write(paths_.conf, next)) {
        return false;
    }
    conf_ = next;
    return true;
}

std::string Settings::startRoomSetup(bool quick) {
    if (backend_ != Backend::Monado) {
        return "Under SteamVR the room setup starts from PiBridge's Room page.";
    }
    if (paths_.roomsetup.empty()) {
        return "pibridge-roomsetup was not found.";
    }
    if (roomsetupPid_) {
        return "The room setup is already running.";
    }
    // Monado + the Lighthouse faceplate: no room map, the room is the backend's Lighthouse stage (pibridge-roomsetup
    // --lh-stage: centre, forward and the floor in the headset, applied at once); inside-out: the mapping walk
    const bool lighthouse = word(conf_, "tracking_mode") != "inside-out";
    std::vector<std::string> argv{paths_.roomsetup, lighthouse ? "--lh-stage" : "--map", "--room", roomName()};
    if (const auto eh = eyeHeight(conf_)) {
        argv.insert(argv.end(), {"--eye-height", fixed(*eh, 3)});
    }
    if (quick && !lighthouse) {
        argv.emplace_back("--quick-direct");
    }
    // the flag is new: an older pibridge-roomsetup refuses unknown options, the variable is read by both
    const auto usage = sinks_.usage ? sinks_.usage(paths_.roomsetup) : std::nullopt;
    if (usage && roomsetupTakesBackend(*usage)) {
        argv.insert(argv.end(), {"--backend", "monado"});
    }
    const auto pid = sinks_.spawn ? sinks_.spawn(argv, {"PIBRIDGE_BACKEND=monado"}) : std::nullopt;
    if (!pid) {
        return "The room setup could not start.";
    }
    roomsetupPid_ = pid;
    roomsetupLast_.clear();
    return quick ? "Quick setup started: follow it in the headset." : "Room setup started: follow it in the headset.";
}

void Settings::roomsetupEnded(int code) {
    roomsetupPid_.reset();
    roomsetupLast_ = code == 0 ? "The last room setup finished." : "The last room setup ended with code " + std::to_string(code) + ".";
}

std::string Settings::act(const std::string& box) {
    const std::size_t colon = box.find(':');
    const std::string id = box.substr(0, colon);
    const std::string arg = colon == std::string::npos ? std::string() : box.substr(colon + 1);
    const int dir = arg == "+" ? 1 : arg == "-" ? -1 : 0;
    int idx = -1;
    if (!arg.empty() && arg != "+" && arg != "-") {
        (void)std::from_chars(arg.data(), arg.data() + arg.size(), idx);
    }
    const auto confWords = [&](std::string_view key, std::string_view value, const std::string& what) {
        return setConf(key, value) ? what + ": saved, used after the Tracking session restarts."
                                   : "Could not write " + paths_.conf.string() + ".";
    };
    if (id == "ipd" && dir != 0) {
        ipd_ = std::clamp(std::round((ipd_ + dir * kIpdStep) / kIpdStep) * kIpdStep, kIpdMin, kIpdMax);
        const std::string mm = fixed(ipd_, 1);
        const bool motor = sinks_.write && sinks_.write(paths_.statusDir / "ipd-request", mm + "\n");
        const bool saved = setConf(kKeyIpd, mm);
        if (!motor) {
            return "IPD " + mm + " mm: could not write the lens request.";
        }
        return "IPD " + mm + " mm: the lenses move now" +
               (saved ? std::string(", the picture uses it after the Tracking session restarts.") : std::string(" (not saved)."));
    }
    if (id == "volume" && dir != 0) {
        if (!volume_) {
            return "No headset sound output.";
        }
        const int target = std::clamp(*volume_ + dir * kVolumeStep, 0, 100);
        if (!sinks_.setVolume || !sinks_.setVolume(target)) {
            return "The volume could not be set.";
        }
        volume_ = target;
        return "Volume " + std::to_string(target) + " %.";
    }
    if (id == "refresh" && (idx == 0 || idx == 1)) {
        return confWords(kKeyRefresh, idx == 0 ? "72" : "90", idx == 0 ? "Refresh rate 72 Hz" : "Refresh rate 90 Hz");
    }
    if (id == "slam" && (idx == 0 || idx == 1)) {
        return confWords(kKeySlamEngine, idx == 0 ? "pimax" : "basalt", idx == 0 ? "Pimax head tracking" : "Basalt head tracking");
    }
    if (id == "ctrltrack" && (idx == 0 || idx == 1)) {
        return confWords(kKeyCtrlTrack, idx == 0 ? "on" : "off",
                         idx == 0 ? "Optical controller tracking on" : "Optical controller tracking off");
    }
    if (id == "camfps" && (idx == 0 || idx == 1)) {
        return confWords(kKeyCamFps, idx == 0 ? "50" : "60", idx == 0 ? "Camera rate 50 Hz" : "Camera rate 60 Hz");
    }
    if (id == "angle" && dir != 0) {
        angle_ = std::clamp(angle_ + dir * kAngleStep, kAngleMin, kAngleMax);
        const bool live = sinks_.write && sinks_.write(paths_.runtimeDir / "pimax-ctrl-angle", std::to_string(angle_) + "\n");
        const bool saved = setConf(kKeyCtrlAngle, std::to_string(angle_));
        return "Controller angle " + angleWords(angle_) + (live ? "" : ": could not write the live file") +
               (saved ? "." : " (not saved).");
    }
    if (id == "inhand" && idx >= 0 && idx < static_cast<int>(kInhandValues.size())) {
        if (backend_ != Backend::Monado) {
            return "Only Monado's driver hides the controllers.";
        }
        const std::string& m = kInhandValues[static_cast<std::size_t>(idx)];
        if (!sinks_.write || !sinks_.write(paths_.runtimeDir / "pimax-ctrl-inhand-mode", m + "\n")) {
            return "Could not write the controller request.";
        }
        inhand_ = m;
        return idx == 0 ? "Controllers always shown (this session)."
                        : idx == 1 ? "Controllers shown while held (this session)." : "Controllers hidden (this session).";
    }
    if (id == "vibrate" && (idx == 0 || idx == 1)) {
        // the slots of the left and right controller (pimax_ctrl.c: 2 and 3)
        const std::string line = hmd::haptics::format_request(idx == 0 ? 2 : 3, 60, 200);
        if (!sinks_.write || !sinks_.write(paths_.runtimeDir / std::string(hmd::haptics::REQUEST_FILE), line)) {
            return "Could not write the vibration request.";
        }
        return idx == 0 ? "Left controller: test vibration sent." : "Right controller: test vibration sent.";
    }
    if (id == "roomsetup" || id == "roomquick") {
        return startRoomSetup(id == "roomquick");
    }
    if (id.starts_with("rec_") && idx >= 0) {
        struct R {
            const char* id;
            const char* key;
            const std::vector<std::string>* values;
            bool quoted;
            const char* what;
        };
        static const R kRec[] = {{"rec_eyes", "eyes", &kRecEyes, true, "Picture"},
                                 {"rec_fps", "fps", &kRecFps, false, "Frame rate"},
                                 {"rec_quality", "quality", &kRecQuality, true, "Quality"},
                                 {"rec_height", "max_height", &kRecHeight, false, "Resolution"},
                                 {"rec_codec", "codec", &kRecCodec, true, "Codec"},
                                 {"rec_audio", "audio", &kRecOnOff, true, "Sound"},
                                 {"rec_mic", "mic", &kRecOnOff, true, "Microphone"}};
        for (const R& r : kRec) {
            if (id == r.id && idx < static_cast<int>(r.values->size())) {
                const std::string& val = (*r.values)[static_cast<std::size_t>(idx)];
                return setRecord(r.key, r.quoted ? "\"" + val + "\"" : val)
                           ? std::string(r.what) + ": saved, used from the next recording."
                           : "Could not write " + paths_.recordConf.string() + ".";
            }
        }
    }
    if (id == "record") {
        const bool rec = sinks_.recording && sinks_.recording();
        if (paths_.record.empty() || !sinks_.spawn ||
            !sinks_.spawn({paths_.record, rec ? "stop" : "start", "--brief"}, {})) {
            return "pibridge-record could not start.";
        }
        return rec ? "Recording stopped: the video is in ~/Videos/pimax." : "Recording.";
    }
    if (id == "recenter") {
        // the Pimax driver recenters when this file changes (its long-press recenter, from the dashboard)
        const bool ok = sinks_.write && sinks_.write(paths_.statusDir / "recenter-request", std::to_string(std::time(nullptr)) + "\n");
        return ok ? "Recentered: forward is where you look." : "Could not reach the driver.";
    }
    return {};
}

} // namespace pibridge::home::dash
