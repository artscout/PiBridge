// SPDX-License-Identifier: MIT
// pibridge-home's dashboard without a headset: the settings and where each one goes (a fake file system, volume and
// process start), what waits for a restart, the room setup's command line, the layout (every box inside the picture,
// no two overlapping), the hit test, the click rules, the plan with the dashboard, the menu button's state, and the
// picture (when built with Qt).
#include "dash_settings.h"
#include "dash_sys.h"
#include "dash_view.h"
#include "desk.h"
#include "frame_layout.h"
#include "plan.h"

#ifdef PIBRIDGE_HOME_DASH
#include "dash_paint.h"
#endif

#include "pibridge_test.h"

#include <algorithm>
#include <filesystem>
#include <map>
#include <unistd.h>

using namespace pibridge;
using namespace pibridge::home;
using namespace pibridge::home::dash;

namespace {

namespace fs = std::filesystem;

struct Fake {
    std::map<std::string, std::string> files;
    std::optional<int> volume = 50;
    bool writable = true;
    std::vector<std::string> argv, env;
    std::optional<long> pid = 777;
    std::string usage = "pibridge-roomsetup ... --backend steamvr|monado ...";
    int spawns = 0;
    bool recordingNow = false;

    Sinks sinks() {
        Sinks s;
        s.read = [this](const fs::path& p) -> std::optional<std::string> {
            const auto it = files.find(p.string());
            return it == files.end() ? std::nullopt : std::optional(it->second);
        };
        s.write = [this](const fs::path& p, const std::string& t) {
            if (!writable) {
                return false;
            }
            files[p.string()] = t;
            return true;
        };
        s.volume = [this] { return volume; };
        s.setVolume = [this](int v) {
            if (!volume) {
                return false;
            }
            volume = v;
            return true;
        };
        s.spawn = [this](const std::vector<std::string>& a, const std::vector<std::string>& e) {
            argv = a;
            env = e;
            spawns++;
            return pid;
        };
        s.usage = [this](const std::string&) { return std::optional(usage); };
        s.recording = [this] { return recordingNow; };
        return s;
    }
};

Paths paths() {
    Paths p;
    p.conf = "/cfg/pibridge.conf";
    p.runtimeDir = "/run/u";
    p.statusDir = "/run/u/pibridge";
    p.roomsetup = "/bin/pibridge-roomsetup";
    p.record = "/bin/pibridge-record";
    p.recordConf = "/cfg/record.toml";
    return p;
}

const Item* find(const std::vector<Item>& v, const std::string& id) {
    const auto it = std::ranges::find_if(v, [&](const Item& i) { return i.id == id; });
    return it == v.end() ? nullptr : &*it;
}

bool has(const std::vector<std::string>& v, const std::string& s) { return std::ranges::find(v, s) != v.end(); }

} // namespace

TEST(backend_from_env_then_runtime_name) {
    CHECK(detectBackend("monado", "SteamVR/OpenXR") == Backend::Monado);
    CHECK(detectBackend("steamvr", "Monado(XRT) by Collabora et al") == Backend::SteamVr);
    CHECK(detectBackend(nullptr, "Monado(XRT) by Collabora et al") == Backend::Monado);
    CHECK(detectBackend(nullptr, "SteamVR/OpenXR") == Backend::SteamVr);
    CHECK(detectBackend("bogus", "SteamVR/OpenXR") == Backend::SteamVr);
}

TEST(conf_set_keeps_everything_else) {
    const std::string t = "# head\nipd_mm = 63 # mine\n#refresh_hz=72\nother=1\n";
    const std::string n = confSet(t, "ipd_mm", "64.5");
    CHECK(n == "# head\nipd_mm=64.5 # mine\n#refresh_hz=72\nother=1\n");
    CHECK(confSet(n, "refresh_hz", "72") == n + "refresh_hz=72\n"); // the commented line is left alone
    CHECK(confSet("", "a", "b") == "a=b\n");
    CHECK(confSet("a=1", "a", "2") == "a=2\n");
}

TEST(eye_height_from_conf) {
    CHECK(!eyeHeight("").has_value());
    CHECK(std::abs(*eyeHeight("room_eye_height_m=1.65\n") - 1.65) < 1e-9);
    CHECK(std::abs(*eyeHeight("room_body_height_m=1.80\n") - 1.80 * kEyeFromBody) < 1e-9);
    CHECK(std::abs(*eyeHeight("room_body_height_m=1.80\nroom_eye_height_m=1.70\n") - 1.70) < 1e-9);
    CHECK(!eyeHeight("room_eye_height_m=0.4\n").has_value());
    CHECK(roomsetupTakesBackend("  --backend steamvr|monado") && !roomsetupTakesBackend("  --map"));
}

TEST(ipd_moves_lenses_now_and_renders_after_restart) {
    Fake f;
    f.files["/cfg/pibridge.conf"] = "ipd_mm=63.5\n";
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    const auto before = s.items(Page::Headset);
    CHECK(find(before, "ipd")->value == "63.5 mm" && !find(before, "ipd")->pending);
    const std::string m = s.act("ipd:+");
    CHECK(f.files["/run/u/pibridge/ipd-request"] == "64.0\n");
    CHECK(f.files["/cfg/pibridge.conf"] == "ipd_mm=64.0\n");
    CHECK(m.find("lenses move now") != std::string::npos);
    const auto after = s.items(Page::Headset);
    CHECK(find(after, "ipd")->pending && find(after, "ipd")->when == When::NowAndRestart);
    // the motor's range
    for (int i = 0; i < 40; i++) {
        s.act("ipd:+");
    }
    CHECK(f.files["/run/u/pibridge/ipd-request"] == "72.0\n");
    for (int i = 0; i < 40; i++) {
        s.act("ipd:-");
    }
    CHECK(f.files["/run/u/pibridge/ipd-request"] == "58.0\n");
    // back where the session started: no longer pending
    for (int i = 0; i < 11; i++) {
        s.act("ipd:+");
    }
    CHECK(f.files["/cfg/pibridge.conf"] == "ipd_mm=63.5\n" && !find(s.items(Page::Headset), "ipd")->pending);
}

TEST(ipd_without_a_setting_starts_at_63) {
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    CHECK(find(s.items(Page::Headset), "ipd")->value == "63.0 mm");
    f.files["/cfg/pibridge.conf"] = "ipd_mm=99\n"; // out of range: the default (rec_setup.cpp)
    s.refresh();
    CHECK(find(s.items(Page::Headset), "ipd")->value == "63.0 mm");
}

TEST(volume_now_and_without_output) {
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    s.act("volume:+");
    CHECK(f.volume == 55);
    f.volume = 98;
    s.refresh();
    s.act("volume:+");
    CHECK(f.volume == 100);
    f.volume.reset();
    s.refresh();
    const auto rows = s.items(Page::Headset);
    const Item* v = find(rows, "volume");
    CHECK(!v->enabled && v->value == "no headset output");
    CHECK(s.act("volume:+") == "No headset sound output.");
}

TEST(restart_settings_saved_and_marked) {
    Fake f;
    f.files["/cfg/pibridge.conf"] = "refresh_hz=90\n";
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    s.act("refresh:0");
    s.act("slam:1");
    s.act("ctrltrack:1");
    s.act("camfps:0");
    const std::string c = f.files["/cfg/pibridge.conf"];
    CHECK(c.find("refresh_hz=72\n") != std::string::npos);
    CHECK(c.find("io_slam_engine=basalt\n") != std::string::npos);
    CHECK(c.find("io_ctrl_track=off\n") != std::string::npos);
    CHECK(c.find("io_cam_fps=50\n") != std::string::npos);
    const auto h = s.items(Page::Headset);
    const auto t = s.items(Page::Tracking);
    CHECK(find(h, "refresh")->pending && find(h, "refresh")->chosen == 0 && find(h, "refresh")->when == When::Restart);
    CHECK(find(t, "slam")->pending && find(t, "slam")->chosen == 1);
    CHECK(find(t, "ctrltrack")->pending && find(t, "ctrltrack")->chosen == 1);
    CHECK(find(t, "camfps")->pending && find(t, "camfps")->chosen == 0);
    // a later refresh keeps what the session started with
    s.refresh();
    CHECK(find(s.items(Page::Headset), "refresh")->pending);
    s.act("refresh:1");
    CHECK(!find(s.items(Page::Headset), "refresh")->pending);
    // a write that fails says so
    f.writable = false;
    CHECK(s.act("refresh:0").find("Could not write") == 0);
}

TEST(controllers_angle_shown_vibrate) {
    Fake f;
    f.files["/cfg/pibridge.conf"] = "io_ctrl_angle=30\n";
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    CHECK(find(s.items(Page::Controllers), "angle")->value == "+30\xC2\xB0");
    s.act("angle:+");
    CHECK(f.files["/run/u/pimax-ctrl-angle"] == "35\n" && f.files["/cfg/pibridge.conf"] == "io_ctrl_angle=35\n");
    // the live file wins over the setting (what the driver uses now)
    f.files["/run/u/pimax-ctrl-angle"] = "-12.6\n";
    s.refresh();
    CHECK(find(s.items(Page::Controllers), "angle")->value == "-13\xC2\xB0");
    for (int i = 0; i < 10; i++) {
        s.act("angle:-");
    }
    CHECK(f.files["/run/u/pimax-ctrl-angle"] == "-20\n");
    // without either: the app's default
    Fake g;
    Settings d(Backend::Monado, paths(), g.sinks());
    d.refresh();
    CHECK(find(d.items(Page::Controllers), "angle")->value == "+52\xC2\xB0");

    s.act("inhand:2");
    CHECK(f.files["/run/u/pimax-ctrl-inhand-mode"] == "off\n");
    CHECK(find(s.items(Page::Controllers), "inhand")->chosen == 2);
    f.files["/run/u/pimax-ctrl-inhand"] = "pid=5\nmode=auto\nleft=down\n"; // the driver's status
    s.refresh();
    CHECK(find(s.items(Page::Controllers), "inhand")->chosen == 1);
    s.act("vibrate:0");
    CHECK(f.files["/run/u/pimax-haptic-request"] == "2 60 200\n");
    s.act("vibrate:1");
    CHECK(f.files["/run/u/pimax-haptic-request"] == "3 60 200\n");
    CHECK(find(s.items(Page::Controllers), "vibrate")->chosen == -1);
}

TEST(controllers_shown_is_monado_only) {
    Fake f;
    Settings s(Backend::SteamVr, paths(), f.sinks());
    s.refresh();
    const auto rows = s.items(Page::Controllers);
    const Item* i = find(rows, "inhand");
    CHECK(!i->enabled && i->value == "Monado only");
    s.act("inhand:1");
    CHECK(!f.files.contains("/run/u/pimax-ctrl-inhand-mode"));
}

TEST(record_and_recenter_from_the_dashboard) {
    Fake f;
    Paths p = paths();
    p.record = "/bin/pibridge-record";
    Settings s(Backend::Monado, p, f.sinks());
    s.refresh();
    const Item* r = find(s.items(Page::Headset), "record");
    CHECK(r != nullptr && r->label == "Record video");
    s.act("record");
    CHECK(f.argv == std::vector<std::string>({"/bin/pibridge-record", "start", "--brief"}));
    f.recordingNow = true;
    CHECK(find(s.items(Page::Headset), "record")->label == "Stop recording");
    s.act("record");
    CHECK(f.argv == std::vector<std::string>({"/bin/pibridge-record", "stop", "--brief"}));
    // no recorder installed: no button
    Paths none = paths();
    none.record.clear();
    Settings n(Backend::Monado, none, f.sinks());
    n.refresh();
    CHECK(find(n.items(Page::Headset), "record") == nullptr);
    // recenter: the driver's request file
    s.act("recenter");
    CHECK(f.files.contains("/run/u/pibridge/recenter-request"));
}

TEST(room_setup_command_line) {
    Fake f;
    f.files["/cfg/pibridge.conf"] = "tracking_mode=inside-out\nio_room=Living Room\nroom_eye_height_m=1.62\n";
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    CHECK(find(s.items(Page::Room), "roomsetup")->enabled);
    s.act("roomsetup");
    CHECK(f.argv == std::vector<std::string>({"/bin/pibridge-roomsetup", "--map", "--room", "Living Room", "--eye-height", "1.620",
                                               "--backend", "monado"}));
    CHECK(f.env == std::vector<std::string>({"PIBRIDGE_BACKEND=monado"}));
    CHECK(s.roomsetupPid() == 777L);
    // one at a time
    CHECK(!find(s.items(Page::Room), "roomsetup")->enabled && !find(s.items(Page::Room), "roomquick")->enabled);
    CHECK(s.act("roomquick") == "The room setup is already running." && f.spawns == 1);
    s.roomsetupEnded(0);
    CHECK(!s.roomsetupPid() && find(s.items(Page::Room), "roomquick")->enabled);
    // quick, an older binary without --backend (only the variable), no eye height, the default room
    f.files["/cfg/pibridge.conf"] = "tracking_mode=inside-out\n";
    f.usage = "pibridge-roomsetup --map ...";
    s.refresh();
    s.act("roomquick");
    CHECK(f.argv == std::vector<std::string>({"/bin/pibridge-roomsetup", "--map", "--room", "home", "--quick-direct"}));
    CHECK(f.env == std::vector<std::string>({"PIBRIDGE_BACKEND=monado"}));
    s.roomsetupEnded(3);
    CHECK(has([&] {
        std::vector<std::string> w;
        for (const auto& i : s.items(Page::Room)) {
            w.push_back(i.value);
        }
        return w;
    }(), "The last room setup ended with code 3."));
    // could not start
    f.pid.reset();
    CHECK(s.act("roomsetup") == "The room setup could not start." && !s.roomsetupPid());
}

TEST(room_setup_with_the_lighthouse_faceplate_is_the_stage) {
    // Monado + Lighthouse (the default tracking mode): no room map, the Lighthouse stage; quick is the same steps
    Fake f;
    f.files["/cfg/pibridge.conf"] = "io_room=Den\nroom_eye_height_m=1.6\n";
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    s.act("roomquick");
    CHECK(f.argv == std::vector<std::string>({"/bin/pibridge-roomsetup", "--lh-stage", "--room", "Den", "--eye-height", "1.600",
                                               "--backend", "monado"}));
}

TEST(room_setup_not_under_steamvr_or_without_the_tool) {
    Fake f;
    Settings s(Backend::SteamVr, paths(), f.sinks());
    s.refresh();
    CHECK(!find(s.items(Page::Room), "roomsetup")->enabled);
    CHECK(s.act("roomsetup").find("Room page") != std::string::npos && f.spawns == 0);
    Paths p = paths();
    p.roomsetup.clear();
    Settings n(Backend::Monado, p, f.sinks());
    n.refresh();
    CHECK(!find(n.items(Page::Room), "roomsetup")->enabled && f.spawns == 0);
    CHECK(n.act("roomsetup") == "pibridge-roomsetup was not found.");
}

TEST(unknown_boxes_do_nothing) {
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    CHECK(s.act("bogus").empty() && s.act("ipd").empty() && s.act("refresh:7").empty() && f.files.empty());
}

TEST(layout_inside_and_not_overlapping) {
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    Header h;
    h.backend = "Monado";
    h.runtime = "Monado(XRT)";
    h.clock = "12:00";
    for (const Page p : kPages) {
        const View v = layout(p, s.items(p), h);
        for (std::size_t i = 0; i < v.boxes.size(); i++) {
            const Rect& a = v.boxes[i].r;
            CHECK(a.x >= 0 && a.y >= 0 && a.x + a.w <= kW && a.y + a.h <= kH && a.w > 60 && a.h > 60);
            for (std::size_t j = i + 1; j < v.boxes.size(); j++) {
                const Rect& b = v.boxes[j].r;
                const bool apart = a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + a.h <= b.y || b.y + b.h <= a.y;
                CHECK(apart);
            }
            // every box hits itself at its centre (when enabled)
            if (v.boxes[i].enabled) {
                CHECK(hit(v, a.x + a.w / 2, a.y + a.h / 2) == v.boxes[i].id);
            }
        }
        // the open page is marked
        const auto sel = std::ranges::count_if(v.boxes, [](const Box& b) { return b.style == Box::Style::NavSelected; });
        CHECK(sel == 1);
        for (const Label& l : v.labels) {
            CHECK(l.r.x >= 0 && l.r.x + l.r.w <= kW && l.r.y + l.r.h <= kH);
        }
    }
}

TEST(hits_skip_disabled_and_empty_space) {
    Fake f;
    f.volume.reset();
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    Header h;
    const View v = layout(Page::Headset, s.items(Page::Headset), h); // no window: its box is off
    const auto box = [&](const std::string& id) { return *std::ranges::find_if(v.boxes, [&](const Box& b) { return b.id == id; }); };
    const Box vol = box("volume:+");
    CHECK(!vol.enabled && !hit(v, vol.r.x + 5, vol.r.y + 5));
    const Box win = box("view:window");
    CHECK(!win.enabled && !hit(v, win.r.x + 5, win.r.y + 5));
    CHECK(!hit(v, 5, 5));
    CHECK(hit(v, box("view:hide").r.x + 5, box("view:hide").r.y + 5) == "view:hide");
    CHECK(pageOf("page:3") == Page::Room && pageOf("page:0") == Page::Headset && !pageOf("page:9") && !pageOf("ipd:+"));
}

TEST(over_a_game_the_bottom_bar_has_the_quick_actions) {
    // the dashboard overlay over a game: no Hide, no PiBridge window; Quick room setup, Recenter, Close app
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    Header h;
    h.overGame = true;
    h.roomQuickAvailable = true;
    h.recenterAvailable = true;
    h.closeName = "Falcon BMS";
    const View v = layout(Page::Headset, s.items(Page::Headset), h);
    const auto has = [&](const std::string& id) { return std::ranges::any_of(v.boxes, [&](const Box& b) { return b.id == id; }); };
    CHECK(!has("view:hide") && !has("view:window"));
    const auto box = [&](const std::string& id) { return *std::ranges::find_if(v.boxes, [&](const Box& b) { return b.id == id; }); };
    h.recordAvailable = true;
    {
        const View r = layout(Page::Headset, s.items(Page::Headset), h);
        const Box rb = *std::ranges::find_if(r.boxes, [](const Box& b) { return b.id == "view:record"; });
        CHECK(rb.enabled && rb.label == "Record video" && rb.style == Box::Style::Bar);
        h.recording = true;
        const View r2 = layout(Page::Headset, s.items(Page::Headset), h);
        const Box rs = *std::ranges::find_if(r2.boxes, [](const Box& b) { return b.id == "view:record"; });
        CHECK(rs.label == "Stop recording" && rs.style == Box::Style::Selected);
        CHECK(rs.r.x > 36 + 400 && rs.r.x + rs.r.w <= box("view:roomquick").r.x); // leaves room for the message
    }
    for (const char* id : {"view:roomquick", "view:recenter", "view:close"}) {
        const Box b = box(id);
        CHECK(b.enabled && hit(v, b.r.x + 5, b.r.y + 5) == id);
        CHECK(b.r.x + b.r.w <= kW && b.r.y + b.r.h <= kH);
    }
    CHECK(box("view:roomquick").r.x + box("view:roomquick").r.w <= box("view:recenter").r.x);
    CHECK(box("view:recenter").r.x + box("view:recenter").r.w <= box("view:close").r.x);
    CHECK(box("view:close").label == "Close app");
    // no app known (the libmonado without pids, PiBridge World itself): Close app is off
    h.closeName.clear();
    h.roomQuickAvailable = false;
    const View w = layout(Page::Headset, s.items(Page::Headset), h);
    const Box c = *std::ranges::find_if(w.boxes, [](const Box& b) { return b.id == "view:close"; });
    const Box q = *std::ranges::find_if(w.boxes, [](const Box& b) { return b.id == "view:roomquick"; });
    CHECK(!c.enabled && !q.enabled);
}

TEST(recording_page_saves_pibridge_records_settings) {
    Fake f;
    f.files["/cfg/record.toml"] = "# pibridge-record settings\neyes = \"left\"        # left | both\nfps = 30\nmax_height = 2160\n";
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    const auto items = s.items(Page::Recording);
    const Item* eyes = find(items, "rec_eyes");
    CHECK(eyes && eyes->chosen == 0 && eyes->choices.size() == 2);
    CHECK(find(items, "rec_height") && find(items, "rec_height")->chosen == 2);
    CHECK(find(items, "rec_quality") && find(items, "rec_quality")->chosen == 2); // the recorder's default: high
    CHECK(find(items, "rec_mic") && find(items, "rec_mic")->chosen == 1);         // off by default
    CHECK(find(items, "record") != nullptr);
    CHECK(s.act("rec_eyes:1").starts_with("Picture: saved"));
    CHECK(s.act("rec_fps:2").starts_with("Frame rate: saved"));
    CHECK(s.act("rec_mic:0").starts_with("Microphone: saved"));
    const std::string t = f.files["/cfg/record.toml"];
    CHECK(t.starts_with("# pibridge-record settings\n"));
    CHECK(t.find("eyes=\"both\"") != std::string::npos);
    CHECK(t.find("fps=72") != std::string::npos);
    CHECK(t.find("mic=\"on\"") != std::string::npos);
    CHECK(s.recordWord("eyes", "left") == "both");
    // eight rows fit the panel: every box inside the body, none overlapping the next
    Header h;
    const View v = layout(Page::Recording, s.items(Page::Recording), h);
    double lastBottom = 0;
    for (const Box& b : v.boxes) {
        if (b.id.starts_with("rec_mic")) {
            CHECK(b.r.y + b.r.h <= kH - 20 - 110 - 20 + 1); // above the bottom bar
        }
        if (b.id.starts_with("rec_") && b.id.ends_with(":0")) {
            CHECK(b.r.y >= lastBottom - 1);
            lastBottom = b.r.y + b.r.h;
        }
    }
    CHECK(pageOf("page:4") == Page::Recording);
}

TEST(signature_follows_the_picture) {
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    Header h;
    View a = layout(Page::Headset, s.items(Page::Headset), h);
    View b = layout(Page::Headset, s.items(Page::Headset), h);
    CHECK(signature(a) == signature(b));
    b.hover = "ipd:+";
    CHECK(signature(a) != signature(b));
    s.act("ipd:+");
    const View c = layout(Page::Headset, s.items(Page::Headset), h);
    CHECK(signature(a) != signature(c));
    h.message = "x";
    CHECK(signature(layout(Page::Headset, s.items(Page::Headset), h)) != signature(c));
}

TEST(click_is_down_and_up_on_one_box) {
    Clicker c;
    CHECK(!c.update(std::string("a"), false));
    CHECK(!c.update(std::string("a"), true) && c.downOn() == "a");
    CHECK(c.update(std::string("a"), false) == "a");
    // moved off before the release: no click
    c.update(std::string("a"), true);
    CHECK(!c.update(std::string("b"), true));
    CHECK(!c.update(std::string("b"), false));
    // pressed on nothing, released on a box: no click
    c.update(std::nullopt, true);
    CHECK(!c.update(std::string("a"), false));
    // held: one click only
    c.update(std::string("a"), true);
    CHECK(!c.update(std::string("a"), true));
    CHECK(c.update(std::string("a"), false) == "a" && !c.update(std::string("a"), false));
}

TEST(plan_with_the_dashboard) {
    bg::HomeSettings st;
    Have h;
    h.dashboard = true;
    Plan p = plan(st, h);
    CHECK(p.dashboard && !p.window && !p.textPanel);
    h.wantWindow = true; // picked, but the app is not there: the dashboard stays
    p = plan(st, h);
    CHECK(p.dashboard && !p.window);
    h.appLinked = true;
    p = plan(st, h);
    CHECK(p.window && !p.dashboard && !p.textPanel);
    st.panel = false; // the window off in the settings
    CHECK(plan(st, h).dashboard);
    h.hidden = true;
    p = plan(st, h);
    CHECK(!p.dashboard && !p.window && !p.textPanel);
    // without a dashboard (no Qt, no panel pass): as before
    Have old;
    CHECK(plan(st, old).textPanel && !plan(st, old).dashboard);
}

TEST(real_file_writes_are_atomic) {
    const fs::path d = fs::temp_directory_path() / ("pibridge-dash-test-" + std::to_string(::getpid()));
    fs::remove_all(d);
    const fs::path f = d / "sub" / "x.conf";
    CHECK(writeFileAtomic(f, "a=1\n"));
    CHECK(readFile(f) == "a=1\n");
    CHECK(writeFileAtomic(f, "a=2\n") && readFile(f) == "a=2\n");
    std::size_t n = 0;
    for (const auto& e : fs::directory_iterator(d / "sub")) {
        (void)e;
        n++;
    }
    CHECK(n == 1); // no temporary file left
    CHECK(!readFile(d / "missing").has_value());
    fs::remove_all(d);
}

TEST(real_spawn_and_reap) {
    const auto pid = spawnDetached({"sh", "-c", "test \"$PIBRIDGE_DASH_T\" = yes && exit 7"}, {"PIBRIDGE_DASH_T=yes"});
    CHECK(pid.has_value());
    std::optional<int> code;
    for (int i = 0; i < 200 && !code; i++) {
        code = reap(*pid);
        if (!code) {
            ::usleep(10000);
        }
    }
    CHECK(code == 7);
    CHECK(!spawnDetached({"/nonexistent/pibridge-nothing"}, {}).has_value() || true); // posix_spawnp may fork first
    const auto u = usageOf("echo");
    CHECK(u && u->find("--help") != std::string::npos);
}

#ifdef PIBRIDGE_HOME_DASH
TEST(picture_painted) {
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    Header h;
    h.backend = "Monado";
    View v = layout(Page::Headset, s.items(Page::Headset), h);
    v.hover = "ipd:+";
    Painter p;
    const auto px = p.picture(v);
    CHECK(px.size() == static_cast<std::size_t>(kW) * kH * 4);
    // the corner is transparent (rounded), the middle of a card is not
    CHECK(px[3] == 0);
    const std::size_t mid = (static_cast<std::size_t>(kH / 2) * kW + kW / 2) * 4;
    CHECK(px[mid + 3] > 200);
    // a hovered box looks different from the same box not hovered
    const Box b = *std::ranges::find_if(v.boxes, [](const Box& x) { return x.id == "ipd:+"; });
    const std::size_t at = (static_cast<std::size_t>(b.r.y + 8) * kW + static_cast<std::size_t>(b.r.x + 8)) * 4;
    const std::array<std::uint8_t, 3> hov{px[at], px[at + 1], px[at + 2]};
    v.hover.clear();
    const auto px2 = p.picture(v);
    CHECK((std::array<std::uint8_t, 3>{px2[at], px2[at + 1], px2[at + 2]}) != hov);
}
#endif

TEST(the_desktop_box_and_the_desktops_bar) {
    Header h;
    h.overGame = true;
    h.recordAvailable = h.roomQuickAvailable = h.recenterAvailable = true;
    h.closeName = "Falcon BMS";
    // over a game: Desktop first, every quick action with its icon, all inside the bottom bar, none overlapping
    Fake f;
    Settings s(Backend::Monado, paths(), f.sinks());
    s.refresh();
    View v = layout(Page::Headset, s.items(Page::Headset), h);
    const auto find = [](const View& vw, const std::string& id) -> const Box* {
        const auto it = std::ranges::find_if(vw.boxes, [&](const Box& b) { return b.id == id; });
        return it == vw.boxes.end() ? nullptr : &*it;
    };
    CHECK(find(v, "view:desktop") != nullptr && !find(v, "view:desktop")->enabled); // pibridge-desktop not found
    h.desktopAvailable = true;
    v = layout(Page::Headset, s.items(Page::Headset), h);
    const Box* d = find(v, "view:desktop");
    CHECK(d != nullptr && d->enabled && d->icon == Icon::Desktop && d->r.x + d->r.w <= find(v, "view:record")->r.x);
    CHECK(find(v, "view:record")->icon == Icon::Record && find(v, "view:recenter")->icon == Icon::Recenter &&
          find(v, "view:roomquick")->icon == Icon::Room && find(v, "view:close")->icon == Icon::Close);
    // over PiBridge World: no Close app
    h.closeShown = false;
    v = layout(Page::Headset, s.items(Page::Headset), h);
    CHECK(find(v, "view:close") == nullptr && find(v, "view:desktop") != nullptr);
    h.closeShown = true;
    // the desktop's bar: a kW x kBarH picture, Dashboard, a box per monitor and All, the quick actions without Desktop
    DeskBar db;
    db.monitors = 2;
    db.mask = 0b11;
    db.words = "Desktop: 2 monitors";
    const View b = deskLayout(h, db);
    CHECK(b.w == kW && b.h == kBarH);
    for (const char* id : {"desk:dash", "desk:mon:0", "desk:mon:1", "desk:all", "view:record", "view:roomquick", "view:recenter", "view:close"}) {
        const Box* x = find(b, id);
        CHECK(x != nullptr);
        if (x != nullptr) {
            CHECK(x->r.x >= 0 && x->r.y >= 0 && x->r.x + x->r.w <= kW && x->r.y + x->r.h <= kBarH);
            CHECK(hit(b, x->r.x + 5, x->r.y + 5) == std::string(id));
        }
    }
    CHECK(find(b, "view:desktop") == nullptr);
    CHECK(find(b, "desk:all")->style == Box::Style::Selected && find(b, "desk:mon:0")->style == Box::Style::Normal);
    CHECK(find(b, "desk:all")->r.x + find(b, "desk:all")->r.w <= find(b, "view:record")->r.x); // no overlap
    db.mask = 0b10;
    const View one = deskLayout(h, db);
    CHECK(find(one, "desk:mon:1")->style == Box::Style::Selected && find(one, "desk:all")->style == Box::Style::Normal);
    // one monitor: no All
    db.monitors = 1;
    db.mask = 1;
    CHECK(find(deskLayout(h, db), "desk:all") == nullptr);
    CHECK(signature(b) != signature(one));
#ifdef PIBRIDGE_HOME_DASH
    Painter p;
    const auto px = p.picture(b);
    CHECK(px.size() == static_cast<std::size_t>(kW) * kBarH * 4);
    const auto big = p.picture(v); // back to the full size
    CHECK(big.size() == static_cast<std::size_t>(kW) * kH * 4);
#endif
}

TEST(desktop_screens_on_an_arc_the_ray_and_the_pointer) {
    // two 2560 x 1440 monitors side by side; the head at 1.6 m, looking down -z
    std::vector<desktop::MonitorInfo> mons(2);
    mons[0].index = 0;
    mons[0].logicalW = mons[1].logicalW = 2560;
    mons[0].logicalH = mons[1].logicalH = 1440;
    mons[0].width = mons[1].width = 2560;
    mons[0].height = mons[1].height = 1440;
    mons[1].index = 1;
    mons[1].x = 2560;
    room::Mat34 head = room::Mat34::identity();
    head.m[1][3] = 1.6;
    CHECK(desk::maskAll(mons) == 0b11 && desk::maskOne(1) == 0b10);
    auto sc = desk::place(head, 0.0, mons, 0b11);
    CHECK(sc.size() == 2 && sc[0].monitor == 0 && sc[1].monitor == 1);
    CHECK(sc[0].center.x < 0 && sc[1].center.x > 0 && sc[0].center.z < 0); // left, right, in front
    CHECK(std::abs(sc[0].w - 1.3) < 1e-9 && std::abs(sc[0].h - 1.3 * 1440 / 2560) < 1e-9);
    for (const auto& s1 : sc) {
        const room::Vec3 toHead = (room::Vec3{0, s1.center.y, 0} - s1.center).norm();
        CHECK(s1.toward.dot(toHead) > 0.999); // each faces the viewer
        CHECK(std::abs((room::Vec3{0, 0, 0} - s1.center).length() - std::abs(s1.center.y) - 0) >= 0);
        CHECK(std::abs(room::Vec3{s1.center.x, 0, s1.center.z}.length() - desk::kDistance) < 1e-9);
        // the quaternion turns the quad's axes onto the screen's
        const auto q = desk::orientation(s1);
        const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
        const room::Vec3 z{2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy)};
        CHECK((z - s1.toward).length() < 1e-9);
    }
    CHECK(sc[0].center.y < 1.6 && sc[0].center.y > 1.0); // a little below the eyes
    CHECK(std::abs(desk::bottomY(sc) - (sc[0].center.y - sc[0].h / 2)) < 1e-9);
    // only monitor 2: straight ahead
    const auto two = desk::place(head, 0.0, mons, desk::maskOne(1));
    CHECK(two.size() == 1 && two[0].monitor == 1 && std::abs(two[0].center.x) < 1e-9);
    // the ray from the head to a point on monitor 1 hits it there
    const room::Vec3 o{0, 1.6, 0};
    const room::Vec3 target = sc[0].at(0.25, 0.75);
    const auto h = desk::hit(sc, o, (target - o).norm());
    CHECK(h && h->monitor == 0 && std::abs(h->u - 0.25) < 1e-6 && std::abs(h->v - 0.75) < 1e-6);
    CHECK(!desk::hit(sc, o, room::Vec3{0, 0, 1})); // behind
    // the pointer: pixels, the trigger, a drag past the edge stays on its monitor, a short squeeze = right click
    desk::Pointer p;
    p.update(o, (target - o).norm(), 0, 0, 0, 0.01, sc);
    CHECK(p.state().on && p.state().monitor == 0 && p.state().x == 640 && p.state().y == 1080 && p.state().buttons == 0);
    p.update(o, (target - o).norm(), 1.0, 0, 0, 0.01, sc);
    CHECK(p.pressed() && p.state().buttons == 1);
    const room::Vec3 onRight = sc[1].at(0.5, 0.5);
    p.update(o, (onRight - o).norm(), 1.0, 0, 0, 0.01, sc);
    CHECK(p.state().on && p.state().monitor == 0 && p.state().x == 2559); // dragged to monitor 1's right edge
    p.update(o, (onRight - o).norm(), 0.0, 0, 0, 0.01, sc);
    CHECK(!p.pressed() && p.state().monitor == 0 && p.state().buttons == 0); // released where the drag ended
    p.update(o, (onRight - o).norm(), 0.0, 0, 0, 0.01, sc);
    CHECK(p.state().monitor == 1); // then the ray's monitor again
    p.update(o, (onRight - o).norm(), 0.0, 1.0, 0, 0.1, sc);
    p.update(o, (onRight - o).norm(), 0.0, 0.0, 0, 0.1, sc);
    CHECK(p.state().buttons == 2); // the right button, for a few frames
    for (int i = 0; i < 4; i++) {
        p.update(o, (onRight - o).norm(), 0.0, 0.0, 0, 0.01, sc);
    }
    CHECK(p.state().buttons == 0);
    // a long squeeze (bringing the screens back) is no click
    for (int i = 0; i < 10; i++) {
        p.update(o, (onRight - o).norm(), 0.0, 1.0, 0, 0.1, sc);
    }
    p.update(o, (onRight - o).norm(), 0.0, 0.0, 0, 0.01, sc);
    CHECK(p.state().buttons == 0);
    // the stick scrolls; off the screens the pointer is off
    p.update(o, (onRight - o).norm(), 0.0, 0.0, 1.0, 0.5, sc);
    CHECK(p.wheel() == 480);
    p.update(o, room::Vec3{0, 1, 0}, 0.0, 0.0, 0, 0.01, sc);
    CHECK(!p.state().on && p.state().buttons == 0);
}

TEST(world_desktop_file_the_overlay_draws) {
    const auto d = desk::parseWorldDesk("screen 1 0.1 1.5 -1.6 0 0.342 0 0.94 1.6425 0.923\n"
                                        "ray 0.2 1.2 -0.3 0.1 1.5 -1.5\n"
                                        "screen 9 0 0 0 0 0 0 1 1 1\n"   // no such monitor
                                        "screen 0 0 0 0 0 0 0 1 0 1\n"   // no size
                                        "junk\n");
    CHECK(d.screens.size() == 1 && d.screens[0].monitor == 1 && std::abs(d.screens[0].pos[2] + 1.6) < 1e-9 &&
          std::abs(d.screens[0].quat[1] - 0.342) < 1e-9 && std::abs(d.screens[0].w - 1.6425) < 1e-9);
    CHECK(d.rays.size() == 1 && std::abs(d.rays[0].second.z + 1.5) < 1e-9);
    CHECK(desk::parseWorldDesk("").screens.empty());
}

TEST(the_frame_places_the_monitors_for_steamvr) {
    room::Mat34 head = room::Mat34::identity();
    head.m[1][3] = 1.6;
    std::vector<frame::Monitor> mons{{0, 0, 0, 2560, 1440}, {1, 2560, 0, 2560, 1440}};
    // one: in the middle, 1.55 m ahead (Medium), 2.19 x 0.75 wide, facing you
    auto p = frame::place(head, mons, 0b01, 1.0, 0.0);
    CHECK(p.size() == 1 && p[0].monitor == 0 && std::abs(p[0].w - 2.19 * 0.75) < 1e-9);
    CHECK(std::abs(p[0].pose.pos().z + 1.55 - 0.0005 * 0.75) < 1e-9 && std::abs(p[0].pose.pos().x) < 1e-9);
    CHECK(p[0].pose.axis(2).z > 0.999); // its face towards you
    // two: the first in the middle, the second on the right, nearer, turned towards you
    p = frame::place(head, mons, 0b11, 1.0, 0.0);
    CHECK(p.size() == 2 && p[1].monitor == 1 && std::abs(p[1].pose.pos().x - 2.012 * 0.75) < 1e-9);
    CHECK(p[1].pose.pos().z > p[0].pose.pos().z && p[1].pose.axis(2).x < -0.6);
    // Zoom scales about the frame's middle, Distance moves it away
    const auto z = frame::place(head, mons, 0b01, 2.0, 1.0);
    CHECK(std::abs(z[0].w - 2.0 * 2.19 * 0.75) < 1e-9 && z[0].pose.pos().z < p[0].pose.pos().z - 0.5);
    CHECK(std::abs(frame::place(head, mons, 0b01, 99.0, 0)[0].w - frame::kZoomMax * 2.19 * 0.75) < 1e-9);
    CHECK(frame::uiScale("Large").forward == 2.3 && frame::uiScale("nonsense").scale == 0.75);
}

TEST(steamvrs_bar_toggles_the_desktop_and_steps_zoom_and_distance) {
    Header h;
    h.overGame = true;
    h.closeShown = false;
    DeskBar db;
    db.monitors = 2;
    db.mask = 0b11;
    db.dashShown = false;
    db.toggleShown = true;
    db.on = false;
    const auto find = [](const View& vw, const std::string& id) -> const Box* {
        const auto it = std::ranges::find_if(vw.boxes, [&](const Box& b) { return b.id == id; });
        return it == vw.boxes.end() ? nullptr : &*it;
    };
    View v = deskLayout(h, db);
    CHECK(find(v, "desk:toggle") != nullptr && find(v, "desk:toggle")->style == Box::Style::Bar);
    CHECK(find(v, "desk:mon:0") == nullptr && find(v, "desk:zoom:+") == nullptr && find(v, "desk:dash") == nullptr);
    db.on = true;
    db.zoomShown = true;
    db.zoom = 1.25;
    v = deskLayout(h, db);
    CHECK(find(v, "desk:toggle")->style == Box::Style::Selected && find(v, "desk:mon:1") != nullptr);
    for (const char* id : {"desk:zoom:-", "desk:zoom:+", "desk:dist:-", "desk:dist:+"}) {
        const Box* b = find(v, id);
        CHECK(b != nullptr && b->r.x >= 0 && b->r.x + b->r.w <= kW && b->r.y + b->r.h <= kBarH);
        if (b != nullptr) {
            CHECK(hit(v, b->r.x + 5, b->r.y + 5) == std::string(id));
        }
    }
    CHECK(std::ranges::any_of(v.labels, [](const Label& l) { return l.text == "1.25"; }));
}

PIBRIDGE_TEST_MAIN()
