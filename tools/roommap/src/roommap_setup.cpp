// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/setupcmd.rs (MIT OR Apache-2.0): `pibridge-roommap setup fast|full|show`
// and `pibridge-roommap auto`. The Rust tool talks to the user in the headset through omarchyvr-hud (an OpenXR overlay
// panel); PiBridge's in-headset UI lives in tools/roomsetup and the app (another part), so this port runs the same steps
// without a panel: every prompt is printed (for the app's output) and the head comes from Monado's head ring (forward
// by holding still works; triggers and the traced boundary need a panel's input, i.e. setup full needs the app).
#include "roommap_cli.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/roommap/floor.h"
#include "pibridge/roommap/guide_ipc.h"
#include "pibridge/roommap/light.h"
#include "pibridge/roommap/live.h"
#include "pibridge/roommap/pipeline.h"
#include "pibridge/roommap/setup.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <numeric>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

namespace pibridge::roommap::cli {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace rs = anchor::rs;
using session::Alignment;
using Clock = std::chrono::steady_clock;

namespace {

const char* SETUP_HELP =
    "pibridge-roommap setup fast|full [--name NAME] [--label TEXT] [--no-hud] [--map-seconds S]\n"
    "    guided room setup (docs/ROOM-SETUP.md): fast = look around in the middle (~30 s), automatic floor\n"
    "    and play area, forward by trigger or holding still; full = map walk (60 s), floor touch, traced boundary, forward.\n"
    "    Saves NAME/room-setup.toml and puts the room in place (needs Monado with 6DoF and the anchor hook).\n"
    "pibridge-roommap setup show [--name NAME]\n"
    "pibridge-roommap auto [--name NAME] [--wait S] [--status FILE]\n"
    "    at a session start: wait for Monado's head ring, recognise the room (retries for --wait s, default 120),\n"
    "    apply floor + centre + forward; state in $XDG_RUNTIME_DIR/omarchyvr-room.status";

double secs(Clock::time_point a) {
    return std::chrono::duration<double>(Clock::now() - a).count();
}

std::string runtime_file(const char* name) {
    const std::string d = cam::env_os("XDG_RUNTIME_DIR").value_or("/tmp");
    return cam::path_join(d, name);
}

std::string status_path() {
    return runtime_file("omarchyvr-room.status");
}

std::string stage_bounds_path() {
    if (auto v = cam::env_os("PIMAX_STAGE_BOUNDS_FILE"); v && !v->empty()) {
        return *v;
    }
    return runtime_file("pimax-stage-bounds");
}

void write_status(const std::string& path, const std::string& state, const std::string& room, std::uint32_t pid,
                  const std::string& text) {
    std::string clean = text;
    for (auto& c : clean) {
        if (c == '"' || c == '\n') {
            c = ' ';
        }
    }
    const std::string t = "# omarchyvr-roommap auto / setup (docs/ROOM-SETUP.md)\nstate = \"" + state + "\"\nroom = \"" +
                          room + "\"\npid = " + std::to_string(pid) + "\ntext = \"" + clean + "\"\nwhen = " +
                          std::to_string(unix_now()) + "\n";
    const std::string tmp = rs::with_extension(path, "tmp");
    if (!cam::write_file(tmp, t) || std::rename(tmp.c_str(), path.c_str()) != 0) {
        std::fprintf(stderr, "cannot write %s\n", path.c_str());
    }
    std::printf("room: %s\n", text.c_str());
}

bool tag_holds(std::uint32_t pid) {
    const auto l = anchor::read_lease(anchor::lease_path());
    const auto o = anchor::read_offset(anchor::offset_path());
    return l && l->kind == "anchor" && l->session == pid && o && o->first == pid;
}

bool alive(std::uint32_t pid) {
    return pid != 0 && cam::path_exists("/proc/" + std::to_string(pid));
}

std::optional<std::uint32_t> ring_pid() {
    return anchor::feed_now().live_pid();
}

std::optional<std::uint32_t> ring_owner() {
    if (auto r = anchor::HeadRing::read(anchor::ring_path())) {
        return r->pid;
    }
    return std::nullopt;
}

R<void> feed_ok(std::uint32_t pid) {
    const auto f = anchor::feed_now();
    if (const auto p = f.live_pid(); p && *p == pid) {
        return {};
    }
    return std::unexpected(f.plain() + "; nothing saved");
}

/// The guide without an in-headset panel: prompts printed (each new one; a running counter at most once a second),
/// the head from Monado's head ring in the room frame of this session's fix.
class RingGuide : public setup::Guide {
public:
    RingGuide(std::string dir, std::uint32_t pid) : dir_(std::move(dir)), pid_(pid), t0_(Clock::now()), last_t_(Clock::now()) {}

    void say(const std::string& title, const std::string& text, std::optional<float>) override {
        const std::string key = title + ": " + text;
        if (key != last_ && (secs(last_t_) >= 1.0 || !last_.starts_with(title))) {
            std::printf("room setup: %s\n", key.c_str());
            last_ = key;
            last_t_ = Clock::now();
        }
    }

    setup::Sample sample() override {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (!conv_ || secs(conv_t_) >= 0.5) {
            conv_t_ = Clock::now();
            conv_.reset();
            if (auto r = store::RelocResult::load(cam::path_join(dir_, "last-reloc.toml")); r && r->pid == pid_ && pid_ != 0) {
                conv_ = r->align.pose();
            }
        }
        setup::Sample s;
        s.t = secs(t0_);
        if (!conv_) {
            return s;
        }
        if (auto r = anchor::HeadRing::read(anchor::ring_path()); r && r->pid == pid_) {
            for (auto it = r->samples.rbegin(); it != r->samples.rend(); ++it) {
                if (it->valid) {
                    const g::Pose room = conv_->compose(it->pose);
                    const g::Vec3 f = g::mat_vec(room.r, {0.0, 0.0, -1.0});
                    s.head = std::pair{room.t, setup::heading(f[0], f[2])};
                    break;
                }
            }
        }
        return s;
    }

    template <class F>
    auto during(const std::string& title, const std::string& text, F f) {
        say(title, text, 0.0f);
        std::printf("room setup: %s: %s\n", title.c_str(), text.c_str());
        return f();
    }

    void restart_clock() { t0_ = Clock::now(); }
    std::uint32_t pid() const { return pid_; }

private:
    std::string dir_;
    std::uint32_t pid_;
    Clock::time_point t0_;
    Clock::time_point last_t_;
    std::string last_;
    std::optional<g::Pose> conv_;
    Clock::time_point conv_t_{};
};

R<bool> wait_for_wear(RingGuide& gd, double seconds) {
    const auto t0 = Clock::now();
    std::vector<std::pair<Clock::time_point, g::Pose>> poses;
    bool said = false;
    auto checked = Clock::now();
    std::optional<Clock::time_point> bad_since;
    while (secs(t0) < seconds) {
        if (secs(checked) >= 0.5) {
            checked = Clock::now();
            if (auto ok = feed_ok(gd.pid()); ok) {
                bad_since.reset();
            } else {
                if (!bad_since) {
                    bad_since = Clock::now();
                }
                if (secs(*bad_since) > 2.0) {
                    return std::unexpected(ok.error());
                }
            }
        }
        if (auto r = anchor::HeadRing::read(anchor::ring_path()); r && r->pid == gd.pid()) {
            for (auto it = r->samples.rbegin(); it != r->samples.rend(); ++it) {
                if (it->valid) {
                    poses.emplace_back(Clock::now(), it->pose);
                    break;
                }
            }
        }
        std::erase_if(poses, [](const auto& x) { return secs(x.first) >= 3.0; });
        std::vector<g::Pose> ps;
        for (const auto& x : poses) {
            ps.push_back(x.second);
        }
        if (light::moved(ps, 8.0, 0.15)) {
            return true;
        }
        if (!said) {
            gd.say("Room setup", "Put the headset on and look around: the room setup starts when you move your head.",
                   std::nullopt);
            said = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::printf("room setup: no head movement in %s s: starting anyway\n", rf::fixed(seconds, 0).c_str());
    return false;
}

std::optional<light::Advice> check_light(RingGuide& gd, const std::string& title, double seconds) {
    auto c = cam::DeviceCalib::load_default(std::nullopt);
    if (!c) {
        std::printf("light check skipped: %s\n", c.error().text().c_str());
        return std::nullopt;
    }
    const std::size_t ids[2] = {0, 1};
    auto rg = stereo::Rig::from_calib(c->first, ids);
    if (!rg) {
        std::printf("light check skipped: %s\n", rg.error().c_str());
        return std::nullopt;
    }
    const std::size_t need = pipeline::build_params(0.2, 15.0).kf_min_points;
    light::MeasureOpts o;
    o.every = 4;
    const auto t0 = Clock::now();
    for (;;) {
        auto qs = light::measure_source("broker", 2.0, o, *rg, orb::OrbParams{});
        if (!qs) {
            std::printf("light check skipped: %s\n", qs.error().c_str());
            return std::nullopt;
        }
        if (qs->empty()) {
            return std::nullopt;
        }
        const auto r = light::summarise(*qs, need);
        light::FrameQuality q;
        q.mean = r.mean;
        q.stereo = r.stereo;
        const auto a = light::advice(q, need);
        std::printf("light check: brightness %s/255 (exposure %u gain %u), %zu stereo points per view (a view needs %zu): %s\n",
                    rf::fixed(r.mean, 0).c_str(), r.exposure, r.gain, r.stereo, need, light::advice_debug(a));
        if (a != light::Advice::TooDark || secs(t0) >= seconds) {
            return a;
        }
        gd.say(title,
               std::string(light::advice_text(a)) + " (" + std::to_string(r.stereo) + " points in view, brightness " +
                   rf::fixed(r.mean, 0) + " of 255.)",
               std::nullopt);
    }
}

R<void> map_with_feedback(RingGuide& gd, const std::string& title, const std::string& text, Args args, bool append,
                          const std::function<void(const light::MapProgress&)>& also = {},
                          const std::function<bool()>* stop_early = nullptr) {
    if (append) {
        args.push_back("--append");
    } else {
        // --cover-target 0.8: the walk continues until at least ~10 of the 12 head-yaw sectors have been faced (the
        // room is actually looked at all around), not a fixed timer; the hard cap stays --seconds + --extend
        // (room-setup rework Phase 1, premature-exit fix).
        for (const auto& x : s({"--min-keyframes", "8", "--extend", "30", "--cover-target", "0.8"})) {
            args.push_back(x);
        }
    }
    auto last = Clock::now() - std::chrono::seconds(5);
    const std::function<void(const light::MapProgress&)> cb = [&](const light::MapProgress& p) {
        if (also) {
            also(p);
        }
        if (secs(last) < 0.7) {
            return;
        }
        last = Clock::now();
        std::printf("room setup: %s: %s\n", title.c_str(), light::progress_text(p).c_str());
    };
    return gd.during(title, text, [&] { return cmd_build_with(args, &cb, stop_early); });
}

void backup_map(const std::string& dir) {
    const std::string m = cam::path_join(dir, "map.bin");
    if (cam::path_exists(m)) {
        if (auto b = cam::read_file(m)) {
            (void)cam::write_file(cam::path_join(dir, "map.bin.prev"), *b);
        }
    }
}

R<store::RelocResult> save_fix_from_current(const std::string& dir, std::uint32_t pid, const std::string& what) {
    const Alignment at_mapping = live::current_offset(pid);
    const auto corr = store::last_correction(dir);
    if (corr) {
        std::printf("loop closure of the mapping walk: the tracker ended %s cm (%s cm vertically) and %s deg from the map "
                    "frame; included in the room fix\n",
                    rf::fixed(g::norm(corr->t) * 100.0, 1).c_str(), rf::fixed(std::fabs(corr->t[1]) * 100.0, 1).c_str(),
                    rf::fixed(std::fabs(rf::to_degrees(corr->yaw)), 2).c_str());
    }
    store::RelocResult r;
    r.map = dir;
    r.source = what;
    r.align = store::fix_after_mapping(at_mapping, corr);
    r.pid = pid;
    if (auto w = r.save(cam::path_join(dir, "last-reloc.toml")); !w) {
        return std::unexpected(w.error());
    }
    return r;
}

/// PiBridge: the fast setup is done standing, so a floor found in the points less than this below the head is a table
/// or a bed, not the floor (live 2026-10-19: a desk 0.76 m below the head was taken; the Rust tool's gate is 0.4 m).
inline constexpr double STANDING_MIN_BELOW_M = 1.0;

/// A points floor too close below the head for a standing user: set aside (floor.toml.rejected), then the map is
/// searched again for a layer at least STANDING_MIN_BELOW_M below the head; the controller touch follows when none.
void reject_table_floor(RingGuide& gd, const std::string& dir, const std::string& name) {
    auto f = floor::StoredFloor::load(dir);
    if (!f || f->source != floor::FloorSource::Points || f->head_above_m >= STANDING_MIN_BELOW_M) {
        return;
    }
    std::printf("floor at %s m is only %s m below the head: not the floor for a standing user (a table?)\n",
                rf::fixed(f->y, 3, true).c_str(), rf::fixed(f->head_above_m, 2).c_str());
    const std::string p = cam::path_join(dir, "floor.toml");
    std::rename(p.c_str(), (p + ".rejected").c_str());
    gd.say("Floor", "The lowest surface seen was only " + rf::fixed(f->head_above_m, 2) +
                        " m below your head (a table?). Looking for the real floor...",
           std::nullopt);
    (void)cmd_floor(s({"detect", "--save", "--name", name.c_str(), "--min-below", "1.0"}));
}

R<std::string> ensure_floor(RingGuide& gd, const std::string& dir, const std::string& name, bool confirm_touch,
                            bool standing) {
    if (!floor::StoredFloor::load(dir)) {
        (void)cmd_floor(s({"detect", "--save", "--name", name.c_str(),
                           "--min-below", standing ? "1.0" : "0.4"}));
    }
    if (standing) {
        reject_table_floor(gd, dir, name);
    }
    const bool have = floor::StoredFloor::load(dir).has_value();
    if (have && !confirm_touch) {
        return std::string("floor from the map");
    }
    const std::string why = have ? "Confirm the floor" : "Floor";
    const std::string text =
        "Lay the RIGHT controller flat on the floor in front of you and let go. Keep looking at it for 2 seconds.";
    const auto r = gd.during(why, text, [&] {
        return cmd_floor(s({"touch", "--hand", "right", "--save", "--seconds", "25", "--name", name.c_str()}));
    });
    if (r && floor::StoredFloor::load(dir)) {
        return std::string("floor from the controller on the floor");
    }
    if (have) {
        gd.say(why, "No controller seen on the floor: keeping the floor found in the map.", std::nullopt);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return std::string("floor from the map (the controller touch did not work)");
    }
    if (!r) {
        return std::unexpected("no floor: the map shows none and the controller touch failed (" + r.error() +
                               "). Look at the floor while mapping, or turn on controller tracking.");
    }
    return std::unexpected("no floor stored (the controller touch found no room fix)");
}

R<setup::RoomSetup> finish(RingGuide& gd, const std::string& dir, const std::string& name, setup::SetupMode mode,
                           const std::string& label, setup::P2 centre, double yaw, double head_y,
                           std::vector<setup::P2> boundary, const store::RelocResult& res, bool apply_offset = true,
                           bool seated = false) {
    const auto sf = floor::StoredFloor::load(dir);
    const double floor_y = sf ? sf->y : std::numeric_limits<double>::quiet_NaN();
    setup::RoomSetup st;
    st.label = label;
    st.mode = mode;
    st.seated = seated;
    st.centre = centre;
    st.yaw = yaw;
    st.boundary = std::move(boundary);
    // head_m is the HEAD-RELATIVE floor drop (how far the floor is below the face); the chaperone write uses it as
    // --eye-height. We use the floor's stored drop (head_above_m) directly so a raw room-frame floor y (which carries a
    // huge absolute SLAM offset) can never leak into the chaperone; fall back to head_y - floor_y only if unset.
    st.head_m = (sf && sf->head_above_m > 0.1) ? sf->head_above_m : (head_y - floor_y);
    st.when = unix_now();
    if (!setup::inside(st.boundary, st.centre)) {
        const double n = static_cast<double>(st.boundary.size());
        setup::P2 c{0.0, 0.0};
        for (const auto& p : st.boundary) {
            c = {c[0] + p[0] / n, c[1] + p[1] / n};
        }
        st.centre = c;
        gd.say("Centre", "You stood outside the traced area: the middle of the area is the centre.", std::nullopt);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    st.play_area = setup::play_area(st.boundary_setup());
    if (st.play_area[0] < 0.3 || st.play_area[1] < 0.3) {
        return std::unexpected("the play area around the centre is too small (" + rf::fixed(st.play_area[0], 1) + " x " +
                               rf::fixed(st.play_area[1], 1) + " m): run the setup again");
    }
    if (auto w = st.save(dir); !w) {
        return std::unexpected(w.error());
    }
    std::printf("saved %s: %s\n", cam::path_join(dir, setup::FILE).c_str(), st.describe().c_str());
    // SteamVR-only output (Crystal Super on Linux, SteamVR is the runtime; room-setup rework Phase 1): the room setup
    // NEVER writes the Monado world offset. The floor/centre/forward go into SteamVR's chaperone instead
    // (pibridge-roomsetup --vr-standing writes the standing zero pose and commits it live). `apply_offset` and `res`
    // are kept in the signature only so the non-setup callers still compile; the setup flow is always no-offset.
    (void)apply_offset;
    (void)res;
    std::printf("steamvr: the setup is saved; the floor/centre/forward go into SteamVR's chaperone, never a Monado "
                "offset\n");
    write_status(status_path(), "ok", name, res.pid, "Room \"" + name + "\" set up: " + st.describe());
    return st;
}

R<setup::RoomSetup> run_steps(RingGuide& gd, setup::SetupMode mode, const std::string& dir, const std::string& name,
                              const std::string& label, double map_s, const std::string& title) {
    const std::uint32_t pid = gd.pid();
    auto map_args = [&](double secs_) {
        return s({"--source", "broker", "--seconds", rf::fixed(secs_, 0).c_str(), "--name", name.c_str()});
    };
    bool have_fix = false;
    if (auto w = wait_for_wear(gd, 60.0); !w) {
        return std::unexpected(w.error());
    }
    (void)check_light(gd, title, 30.0);
    if (auto f = feed_ok(pid); !f) {
        return std::unexpected(f.error());
    }
    if (mode == setup::SetupMode::Fast && cam::path_exists(cam::path_join(dir, "map.bin"))) {
        const auto r = gd.during(
            title, "Stand in the middle of your play space. Look around slowly: left, right, behind you, and at the floor.",
            [&] { return cmd_relocalize(s({"--source", "broker", "--seconds", "25", "--name", name.c_str()})); });
        if (r) {
            have_fix = true;
            if (auto m = map_with_feedback(gd, title, "Room recognised. Keep looking around for a few seconds.", map_args(12.0),
                                           true);
                !m) {
                std::printf("map not updated: %s\n", m.error().c_str());
            }
        } else {
            if (auto f = feed_ok(pid); !f) {
                return std::unexpected(f.error());
            }
            std::printf("room not recognised (%s): mapping it anew\n", r.error().c_str());
            gd.say(title, "This room does not match the saved map: making a new one.", std::nullopt);
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    if (!have_fix) {
        const std::string text =
            mode == setup::SetupMode::Fast
                ? "Stand in the middle of your play space. Look around slowly: left, right, behind you, and at the floor."
                : "Walk slowly through the room. Look at every wall, the furniture and the floor around you. End where you "
                  "started.";
        backup_map(dir);
        if (auto m = map_with_feedback(gd, mode == setup::SetupMode::Fast ? title : "1/4 Map the room", text,
                                       map_args(map_s), false);
            !m) {
            return std::unexpected(m.error());
        }
        if (auto f = save_fix_from_current(dir, pid, "room setup (map built in this session)"); !f) {
            return std::unexpected(f.error());
        }
    }
    auto res = store::RelocResult::load(cam::path_join(dir, "last-reloc.toml"));
    if (!res) {
        return std::unexpected(res.error());
    }
    if (res->pid != pid) {
        return std::unexpected("the room fix belongs to another Monado session");
    }
    gd.say(mode == setup::SetupMode::Fast ? title : "2/4 Floor", "Finding the floor...", std::nullopt);
    auto note = ensure_floor(gd, dir, name, mode == setup::SetupMode::Full, mode == setup::SetupMode::Fast);
    if (!note) {
        return std::unexpected(note.error());
    }
    std::printf("%s\n", note->c_str());
    auto fl = floor::StoredFloor::load(dir);
    if (!fl) {
        return std::unexpected("no floor: " + fl.error());
    }
    const double floor_y = fl->y;
    std::optional<std::vector<setup::P2>> boundary_room;
    if (mode == setup::SetupMode::Full) {
        gd.restart_clock();
        auto tr = setup::trace_boundary(gd, 240.0);
        if (!tr) {
            return std::unexpected(tr.error());
        }
        auto b = setup::clean_boundary(*tr, 0.05);
        if (!b) {
            return std::unexpected(b.error());
        }
        boundary_room = *b;
    }
    gd.restart_clock();
    auto cf = setup::confirm_forward(gd, 120.0);
    if (!cf) {
        return std::unexpected(cf.error());
    }
    const auto [centre, yaw, head_y] = *cf;
    std::vector<setup::P2> boundary;
    if (boundary_room) {
        boundary = *boundary_room;
    } else {
        auto m = map::RoomMap::load(dir);
        if (!m) {
            return std::unexpected(m.error());
        }
        const auto [pts, heads] = floor::map_points(*m, 4.5);
        (void)heads;
        std::vector<std::array<double, 3>> p3(pts.begin(), pts.end());
        const auto [obs, ext] = setup::obstacles_from_points(p3, floor_y, centre, yaw);
        for (const auto& p : setup::free_area(obs, ext, 2.5)) {
            boundary.push_back(setup::to_room(centre, yaw, p));
        }
    }
    return finish(gd, dir, name, mode, label, centre, yaw, head_y, std::move(boundary), *res);
}

/// PiBridge: the guide of the in-headset room setup (`setup fast --guide DIR`, docs/ROOM-SETUP.md "Map the room in the
/// headset"). The prompts and the mapping feedback go to DIR/state for `pibridge-roomsetup --map` (passthrough, floor
/// grid, boundary, controller); the user's answers come back in DIR/answer (pibridge/roommap/guide_ipc.h). The head
/// still comes from Monado's head ring (RingGuide).
class HeadsetGuide : public RingGuide {
public:
    HeadsetGuide(std::string dir, std::uint32_t pid, std::string guide_dir, pid_t tool, std::optional<double> eye)
        : RingGuide(std::move(dir), pid), guide_(std::move(guide_dir)), tool_(tool), eye_(eye) {}
    /// The user's eye height standing upright (--eye-height): the floor from the head is the first floor method.
    std::optional<double> eye() const { return eye_; }

    void say(const std::string& title, const std::string& text, std::optional<float> progress) override {
        RingGuide::say(title, text, progress);
        if (st_.title == title && st_.text == text && st_.progress == (progress ? std::optional<double>(*progress) : std::nullopt)) {
            return;
        }
        st_.title = title;
        st_.text = text;
        st_.progress = progress ? std::optional<double>(*progress) : std::nullopt;
        publish();
    }
    void step(const std::string& s) {
        st_.step = s;
        st_.progress.reset();
        publish();
    }
    void mapping(const light::MapProgress& p) {
        if (st_.keyframes == p.keyframes && st_.points == p.quality.stereo) {
            return;
        }
        st_.keyframes = p.keyframes;
        st_.points = p.quality.stereo;
        publish();
    }
    void conv(const Alignment& a) {
        st_.conv = guide::Conv{a.yaw, {a.t[0], a.t[1], a.t[2]}};
        publish();
    }
    void floor_y(std::optional<double> y) {
        st_.floor = y;
        publish();
    }
    void boundary(std::vector<setup::P2> b) {
        st_.boundary = std::move(b);
        publish();
    }

    /// The next answer of the headset for `step`: an error when the headset tool is gone, the head feed is lost, the
    /// user stopped the setup in the headset, or nothing came in `timeout_s`.
    R<guide::Answer> wait(const std::string& step, double timeout_s) {
        const auto t0 = Clock::now();
        auto checked = Clock::now();
        std::optional<Clock::time_point> bad_since;
        for (;;) {
            if (auto t = guide::read_file(guide_, guide::ANSWER_FILE)) {
                if (auto a = guide::parse_answer(*t); a && a->id > last_ && a->pid == static_cast<std::uint32_t>(tool_)) {
                    last_ = a->id;
                    if (a->kind == "cancel") {
                        return std::unexpected(std::string("stopped in the headset"));
                    }
                    if (a->step == step) {
                        return *a;
                    }
                }
            }
            if (tool_ > 1 && ::kill(tool_, 0) != 0 && errno != EPERM) {
                return std::unexpected(std::string("the headset view (pibridge-roomsetup) ended"));
            }
            if (secs(checked) >= 0.5) {
                checked = Clock::now();
                if (auto ok = feed_ok(pid()); ok) {
                    bad_since.reset();
                } else {
                    if (!bad_since) {
                        bad_since = Clock::now();
                    }
                    if (secs(*bad_since) > 2.0) {
                        return std::unexpected(ok.error());
                    }
                }
            }
            if (secs(t0) > timeout_s) {
                return std::unexpected("no answer in the headset in " + rf::fixed(timeout_s, 0) + " s: run the setup again");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    /// Whether the headset pressed Finish during the look-around (the map step's only chip); consumes the answer so
    /// the next wait() does not see it again. Used as cmd_build_with's stop_early hook (the mapping loop polls it).
    bool finish_requested() {
        if (auto t = guide::read_file(guide_, guide::ANSWER_FILE)) {
            if (auto a = guide::parse_answer(*t); a && a->id > last_ && a->pid == static_cast<std::uint32_t>(tool_) &&
                                                  a->step == "map" && a->kind == "next") {
                last_ = a->id;
                return true;
            }
        }
        return false;
    }

private:
    void publish() {
        st_.pid = static_cast<std::uint32_t>(::getpid());
        st_.seq++;
        if (auto w = guide::write_atomic(guide_, guide::STATE_FILE, guide::format_state(st_)); !w) {
            std::fprintf(stderr, "%s\n", w.error().c_str());
        }
    }

    std::string guide_;
    pid_t tool_;
    std::optional<double> eye_;
    guide::State st_;
    std::uint64_t last_ = 0;
};

/// The fast setup with the headset's answers: the same mapping as run_steps (relocalise a saved map or map anew, the
/// room fix of this session), then the floor shown as a grid on the real floor (trigger = yes, or a controller
/// touches the floor), the boundary (the free area found around you, or drawn on the floor with the controller), and
/// forward where you look when you pull the trigger. The files written are run_steps' (finish).
R<setup::RoomSetup> run_steps_headset(HeadsetGuide& gd, const std::string& dir, const std::string& name,
                                      const std::string& label, double map_s, const std::string& title,
                                      bool apply_offset = true, bool quick_direct = false) {
    const std::uint32_t pid = gd.pid();
    auto map_args = [&](double secs_) {
        return s({"--source", "broker", "--seconds", rf::fixed(secs_, 0).c_str(), "--name", name.c_str()});
    };
    const std::function<void(const light::MapProgress&)> fb = [&gd](const light::MapProgress& p) { gd.mapping(p); };
    gd.step("wear");
    if (auto w = wait_for_wear(gd, 60.0); !w) {
        return std::unexpected(w.error());
    }
    gd.step("light");
    (void)check_light(gd, title, 30.0);
    if (auto f = feed_ok(pid); !f) {
        return std::unexpected(f.error());
    }
    const std::string look = "Stand in the middle of your play space. Look around slowly: left, right, behind you, and "
                             "at the floor.";
    // the Finish chip on the look-around step ends the walk early (the mapping loop polls this through stop_early)
    const std::function<bool()> stop_cb = [&gd] { return gd.finish_requested(); };

    // --- the mode menu (room-setup rework): the pointer picks Full mapping or Quick setup in the headset. QUICK ROOM
    // SETUP launches with --quick-direct, which skips this menu and goes straight to quick. ---
    bool full = false;
    if (!quick_direct) {
        gd.step("mode");
        gd.say("Room setup",
               "Point at a button and pull the trigger. FULL MAPPING: walk the room and trace the boundary. QUICK SETUP: "
               "look around where you stand and keep the free area found for you.",
               std::nullopt);
        auto a = gd.wait("mode", 600.0);
        if (!a) {
            return std::unexpected(a.error());
        }
        full = !a->nums.empty() && a->nums[0] >= 0.5;
    } else {
        std::printf("room setup: quick-direct, skipping the mode menu\n");
    }
    const setup::SetupMode smode = full ? setup::SetupMode::Full : setup::SetupMode::Fast;
    const double walk_s = full ? std::max(map_s, 60.0) : map_s;
    std::printf("room setup mode: %s (map walk %s s)\n", full ? "full" : "quick", rf::fixed(walk_s, 0).c_str());

    // Quick setup asks the posture and the play-area size up front: the posture sets the floor drop below the head
    // (standing / seated) and the size sets the play-circle radius.
    setup::Posture posture = setup::Posture::Stand;
    double quick_radius = setup::ZONE_R_FALLBACK_M;
    if (!full) {
        gd.step("posture");
        gd.say("Posture", "Will you play STANDING or SEATED? Point at a button and pull the trigger.", std::nullopt);
        if (auto a = gd.wait("posture", 600.0)) {
            posture = setup::posture_parse(a->nums.empty() ? 0 : static_cast<int>(std::lround(a->nums[0])));
        } else {
            return std::unexpected(a.error());
        }
        gd.step("size");
        gd.say("Play area size", "How much space do you have? SMALL ~1 m, MEDIUM ~1.5 m, BIG ~2 m around you.",
               std::nullopt);
        if (auto a = gd.wait("size", 600.0)) {
            quick_radius =
                setup::zone_radius(setup::zone_size_parse(a->nums.empty() ? 1 : static_cast<int>(std::lround(a->nums[0]))));
        } else {
            return std::unexpected(a.error());
        }
        std::printf("quick setup: %s, play circle %s m\n", posture == setup::Posture::Sit ? "seated" : "standing",
                    rf::fixed(quick_radius, 2).c_str());
    }
    const bool seated = !full && posture == setup::Posture::Sit;

    // Room binding: recognise this space against a saved map (relocalize); if recognised, skip the look-around so the
    // user does not have to look around again; otherwise (not recognised, or no saved map) do one look-around to start
    // tracking and save a map. Applies to both flows.
    const bool saved_room = cam::path_exists(cam::path_join(dir, "map.bin"));
    bool recognised = false;
    if (saved_room) {
        gd.step("map");
        const auto r = gd.during(title, look, [&] {
            return cmd_relocalize(s({"--source", "broker", "--seconds", "20", "--name", name.c_str()}));
        });
        if (r) {
            recognised = true;
            std::printf("room recognised against the saved map: skipping the look-around\n");
        } else {
            if (auto f = feed_ok(pid); !f) {
                return std::unexpected(f.error());
            }
            std::printf("room not recognised (%s): a look-around is needed\n", r.error().c_str());
        }
    }
    if (!recognised) {
        gd.step("map");
        backup_map(dir);
        if (auto m = map_with_feedback(gd, title, look, map_args(walk_s), false, fb, &stop_cb); !m) {
            return std::unexpected(m.error());
        }
        if (auto f = save_fix_from_current(dir, pid, "room setup look-around"); !f) {
            return std::unexpected(f.error());
        }
    }

    // the room fix of this session (its frame for the live head samples); both flows have one after binding
    auto res = store::RelocResult::load(cam::path_join(dir, "last-reloc.toml"));
    if (!res) {
        return std::unexpected(res.error());
    }
    if (res->pid != pid) {
        return std::unexpected("the room fix belongs to another Monado session");
    }
    gd.conv(res->align);

    // the live head in the room frame (centre x z, forward heading, head y); neutral when there is no sample yet
    const auto head_now = [&]() -> std::tuple<setup::P2, double, double> {
        if (const auto h = gd.sample().head) {
            return {setup::P2{h->first[0], h->first[2]}, h->second, h->first[1]};
        }
        return {setup::P2{0.0, 0.0}, 0.0, 1.6};
    };
    // store the floor as a head-relative drop (the chaperone uses the drop, never a raw room-frame y)
    const auto save_floor = [&](double head_y, double drop) -> R<void> {
        floor::StoredFloor f;
        f.y = head_y - drop;
        f.source = floor::FloorSource::Standing;
        f.head_above_m = drop;
        f.confidence = 0.9;
        f.sigma_m = 0.02;
        f.when = unix_now();
        if (auto w = f.save(dir); !w) {
            return std::unexpected("the floor could not be saved: " + w.error());
        }
        gd.floor_y(f.y);
        return {};
    };

    if (!full) {
        // --- QUICK: no floor step at all. The floor is the posture default silently (standing eye height / 1.60,
        // seated 1.30). One final confirm with a FINISH button, then everything is taken from the LIVE head: centre
        // under the head, forward where the head looks, play area the chosen S/M/L circle. Seated writes a seated-zero
        // chaperone with no perimeter (main.cpp). ---
        const double drop = setup::posture_floor_drop(posture, gd.eye());
        gd.step("confirm");
        gd.say("Ready to save", seated ? "Sit where you want the centre and press FINISH."
                                       : "Stand where you want the centre, look where forward should be, and press FINISH.",
               std::nullopt);
        if (const auto h = gd.sample().head) {
            gd.boundary(setup::circle({h->first[0], h->first[2]}, quick_radius, 32)); // preview the play circle
        }
        if (auto a = gd.wait("confirm", 600.0); !a) {
            return std::unexpected(a.error());
        }
        const auto [centre, yaw, head_y] = head_now();
        if (auto w = save_floor(head_y, drop); !w) {
            return std::unexpected(w.error());
        }
        std::vector<setup::P2> boundary = setup::circle(centre, quick_radius, 32);
        gd.boundary(boundary);
        return finish(gd, dir, name, smode, label, centre, yaw, head_y, std::move(boundary), *res, apply_offset, seated);
    }

    // --- CUSTOM / FULL (always standing): a floor fine-tune step, then draw the border, then finish. ---
    std::vector<setup::P2> boundary;
    setup::P2 centre{};
    double fwd_yaw = 0, fwd_head_y = 0;
    std::string phase = "floor";
    while (phase != "finish") {
        if (phase == "floor") {
            // the floor starts the entered eye height (else 1.60 m) below the head; the headset shows "the floor is N m
            // below your face" and the thumbstick nudges it +/- 1 cm within 1.00..2.00 m. NEXT answers with the drop.
            const auto [c0, yaw0, head_y0] = head_now();
            (void)c0;
            (void)yaw0;
            const double drop_default = gd.eye().value_or(setup::FLOOR_DROP_STAND_M);
            gd.floor_y(head_y0 - drop_default);
            gd.step("floor-confirm");
            gd.say("Floor", "The grid shows the floor below your feet. Nudge it up / down with the thumbstick, then press "
                            "NEXT.",
                   std::nullopt);
            auto fa = gd.wait("floor-confirm", 600.0);
            if (!fa) {
                return std::unexpected(fa.error());
            }
            if (fa->kind == "repeat") {
                gd.floor_y(head_y0 - drop_default);
                continue;
            }
            if (fa->kind == "height" && fa->nums.size() == 2) {
                // nums[0] = floor room y (head - drop), nums[1] = the head-relative drop (clamped in the headset)
                const auto [cc, yy, hy] = head_now();
                (void)cc;
                (void)yy;
                if (auto w = save_floor(hy, fa->nums[1]); !w) {
                    return std::unexpected(w.error());
                }
                std::printf("floor set %s m below the face\n", rf::fixed(fa->nums[1], 2).c_str());
                phase = "boundary";
            }
            continue;
        }
        // phase == "boundary"
        auto fl = floor::StoredFloor::load(dir);
        if (!fl) {
            return std::unexpected("no floor: " + fl.error());
        }
        gd.step("boundary");
        gd.say("Play area", "Hold the trigger and draw the edge of your space on the floor, then press SAVE.",
               std::nullopt);
        bool advanced = false;
        while (!advanced) {
            auto a = gd.wait("boundary", 600.0);
            if (!a) {
                return std::unexpected(a.error());
            }
            if (a->kind == "back") {
                phase = "floor";
                break;
            }
            if (a->kind == "repeat") {
                gd.boundary({});
                gd.say("Play area", "Hold the trigger and draw the edge of your space on the floor, then press SAVE.",
                       std::nullopt);
                continue;
            }
            if (a->kind == "path") {
                std::vector<setup::P2> path;
                for (std::size_t i = 0; i + 1 < a->nums.size(); i += 2) {
                    path.push_back({a->nums[i], a->nums[i + 1]});
                }
                // tidy the stroke (clamp to <= 5 m from the centre, drop near-duplicate points, cap the count), then
                // accept it only if the convex hull encloses a usable play area (>= 1.5 m^2).
                const auto [c, yaw, head_y] = head_now();
                const auto prepared = setup::prepare_boundary(path, c);
                auto b = setup::clean_boundary(prepared, setup::MIN_POINT_SPACING_M);
                if (b && setup::zone_area_ok(*b)) {
                    boundary = *b;
                    gd.boundary(boundary);
                    centre = c;        // centre / forward from the live head, no separate forward step
                    fwd_yaw = yaw;
                    fwd_head_y = head_y;
                    phase = "finish";
                    advanced = true;
                    break;
                }
                const std::string why = !b ? b.error()
                                           : ("the area is too small, it needs at least " +
                                              rf::fixed(setup::MIN_ZONE_AREA_M2, 1) + " m^2");
                gd.say("Play area", why + ". Hold the trigger and draw a bigger loop, then press SAVE.", std::nullopt);
                continue;
            }
            gd.say("Play area", "Hold the trigger and draw the edge of your space on the floor, then press SAVE.",
                   std::nullopt);
        }
    }
    return finish(gd, dir, name, smode, label, centre, fwd_yaw, fwd_head_y, std::move(boundary), *res, apply_offset,
                  seated);
}

R<void> cmd_setup_run(setup::SetupMode mode, Args v) {
    auto dir_r = map_dir(v);
    if (!dir_r) {
        return std::unexpected(dir_r.error());
    }
    const std::string dir = *dir_r;
    std::string name = rf::utf8_lossy(rs::file_name(dir));
    if (name.empty()) {
        name = "home";
    }
    auto label_o = opt(v, "--label");
    if (!label_o) {
        return std::unexpected(label_o.error());
    }
    const std::string label = label_o->value_or(name);
    const bool no_hud = flag(v, "--no-hud");
    // SteamVR path: save the setup but do NOT write the Monado world offset (pibridge-roomsetup --vr-standing puts the
    // floor/centre/forward into SteamVR's chaperone instead). Also skips the ">20 m offset" guard of the offset write.
    const bool no_offset = flag(v, "--no-offset");
    // QUICK ROOM SETUP launches with --quick-direct: the headset flow skips the Full/Quick mode menu and goes to quick.
    const bool quick_direct = flag(v, "--quick-direct");
    auto guide_o = opt(v, "--guide");
    if (!guide_o) {
        return std::unexpected(guide_o.error());
    }
    auto eye_o = opt(v, "--eye-height");
    if (!eye_o) {
        return std::unexpected(eye_o.error());
    }
    std::optional<double> eye;
    if (*eye_o) {
        const auto e = rs::parse_f64(**eye_o);
        if (!e || !(*e >= 1.0 && *e <= 2.3)) {
            return std::unexpected(std::string("--eye-height is your eye height standing upright, 1.0-2.3 m"));
        }
        eye = *e;
    }
    if (*guide_o && mode != setup::SetupMode::Fast) {
        return std::unexpected(std::string("--guide works with setup fast"));
    }
    auto map_s = num_f64(v, "--map-seconds", mode == setup::SetupMode::Fast ? 30.0 : 60.0);
    if (!map_s) {
        return std::unexpected(map_s.error());
    }
    if (auto d = done(v); !d) {
        return d;
    }
    const auto feed = anchor::feed_now();
    const auto pid = feed.live_pid();
    if (!pid) {
        return std::unexpected("cannot start the room setup: " + feed.plain());
    }
    if (tag_holds(*pid)) {
        return std::unexpected("the AprilTag placed the world in this session: press Clear offset on the Room page first");
    }
    // The room setup is SteamVR-chaperone-only: finish() writes no Monado world offset. The (now vestigial) offset
    // lease must therefore NOT be able to block a user-initiated setup. Take it when it is free, but if a leftover or
    // background holder (e.g. the relocalizer, or a stale pid) has it, just carry on — the setup still finishes to the
    // SteamVR chaperone, and it never writes the offset, so holding the lease is not required.
    if (auto l = take_lease(*pid, anchor::Role::Setup); !l) {
        std::printf("note: could not take the room offset lease (%s); continuing anyway (the setup writes no Monado "
                    "offset, only the SteamVR chaperone)\n",
                    l.error().c_str());
    }
    if (*guide_o) {
        // PiBridge: the in-headset room setup (pibridge-roomsetup --map) shows the steps and answers them
        const std::string gdir = **guide_o;
        std::error_code ec;
        std::filesystem::create_directories(gdir, ec);
        std::filesystem::remove(gdir + "/" + guide::ANSWER_FILE, ec);
        HeadsetGuide hg(dir, *pid, gdir, ::getppid(), eye);
        auto r = run_steps_headset(hg, dir, name, label, *map_s, "Room setup", !no_offset, quick_direct);
        if (r) {
            hg.step("done");
            hg.say("Room setup done",
                   "Play area " + rf::fixed(r->play_area[0], 1) + " x " + rf::fixed(r->play_area[1], 1) +
                       " m. The floor, the centre and forward are saved; they come back at every start.",
                   std::nullopt);
        } else {
            hg.step("failed");
            hg.say("Room setup stopped", r.error(), std::nullopt);
            write_status(status_path(), "failed", name, *pid, "Room setup stopped: " + r.error());
            return std::unexpected(r.error());
        }
        return {};
    }
    if (!no_hud) {
        std::printf("note: no in-headset panel (pibridge-roommap has none; the app's room setup shows the steps in the "
                    "headset): follow the steps here\n");
    }
    if (auto f = feed_ok(*pid); !f) {
        write_status(status_path(), "failed", name, *pid, "Room setup stopped: " + f.error());
        return std::unexpected(f.error());
    }
    RingGuide gd(dir, *pid);
    const std::string title = mode == setup::SetupMode::Fast ? "Fast room setup" : "Room setup";
    auto r = run_steps(gd, mode, dir, name, label, *map_s, title);
    if (r) {
        gd.say("Room setup done",
               "Play area " + rf::fixed(r->play_area[0], 1) + " x " + rf::fixed(r->play_area[1], 1) +
                   " m. The floor, the centre and forward are saved; they come back at every start.",
               std::nullopt);
    } else {
        gd.say("Room setup stopped", r.error(), std::nullopt);
        write_status(status_path(), "failed", name, *pid, "Room setup stopped: " + r.error());
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (!r) {
        return std::unexpected(r.error());
    }
    return {};
}

R<void> cmd_setup_show(Args v) {
    auto dir = map_dir(v);
    if (!dir) {
        return std::unexpected(dir.error());
    }
    if (auto d = done(v); !d) {
        return d;
    }
    auto st = setup::RoomSetup::load(*dir);
    if (!st) {
        return std::unexpected(st.error());
    }
    std::printf("%s: \"%s\" %s\n", dir->c_str(), st->label.c_str(), st->describe().c_str());
    std::printf("centre (%s, %s) m, forward %s deg in the map, %zu boundary points, head %s m above the floor at setup\n",
                rf::fixed(st->centre[0], 2, true).c_str(), rf::fixed(st->centre[1], 2, true).c_str(),
                rf::fixed(rf::to_degrees(st->yaw), 1).c_str(), st->boundary.size(), rf::fixed(st->head_m, 2).c_str());
    return {};
}

using MapId = std::optional<std::tuple<std::uint64_t, std::int64_t, std::int64_t>>;

MapId map_identity(const std::string& dir) {
    struct stat st {};
    if (::stat(cam::path_join(dir, "map.bin").c_str(), &st) != 0) {
        return std::nullopt;
    }
    return std::tuple<std::uint64_t, std::int64_t, std::int64_t>{static_cast<std::uint64_t>(st.st_size),
                                                                st.st_mtim.tv_sec, st.st_mtim.tv_nsec};
}

std::optional<std::string> auto_must_stop(const std::string& offset, std::uint32_t pid, std::uint32_t me,
                                          const MapId& started, const MapId& now) {
    if (!anchor::holds_lease_as(offset, pid, me)) {
        const auto l = anchor::read_lease(anchor::lease_path_for(offset));
        const char* who = (l && l->role) ? anchor::role_title(*l->role) : "another writer";
        return std::string("Room placement left to ") + who + " in this session.";
    }
    if (started != now) {
        return std::string("The room map was made again in this session: the room setup places it.");
    }
    return std::nullopt;
}

} // namespace

R<std::string> write_stage_bounds(std::uint32_t pid, std::array<double, 2> play_area) {
    const std::string p = stage_bounds_path();
    const std::string tmp = rs::with_extension(p, "tmp");
    const std::string text = "pimax-stage-bounds 1\npid " + std::to_string(pid) + "\nsize " + rf::fixed(play_area[0], 3) +
                             " " + rf::fixed(play_area[1], 3) + "\n";
    if (auto w = cam::write_file(tmp, text); !w) {
        return std::unexpected(w.error().text());
    }
    if (std::rename(tmp.c_str(), p.c_str()) != 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    return p;
}

R<void> cmd_setup(Args v) {
    if (v.empty()) {
        return std::unexpected(std::string("setup needs fast, full or show:\n") + SETUP_HELP);
    }
    const std::string x = v[0];
    v.erase(v.begin());
    if (x == "fast") {
        return cmd_setup_run(setup::SetupMode::Fast, std::move(v));
    }
    if (x == "full") {
        return cmd_setup_run(setup::SetupMode::Full, std::move(v));
    }
    if (x == "show") {
        return cmd_setup_show(std::move(v));
    }
    return std::unexpected("unknown setup command " + rf::debug_str(x) + "\n" + SETUP_HELP);
}

R<void> cmd_auto(Args v) {
    auto dir_r = map_dir(v);
    if (!dir_r) {
        return std::unexpected(dir_r.error());
    }
    const std::string dir = *dir_r;
    auto wait = num_f64(v, "--wait", 120.0);
    if (!wait) {
        return std::unexpected(wait.error());
    }
    auto st_o = opt(v, "--status");
    if (!st_o) {
        return std::unexpected(st_o.error());
    }
    const std::string status = st_o->value_or(status_path());
    if (auto d = done(v); !d) {
        return d;
    }
    const std::string name = rf::utf8_lossy(rs::file_name(dir));
    if (!cam::path_exists(cam::path_join(dir, "map.bin"))) {
        write_status(status, "off", name, 0, "No room \"" + name + "\" saved yet: run Fast setup on the Room page.");
        return {};
    }
    const auto t0 = Clock::now();
    std::uint32_t pid = 0;
    for (;;) {
        if (const auto p = ring_pid()) {
            pid = *p;
            break;
        }
        if (secs(t0) > *wait) {
            write_status(status, "failed", name, 0, "Room not put in place: Monado runs without the room anchor hook or 6DoF.");
            return {};
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (tag_holds(pid)) {
        write_status(status, "skipped", name, pid, "The AprilTag placed the world in this session: the room map leaves it alone.");
        return {};
    }
    if (auto l = take_lease(pid, anchor::Role::Auto); !l) {
        write_status(status, "skipped", name, pid, "Room left as it is: " + l.error() + ".");
        return {};
    }
    const MapId map_id = map_identity(dir);
    write_status(status, "searching", name, pid, "Looking for the room \"" + name + "\": look around for a few seconds.");
    int tries = 0;
    for (;;) {
        if (!alive(pid) || ring_owner() != pid) {
            write_status(status, "off", name, pid, "The session ended before the room was recognised.");
            return {};
        }
        if (const auto why = auto_must_stop(anchor::offset_path(), pid, rs::process_id(), map_id, map_identity(dir))) {
            write_status(status, "skipped", name, pid, *why);
            return {};
        }
        tries++;
        const auto r = cmd_relocalize(s({"--source", "broker", "--seconds", "15", "--name", name.c_str(), "--apply"}));
        if (r) {
            std::string extra;
            if (auto st = setup::RoomSetup::load(dir)) {
                extra = " (floor, centre and forward; " + st->describe() + ")";
            } else {
                extra = " (floor only: no room setup yet, run Fast setup for the play area and forward)";
            }
            write_status(status, "ok", name, pid,
                         "Room \"" + name + "\" in place after " + rf::fixed(secs(t0), 0) + " s" + extra + ".");
            return {};
        }
        std::printf("try %d: %s\n", tries, r.error().c_str());
        if (secs(t0) > *wait) {
            write_status(status, "failed", name, pid,
                         "Room \"" + name +
                             "\" not recognised: the world starts where the headset is. Look around the room and press "
                             "Recognise room, or run Fast setup again (lights, furniture moved?).");
            return {};
        }
        write_status(status, "searching", name, pid,
                     "Still looking for the room \"" + name + "\" (try " + std::to_string(tries) + "): look around slowly.");
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}

} // namespace pibridge::roommap::cli
