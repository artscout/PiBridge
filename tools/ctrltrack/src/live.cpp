// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/live.rs (MIT OR Apache-2.0).
//
// `live`: controller poses from the live LED frames (camera broker), published for Monado. Each LED frame: blobs per
// camera (BlobParams::live), then the shared tracker (tracker.h): every controller seen recently is tracked from its
// velocity-predicted prior (12 px gate, 24 px when it moves fast; 3-4 LED fixes with the orientation tied to the
// prediction); a controller that is not tracked is searched by brute force (about 10 ms over 4 threads). Poses go to
// the seqlock file read by Monado's Pimax controller driver (PIMAX_CTRL_TRACK=1). `--baseline` runs the tracker and
// blob detection as before 2026-10-15 (A/B). `show` prints what Monado would read; `health` the heartbeat as JSON.
#include "cli.h"

#include "pibridge/cam/frame.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/cam/source.h"
#include "pibridge/ctrl/hint.h"
#include "pibridge/ctrl/parity.h"
#include "pibridge/ctrl/track/blob.h"
#include "pibridge/ctrl/track/imutrack.h"
#include "pibridge/ctrl/track/rig.h"
#include "pibridge/ctrl/track/shm.h"
#include "pibridge/ctrl/track/tracker.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <format>
#include <limits>
#include <thread>

namespace pibridge::ctrltool {

namespace g = cam::geom;
namespace rf = cam::rfmt;
namespace tr = ctrl::track;
namespace shm = ctrl::track::shm;

namespace {

using Clock = std::chrono::steady_clock;

double secs_since(Clock::time_point t) { return std::chrono::duration<double>(Clock::now() - t).count(); }

bool has(std::span<const std::string> a, std::string_view k) { return std::find(a.begin(), a.end(), k) != a.end(); }

/// Default pitch of the aim frame relative to the LED-model frame (deg): with it, the aim ray of a controller held
/// the way the user held them in omarchyvr's `ctrl-test.pxcam` points level and straight ahead (mean model
/// orientation 0-40 s, about 52 deg about +x for both hands).
constexpr double DEFAULT_AIM_PITCH_DEG = 52.0;

std::uint64_t sat_sub(std::uint64_t a, std::uint64_t b) { return a > b ? a - b : 0; }

} // namespace

int cmd_live(std::span<const std::string> a) {
    const std::string spec = tr::arg(a, "--source").value_or("broker");
    const std::string out = tr::arg(a, "--out").value_or(shm::default_path());
    const double secs =
        tr::arg(a, "--seconds") ? parse_f64(*tr::arg(a, "--seconds")) : std::numeric_limits<double>::infinity();
    double pitch = DEFAULT_AIM_PITCH_DEG;
    if (const auto v = tr::arg(a, "--aim-pitch-deg")) {
        pitch = parse_f64(*v);
    } else if (const auto e = cam::env_utf8("PIMAX_CTRLTRACK_AIM_PITCH_DEG")) {
        pitch = parse_f64_ok(*e).value_or(DEFAULT_AIM_PITCH_DEG);
    }
    const double stats_every = tr::arg(a, "--stats") ? parse_f64(*tr::arg(a, "--stats")) : 5.0;
    auto loaded = tr::Rig::try_load(a);
    if (!loaded) {
        eprintln("pimax-ctrltrack: " + loaded.error());
        eprintln("pimax-ctrltrack: optical controller tracking is off; the controllers still work (buttons, gyro).");
        return 2;
    }
    const tr::Rig rig = std::move(*loaded);
    // device_from_aim: rotation about the model's x axis
    const g::Pose device_from_aim{g::rot_x(-pitch), {0.0, 0.0, 0.0}};
    std::array<std::array<float, 4>, 2> aim_from_imu{};
    for (std::size_t i = 0; i < 2; i++) {
        const g::Mat3 r = g::mat_mul(g::transpose(device_from_aim.r), rig.models[i].imu_to_device.r);
        const auto q = g::quat_from_mat(r);
        aim_from_imu[i] = {static_cast<float>(q[0]), static_cast<float>(q[1]), static_cast<float>(q[2]),
                           static_cast<float>(q[3])};
    }
    auto opened = cam::open_source(spec, 0, cam::ReplayOptions{false, true});
    if (!opened) {
        eprintln(std::format("cannot open source {}: {}", rf::debug_str(spec), opened.error().text()));
        return 1;
    }
    std::unique_ptr<cam::Source> src = std::move(*opened);
    auto created = shm::PoseWriter::create(out);
    if (!created) {
        eprintln(std::format("cannot create {}: {}", rf::utf8_lossy(out), created.error().text()));
        return 1;
    }
    std::unique_ptr<shm::PoseWriter> w = std::move(*created);
    eprintln(std::format("live: source {} -> {} (aim pitch {} deg about the model x axis)", src->describe(),
                         rf::utf8_lossy(out), rf::display_f64(pitch)));

    // PiBridge's improvements (docs/research/ctrl-tracking-improvements.md) unless PIBRIDGE_CTRL_PARITY=1 (the Rust tool)
    const bool parity = ctrl::parity_mode();
    tr::BlobParams bp = has(a, "--baseline") ? tr::BlobParams{} : tr::BlobParams::from_args(a);
    if (!parity && !has(a, "--baseline")) {
        const tr::BlobParams im = tr::BlobParams::improved();
        bp.core = im.core;
        bp.dim = im.dim;
    }
    std::vector<std::uint8_t> tile(cam::TILE_W * cam::TILE_H);
    const auto t_start = Clock::now();
    tr::TrackCfg cfg = has(a, "--baseline") ? tr::TrackCfg::baseline() : parity ? tr::TrackCfg{} : tr::TrackCfg::improved();
    cfg.body = !has(a, "--no-body");
    // the fuser's hint file (pibridge/ctrl/hint.h), opt-in: PIBRIDGE_CTRL_TRACK_HINT=1
    if (!parity) {
        if (const auto e = cam::env_utf8("PIBRIDGE_CTRL_TRACK_HINT"); e && rf::trim(*e) == "1") {
            cfg.use_hint = true;
        }
    }
    std::unique_ptr<ctrl::hint::Reader> hint_reader;
    std::uint64_t hint_tried = 0;
    const g::Pose aim_from_device = device_from_aim.inverse();
    tr::Tracker tracker(cfg);
    tracker.device_from_aim = device_from_aim.r;
    // the IMU-led tracker (imutrack.h): the default unless PIBRIDGE_CTRL_TRACKER=frame (or parity, or --baseline). It
    // runs the per-frame tracker itself while there is no IMU (hint file) or no head pose.
    std::unique_ptr<tr::ImuTracker> itk;
    if (!parity && !has(a, "--baseline")) {
        const auto e = cam::env_utf8("PIBRIDGE_CTRL_TRACKER");
        if (!(e && rf::trim(*e) == "frame")) {
            itk = std::make_unique<tr::ImuTracker>(tr::ImuTrackCfg{}, cfg);
            itk->device_from_aim = device_from_aim.r;
        }
    }
    std::FILE* trace = nullptr;
    if (const auto p = tr::arg(a, "--trace")) {
        trace = std::fopen(p->c_str(), "w");
        if (trace == nullptr) {
            panic("trace: " + io_debug(cam::IoError::last_os()));
        }
    }
    auto t_stats = Clock::now();
    double solve_ms = 0.0;
    // Heartbeat in the pose file (shm.h): written at least every 0.5 s (the read timeout) while this loop runs, so a
    // probe can tell a live tracker without controllers in view from a hung or dead one. The source reopens itself
    // after a broker restart or a camera drop (libpibridge-cam resilient); `reopens` counts those.
    shm::Beat beat;
    std::uint64_t gen = src->generation();
    std::uint64_t inhand_checked = 0;
    for (;;) {
        if (secs_since(t_start) > secs) {
            break;
        }
        const std::uint64_t now = shm::monotonic_ns();
        if (sat_sub(now, beat.beat_ns) >= 100'000'000) {
            const std::uint64_t gn = src->generation();
            if (gn != gen) {
                gen = gn;
                beat.reopens += 1;
            }
            beat.beat_ns = now;
            beat.source_up = src->up();
            w->beat(beat);
        }
        // Monado's in-hand status (PIMAX_CTRL_INHAND): a controller lying on the desk is not looked for
        if (sat_sub(now, inhand_checked) >= 100'000'000) {
            inhand_checked = now;
            const auto wanted = shm::inhand_wanted(shm::inhand_status_path());
            if (wanted != tracker.wanted) {
                eprintln(std::format("live: looking for left {}, right {} (Monado in-hand status)",
                                     wanted[0] ? "yes" : "no", wanted[1] ? "yes" : "no"));
                tracker.wanted = wanted;
            }
        }
        auto got = src->next(500);
        if (!got) {
            // only a replay ends this way; a live source reopens by itself
            eprintln("live: source error: " + got.error().text());
            break;
        }
        if (!*got) {
            if (src->finished()) {
                eprintln("live: source finished");
                break;
            }
            continue;
        }
        const cam::RawFrame fr = **got;
        const auto m = cam::Meta::parse(fr.data.first(cam::META_LEN));
        if (!m) {
            continue;
        }
        if (m->kind() != cam::FrameKind::Led) {
            continue;
        }
        beat.led_frames += 1;
        if (!tracker.wanted[0] && !tracker.wanted[1]) {
            continue; // both controllers put down: no blob work at all
        }
        const std::uint64_t frame_ns = fr.host_ts_ns;
        const auto t0 = Clock::now();
        if (cfg.use_hint || cfg.imu_id) {
            if (!hint_reader && sat_sub(now, hint_tried) > 1'000'000'000) {
                hint_tried = now;
                hint_reader.reset(ctrl::hint::Reader::open(ctrl::hint::default_path()));
            }
            for (std::size_t s = 0; s < 2; s++) {
                tracker.hint[s] = std::nullopt;
                if (!hint_reader || !hint_reader->live()) {
                    continue;
                }
                const auto ring = hint_reader->read(s);
                // the controller's IMU for the hand-identity test (calibrated gyro axes only)
                for (const auto& x : ring) {
                    if ((x.flags & 1u) != 0) {
                        tr::ImuSample is;
                        is.t = x.t_ns;
                        is.qg = tr::quat_to_mat({x.qg[0], x.qg[1], x.qg[2], x.qg[3]});
                        is.up_aim = {x.up_aim[0], x.up_aim[1], x.up_aim[2]};
                        is.up_ok = (x.flags & 2u) != 0;
                        is.up_body = {x.up_head[0], x.up_head[1], x.up_head[2]};
                        is.still_s = x.still_s;
                        tracker.imu_push(s, is);
                        if (itk) {
                            tr::ImuIn ii;
                            ii.t = x.t_ns;
                            ii.qg = is.qg;
                            ii.f_aim = {x.f_aim[0], x.f_aim[1], x.f_aim[2]};
                            ii.up_aim = is.up_aim;
                            ii.up_ok = is.up_ok;
                            ii.still_s = x.still_s;
                            ii.world_from_head = g::Pose{
                                tr::quat_to_mat({x.head_quat[0], x.head_quat[1], x.head_quat[2], x.head_quat[3]}),
                                {x.head_pos[0], x.head_pos[1], x.head_pos[2]}};
                            itk->imu_push(s, ii);
                        }
                    }
                }
                if (!cfg.use_hint) {
                    continue;
                }
                const auto e = ctrl::hint::at(ring, frame_ns);
                if (!e || e->state == 0 || (e->flags & 4u) == 0) {
                    continue;
                }
                const g::Pose body_from_aim{
                    tr::quat_to_mat({e->quat[0], e->quat[1], e->quat[2], e->quat[3]}),
                    {e->pos[0], e->pos[1], e->pos[2]}};
                tracker.hint[s] = tr::Hint{body_from_aim.compose(aim_from_device),
                                           static_cast<double>(sat_sub(frame_ns, e->fix_ns)) * 1e-9, e->state};
            }
        }
        const cam::TileRows rows = cam::tile_rows(&*m, fr.data, fr.stride);
        tr::LedFrame lf{static_cast<std::size_t>(fr.sequence), m->ts_us, {}};
        lf.px.reserve(cam::TILE_COUNT);
        for (std::size_t t = 0; t < cam::TILE_COUNT; t++) {
            cam::copy_tile(fr.data, fr.stride, t, rows.y[t], tile, cam::TILE_W);
            std::vector<std::pair<double, double>> px;
            for (const tr::Blob& b : tr::detect(tile, cam::TILE_W, cam::TILE_H, cam::TILE_W, bp)) {
                px.emplace_back(b.x, b.y);
            }
            lf.px.push_back(std::move(px));
        }
        if (itk) {
            itk->wanted = tracker.wanted;
        }
        for (const tr::Fix& fx : itk ? itk->step(rig, lf, frame_ns) : tracker.step(rig, lf, frame_ns)) {
            const std::size_t slot = fx.slot;
            const tr::Found& f = fx.found;
            if (trace != nullptr) {
                const g::Vec3 p = f.joint.body_from_model.t;
                const std::string l = std::format("{},{},{},{},{},{},{}\n", frame_ns, slot, rf::fixed(p[0], 4),
                                                  rf::fixed(p[1], 4), rf::fixed(p[2], 4), f.joint.total(), f.model);
                std::fwrite(l.data(), 1, l.size(), trace);
            }
            const g::Pose body_from_aim = (fx.published ? *fx.published : f.joint.body_from_model).compose(device_from_aim);
            const auto q = g::quat_from_mat(body_from_aim.r);
            shm::HandPose h;
            h.frame_ns = frame_ns;
            h.publish_ns = shm::monotonic_ns();
            h.pos = {static_cast<float>(body_from_aim.t[0]), static_cast<float>(body_from_aim.t[1]),
                     static_cast<float>(body_from_aim.t[2])};
            h.quat = {static_cast<float>(q[0]), static_cast<float>(q[1]), static_cast<float>(q[2]),
                      static_cast<float>(q[3])};
            h.rms_px = static_cast<float>(f.joint.rms_px);
            h.leds = static_cast<std::uint32_t>(f.joint.total());
            h.cams = static_cast<std::uint32_t>(f.joint.cams());
            h.aim_from_imu = aim_from_imu[slot];
            w->set(slot, h);
        }
        solve_ms += secs_since(t0) * 1e3;
        if (secs_since(t_stats) >= stats_every) {
            const tr::Stats s = itk ? itk->stats() : tracker.stats;
            const double n = static_cast<double>(std::max<std::uint64_t>(s.led_frames, 1));
            eprintln(std::format(
                "live: {} LED frames, left {} %, right {} %, prior tracks {}/{}, few-LED {}/{}, searches {}, "
                "body-model rejects {}, collapsed {}, frame work {} ms (mean)",
                s.led_frames, rf::fixed(100.0 * static_cast<double>(s.hand[0]) / n, 0),
                rf::fixed(100.0 * static_cast<double>(s.hand[1]) / n, 0), s.hand[0] + s.hand[1], s.tries, s.few[0],
                s.few[1], s.searches, s.unassigned, s.collapsed, rf::fixed(solve_ms / n, 1)));
            for (std::size_t slot = 0; slot < 2; slot++) {
                if (const auto l = tracker.last(slot)) {
                    const g::Pose ba = std::get<1>(*l).compose(device_from_aim);
                    eprintln(std::format("   {}: ({} {} {}) m", slot == 0 ? "left" : "right",
                                         rf::fixed(ba.t[0], 3, true), rf::fixed(ba.t[1], 3, true),
                                         rf::fixed(ba.t[2], 3, true)));
                }
            }
            t_stats = Clock::now();
            tracker.stats = tr::Stats{};
            if (itk) {
                itk->stats() = tr::Stats{};
            }
            solve_ms = 0.0;
        }
    }
    if (trace != nullptr) {
        std::fclose(trace);
    }
    return 0;
}

int cmd_show(std::span<const std::string> a) {
    const std::string path = tr::arg(a, "--out").value_or(shm::default_path());
    const double secs = tr::arg(a, "--seconds") ? parse_f64(*tr::arg(a, "--seconds")) : 10.0;
    auto opened = shm::PoseReader::open(path);
    if (!opened) {
        eprintln(std::format("{}: {}", rf::utf8_lossy(path), opened.error().text()));
        return 1;
    }
    const std::unique_ptr<shm::PoseReader> r = std::move(*opened);
    const auto t0 = Clock::now();
    while (secs_since(t0) < secs) {
        if (const auto snap = r->read_all()) {
            const auto& [h, b] = *snap;
            const std::uint64_t now = shm::monotonic_ns();
            std::string line;
            if (const auto age = b.age_ns(now)) {
                line = std::format("beat {} ms src {} reopens {}  ",
                                   rf::fixed_w(rf::fmin(static_cast<double>(*age) * 1e-6, 99999.0), 5, 0),
                                   b.source_up ? "up" : "DOWN", b.reopens);
            }
            for (std::size_t i = 0; i < 2; i++) {
                const shm::HandPose& p = h[i];
                const double age = p.frame_ns == 0 ? std::numeric_limits<double>::infinity()
                                                   : static_cast<double>(sat_sub(now, p.frame_ns)) * 1e-6;
                const g::Vec3 fwd = g::mat_vec(
                    tr::quat_to_mat({static_cast<double>(p.quat[0]), static_cast<double>(p.quat[1]),
                                     static_cast<double>(p.quat[2]), static_cast<double>(p.quat[3])}),
                    {0.0, 0.0, -1.0});
                line += std::format(
                    "{} age {} ms pos ({} {} {}) aim fwd ({} {} {}) LEDs {:2} cams {} rms {}   ", i == 0 ? "L" : "R",
                    rf::fixed_w(rf::fmin(age, 99999.0), 6, 0), rf::fixed(p.pos[0], 3, true),
                    rf::fixed(p.pos[1], 3, true), rf::fixed(p.pos[2], 3, true), rf::fixed(fwd[0], 2, true),
                    rf::fixed(fwd[1], 2, true), rf::fixed(fwd[2], 2, true), p.leds, p.cams, rf::fixed(p.rms_px, 2));
            }
            println(line);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}

int cmd_health(std::span<const std::string> a) {
    const std::string path = tr::arg(a, "--out").value_or(shm::default_path());
    double max_age = 2.0;
    if (const auto v = tr::arg(a, "--max-age")) {
        max_age = parse_f64_ok(*v).value_or(2.0);
    }
    std::optional<std::pair<std::array<shm::HandPose, 2>, shm::Beat>> snap;
    if (auto r = shm::PoseReader::open(path)) {
        snap = (*r)->read_all();
    }
    if (!snap) {
        println("{\"ok\":false,\"why\":\"no pose file\"}");
        return 1;
    }
    std::uint32_t pid = 0;
    if (const auto d = cam::read_file(path); d && d->size() >= 16) {
        pid = static_cast<std::uint32_t>(static_cast<std::uint8_t>((*d)[12])) |
              static_cast<std::uint32_t>(static_cast<std::uint8_t>((*d)[13])) << 8 |
              static_cast<std::uint32_t>(static_cast<std::uint8_t>((*d)[14])) << 16 |
              static_cast<std::uint32_t>(static_cast<std::uint8_t>((*d)[15])) << 24;
    }
    const shm::Beat& b = snap->second;
    const auto age = b.age_ns(shm::monotonic_ns());
    if (!age) {
        println(std::format("{{\"ok\":false,\"why\":\"no heartbeat (older writer)\",\"pid\":{}}}", pid));
        return 1;
    }
    const double age_s = static_cast<double>(*age) * 1e-9;
    const bool fresh = age_s <= max_age;
    const bool ok = fresh && b.source_up;
    const char* why = !fresh ? "stale heartbeat" : !b.source_up ? "source reopening" : "ok";
    println(std::format("{{\"ok\":{},\"why\":\"{}\",\"pid\":{},\"age_s\":{},\"source_up\":{},\"led_frames\":{},"
                        "\"reopens\":{}}}",
                        ok, why, pid, rf::fixed(age_s, 2), b.source_up, b.led_frames, b.reopens));
    return ok ? 0 : 2;
}

} // namespace pibridge::ctrltool
