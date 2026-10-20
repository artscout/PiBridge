// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/sim.rs (MIT OR Apache-2.0); the simulated chip of the on-chip mode is new.
#include "sim.h"

#include "pibridge/eye/frames.h"
#include "pibridge/eyemath/correction.h"
#include "pibridge/eyemath/rng.h"

#include <cmath>
#include <deque>
#include <numbers>

namespace pibridge::eyecal {

namespace em = pibridge::eyemath;
namespace cal = pibridge::eye::calibration;

namespace {

double rad(double d) { return d * std::numbers::pi / 180.0; }

double dist(const Vec3& a, const Vec3& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

/// The simulated chip: answers after the captured delays.
struct SimChip {
    struct Due {
        double at;
        std::uint32_t seq;
        cal::Step step;
    };
    std::deque<Due> due;
    std::uint32_t next_seq = cal::FIRST_SEQ;
    bool calibrated = false;
    std::optional<std::uint32_t> refuse;
    std::vector<SimRequest> log;

    void submit(const cal::Step& s, double t, const Vec3& gaze) {
        const double delay = s.op == cal::OP_TARGET ? 0.165 : s.op == cal::OP_COMPUTE ? 0.5 : s.op == cal::OP_DOWNLOAD ? 0.08 : 0.007;
        due.push_back(Due{t + delay, next_seq, s});
        log.push_back(SimRequest{s, next_seq, t, gaze});
        next_seq++;
    }

    /// The answers due by `t`.
    std::vector<std::pair<std::uint32_t, cal::Answer>> answers(double t) {
        std::vector<std::pair<std::uint32_t, cal::Answer>> out;
        while (!due.empty() && due.front().at <= t) {
            const Due d = due.front();
            due.pop_front();
            cal::Answer a{d.seq, d.step.op, refuse == d.step.op ? 0U : 1U, 0, {}};
            if (d.step.op == cal::OP_DOWNLOAD) {
                a.payload = pibridge::eye::frames::profile_payload(sim_profile_body());
            }
            if (d.step.op == cal::OP_COMPUTE && a.status == 1) {
                calibrated = true;
            }
            out.emplace_back(d.seq, std::move(a));
        }
        return out;
    }
};

} // namespace

Tangent distort(std::size_t eye, Tangent t) {
    const double u = t.u;
    const double v = t.v;
    const Tangent off = eye == 0 ? Tangent{0.13, -0.17} : Tangent{-0.12, -0.18};
    return {off.u + 0.62 * u + 0.04 * v, off.v + 0.03 * u + 0.62 * v - 0.6 * std::max(v, 0.0) * v};
}

pibridge::eye::Bytes sim_profile_body() {
    pibridge::eye::Bytes b(33887);
    for (std::size_t i = 0; i < b.size(); i++) {
        b[i] = static_cast<std::uint8_t>(i * 7 + 3);
    }
    const auto lead = static_cast<std::uint32_t>(b.size() - 168);
    for (int i = 0; i < 4; i++) {
        b[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(lead >> (8 * i));
    }
    return b;
}

SimOutcome run(Procedure proc, const SimParams& p) {
    em::Rng rng(p.seed);
    SimOutcome out{{}, proc, std::nullopt, {}, std::nullopt, false, false, std::nullopt, {0.0, 0.0, -2.0}};
    std::optional<double> grid_since; // back in the check grid after the save
    SimChip chip;
    chip.refuse = p.chip_refuse;
    const double rate = 120.0;
    const double frame_dt = 1.0 / 72.0;
    double t = 0.0;
    double next_sample = 0.0;
    std::optional<Vec3> gaze_target; // where the eyes are now (head-frame point)
    std::optional<std::pair<Vec3, double>> pending; // new point, time it was seen
    std::optional<std::tuple<Vec3, Vec3, double>> saccade;
    double next_blink = p.blink_every_s * (0.5 + rng.uniform());
    double blink_until = -1.0;
    bool turned = false;
    double head_yaw = 0.0;
    bool started = false;
    Vec3 eye_point{0.0, 0.0, -2.0};
    const std::array<Vec3, 2> eye_pos = {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}};
    proc.set_chip_ready(true);
    const auto handle = [&](std::vector<Effect> fx) {
        for (auto& e : fx) {
            switch (e.kind) {
            case Effect::Kind::Print: out.log.push_back(e.text); break;
            case Effect::Kind::SaveCalibration: out.saved = e.text; break;
            case Effect::Kind::ChipRequest: chip.submit(e.step, t, eye_point); break;
            case Effect::Kind::SaveProfile: out.profile = e.profile; break;
            case Effect::Kind::KeepProfile: out.profile_kept = true; break;
            case Effect::Kind::Quit: out.quit = true; break;
            default: break;
            }
        }
    };
    while (t < 400.0) {
        t += frame_dt;
        // the wearer's target: the first mark of the scene, plus the planned glance error
        const Scene scene = proc.scene();
        std::optional<Vec3> want;
        if (!scene.marks.empty()) {
            want = scene.marks.front().pos;
        }
        if (p.glance && proc.phase.is(Phase::Kind::Point) && proc.phase.set == Set::Calibration &&
            proc.phase.i == p.glance->first && want) {
            const auto tn = em::tangent_of(*want).value_or(Tangent{0.0, 0.0});
            const Vec3 d = em::dir_of(tn.u + std::tan(rad(p.glance->second)), tn.v);
            want = Vec3{d[0] * 2.0, d[1] * 2.0, d[2] * 2.0};
        }
        if (want) {
            const bool same = gaze_target && dist(*gaze_target, *want) < 1e-6;
            const bool pend = pending && dist(pending->first, *want) < 1e-6;
            if (!same && !pend) {
                pending = std::pair{*want, t};
            }
        }
        if (pending && t - pending->second >= p.latency_s) {
            saccade = std::tuple{gaze_target.value_or(pending->first), pending->first, t};
            pending.reset();
        }
        // samples since the last frame
        std::vector<RawSample> batch;
        while (next_sample <= t) {
            const double ts = next_sample;
            next_sample += 1.0 / rate;
            Vec3 point{0.0, 0.0, -2.0};
            if (saccade && ts - std::get<2>(*saccade) < 0.04) {
                const double f = std::clamp((ts - std::get<2>(*saccade)) / 0.04, 0.0, 1.0);
                const Vec3& a = std::get<0>(*saccade);
                const Vec3& b = std::get<1>(*saccade);
                point = {a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f};
            } else if (saccade) {
                gaze_target = std::get<1>(*saccade);
                point = *gaze_target;
            } else {
                point = gaze_target.value_or(Vec3{0.0, 0.0, -2.0});
            }
            eye_point = point;
            if (ts >= next_blink) {
                blink_until = ts + 0.12;
                next_blink = ts + p.blink_every_s * (0.5 + rng.uniform());
            }
            const bool blinking = ts < blink_until;
            const bool edge = !blinking && std::fabs(ts - blink_until) < 0.03;
            RawSample rs;
            rs.t = ts;
            for (std::size_t e = 0; e < 2; e++) {
                if (blinking || rng.uniform() < p.drop) {
                    continue;
                }
                const Vec3& o = eye_pos[e];
                const auto tt = em::tangent_of({point[0] - o[0], point[1] - o[1], point[2] - o[2]});
                if (!tt) {
                    continue;
                }
                Tangent g = distort(e, *tt);
                if (chip.calibrated) {
                    // the chip's own calibration: right apart from a constant offset
                    const auto& off = p.chip_offset_deg[e];
                    g = Tangent{std::tan(std::atan(tt->u) + rad(off.u)), std::tan(std::atan(tt->v) + rad(off.v))};
                }
                const double sd = rad(p.noise_deg);
                g.u += sd * rng.gauss();
                g.v += sd * rng.gauss();
                if (edge) {
                    g.v -= rad(3.0); // the lid pulls the pupil estimate down
                }
                rs.eyes[e] = RawEye{em::dir_of(g.u, g.v), {o[0], o[1], o[2] + 0.02}, 3.5};
            }
            batch.push_back(rs);
        }
        if (p.head_turn && !turned && t >= p.head_turn->first) {
            turned = true;
            head_yaw = p.head_turn->second;
        }
        const double half = rad(head_yaw) * 0.5;
        const std::array<double, 4> head = {0.0, std::sin(half), 0.0, std::cos(half)};
        handle(proc.frame(t, head, eye_pos, batch, std::nullopt));
        for (auto& [seq, a] : chip.answers(t)) {
            handle(proc.chip_answer(seq, a));
        }
        if (!started && t > 0.5) {
            started = true;
            handle(proc.command(Cmd::Next));
        }
        if (!grid_since && started && proc.phase.is(Phase::Kind::Results) && t > 2.0 && !proc.chip_waiting()) {
            if (p.stop_at_end) {
                break;
            }
            if (!proc.grid_flow()) {
                handle(proc.command(Cmd::Accept));
                break;
            }
            handle(proc.command(Cmd::Redo)); // B: save, back to the check grid
            grid_since = t;
        }
        if (grid_since && t - *grid_since >= 1.0) {
            out.grid_gaze = proc.gaze();
            out.grid_look = eye_point;
            handle(proc.command(Cmd::Redo)); // B in the grid: exit
            break;
        }
        if (proc.phase.is(Phase::Kind::Intro) && started && !proc.calibration && t > 1.0) {
            // restarted (axis fix): start again
            handle(proc.command(Cmd::Next));
        }
    }
    out.procedure = std::move(proc);
    out.chip_requests = std::move(chip.log);
    return out;
}

} // namespace pibridge::eyecal
