// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/sim.rs tests (MIT OR Apache-2.0); the on-chip mode tests are new.
#include "report.h"
#include "sim.h"

#include "pibridge/eye/calibration.h"
#include "pibridge/eyemath/correction.h"
#include "pibridge/eyemath/rng.h"

#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>

using namespace pibridge::eyecal;
namespace cal = pibridge::eye::calibration;
namespace em = pibridge::eyemath;

namespace {

std::string joined(const std::vector<std::string>& v) {
    std::string s;
    for (const auto& l : v) {
        s += l + "\n";
    }
    return s;
}

bool any_line(const SimOutcome& o, std::string_view part) {
    return std::ranges::any_of(o.log, [&](const std::string& l) { return l.find(part) != std::string::npos; });
}

double dist(const Vec3& a, const Vec3& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

double deg(double r) { return r * 180.0 / std::numbers::pi; }

double angle_between(const Vec3& a, const Vec3& b) {
    const double la = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const double lb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    return deg(std::acos(std::clamp((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb), -1.0, 1.0)));
}

// drive the calibration part with x-mirrored raw samples and stop at the first restart request (sim.rs run_mirrored)
SimOutcome run_mirrored(Procedure proc) {
    std::vector<std::string> log;
    em::Rng rng(3);
    double t = 0.0;
    bool started = false;
    const std::array<Vec3, 2> eye_pos = {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}};
    Vec3 last_target{0.0, 0.0, -2.0};
    double since = 0.0;
    while (t < 120.0) {
        t += 1.0 / 72.0;
        const auto sc = proc.scene();
        const Vec3 want = sc.marks.empty() ? Vec3{0.0, 0.0, -2.0} : sc.marks.front().pos;
        if (dist(want, last_target) > 1e-6) {
            last_target = want;
            since = t;
        }
        std::vector<RawSample> batch;
        for (int k = 0; k < 2; k++) {
            const double ts = t - k / 120.0;
            RawSample rs;
            rs.t = ts;
            for (std::size_t e = 0; e < 2; e++) {
                const Vec3 o = eye_pos[1 - e];
                const Vec3 p = t - since < 0.2 ? Vec3{0.0, 0.0, -2.0} : want;
                const auto uv = em::tangent_of({p[0] - o[0], p[1] - o[1], p[2] - o[2]});
                const auto d = distort(e, *uv);
                const double s = 0.3 * std::numbers::pi / 180.0;
                rs.eyes[e] = RawEye{em::dir_of(-d.u + s * rng.gauss(), d.v + s * rng.gauss()), o, 3.0};
            }
            batch.push_back(rs);
        }
        std::ranges::sort(batch, [](const RawSample& a, const RawSample& b) { return a.t < b.t; });
        for (const auto& e : proc.frame(t, std::array<double, 4>{0.0, 0.0, 0.0, 1.0}, eye_pos, batch, std::nullopt)) {
            if (e.kind == Effect::Kind::Print) {
                log.push_back(e.text);
            }
        }
        if (!started && t > 0.5) {
            started = true;
            (void)proc.command(Cmd::Next);
        }
        if (std::ranges::any_of(log, [](const std::string& l) { return l.find("restarting the tracker") != std::string::npos; })) {
            break;
        }
    }
    return SimOutcome{log, proc, std::nullopt, {}, std::nullopt, false, false, std::nullopt, {0.0, 0.0, -2.0}};
}

} // namespace

TEST(full_run_calibrates_the_simulated_chip_to_about_a_degree) {
    const auto out = run(Procedure(Settings{}), SimParams{});
    const auto& p = out.procedure;
    const auto s = summary(p);
    CHECK(p.val_res.size() == 10);
    CHECK(s.raw.mean > 6.0);
    CHECK(s.accuracy.mean < 1.0);
    if (!(s.accuracy.mean < 1.0)) {
        std::fprintf(stderr, "accuracy %.2f\n%s", s.accuracy.mean, joined(out.log).c_str());
    }
    CHECK(out.saved.has_value());
    if (out.saved) {
        const auto c = em::Calibration::parse(*out.saved);
        CHECK(c.has_value() && c->conv.flip_x); // the conversion of the run is stored
        CHECK(out.saved->find("# validated") != std::string::npos);
    }
}

TEST(the_pimax_path_calibrates_and_its_end_screen_has_two_buttons) {
    const Settings s = Settings{}.pimax();
    const auto out = run(Procedure(s), SimParams{});
    const auto& p = out.procedure;
    CHECK(p.calibration_targets().size() == 11);
    CHECK(std::all_of(p.calibration_targets().begin() + 6, p.calibration_targets().end(),
                      [](const Target& t) { return t.bright; })); // the second half is on white
    CHECK(summary(p).accuracy.mean < 1.3);
    // B at the end: save and exit
    SimParams p5;
    p5.seed = 5;
    auto p2 = run(Procedure(s), p5).procedure;
    CHECK(p2.phase.is(Phase::Kind::Results));
    auto fx = p2.command(Cmd::Redo);
    CHECK(std::ranges::any_of(fx, [](const Effect& e) { return e.kind == Effect::Kind::SaveCalibration; }));
    CHECK(std::ranges::any_of(fx, [](const Effect& e) { return e.kind == Effect::Kind::Quit; }));
    // A at the end: the calibration again, at once (no intro)
    SimParams p6;
    p6.seed = 6;
    auto p3 = run(Procedure(s), p6).procedure;
    fx = p3.command(Cmd::Next);
    CHECK(!std::ranges::any_of(fx, [](const Effect& e) {
        return e.kind == Effect::Kind::SaveCalibration || e.kind == Effect::Kind::Quit;
    }));
    CHECK(p3.phase.is(Phase::Kind::Point) && p3.phase.set == Set::Calibration && p3.phase.i == 0);
    // B at the start: exit
    Procedure p4(Settings{}.pimax());
    fx = p4.command(Cmd::Redo);
    CHECK(std::ranges::any_of(fx, [](const Effect& e) { return e.kind == Effect::Kind::Quit; }));
    // during the run the trigger and B do nothing
    CHECK(p3.command(Cmd::Start).empty());
    CHECK(p3.command(Cmd::Redo).empty());
    CHECK(p3.phase.is(Phase::Kind::Point) && p3.phase.set == Set::Calibration && p3.phase.i == 0);
}

TEST(the_pimax_path_remeasures_the_right_target_and_never_loops) {
    // a 7 degree glance at the white left target (index 6, after the centre shown twice): the re-measure must be that
    // target, not the one at the same index of the results (the merged centre), and the run must end
    SimParams sp;
    sp.glance = std::pair<std::size_t, double>{6, 7.0};
    sp.seed = 4;
    const auto out = run(Procedure(Settings{}.pimax()), sp);
    CHECK(any_line(out, "-15,+0 is")); // the glance is caught
    const auto again =
        std::ranges::count_if(out.log, [](const std::string& l) { return l.starts_with("Calibration 7/11 (-15,+0)"); });
    CHECK(again >= 2); // target 7/11 (-15,+0) measured again
    CHECK(out.procedure.phase.is(Phase::Kind::Results)); // the run ends
    if (again < 2 || !out.procedure.phase.is(Phase::Kind::Results)) {
        std::fprintf(stderr, "%s", joined(out.log).c_str());
    }
}

TEST(a_head_turn_restarts_the_target_and_a_glance_is_remeasured) {
    SimParams sp;
    sp.head_turn = std::pair{5.0, 6.0};
    sp.glance = std::pair<std::size_t, double>{3, 7.0};
    sp.seed = 9;
    const auto out = run(Procedure(Settings{}), sp);
    const auto& p = out.procedure;
    CHECK(p.head_restarts >= 1);
    CHECK(any_line(out, "measuring it again")); // the 7 degree glance must be caught
    CHECK(summary(p).accuracy.mean < 1.3);
}

TEST(a_mirrored_x_axis_is_detected_and_fixed) {
    // the tracker of this run uses the wrong FLIP_X: the simulated chip's x comes out mirrored, which the simulation
    // models by mirroring the true tangent
    Settings s;
    s.conv.flip_x = false;
    const auto out = run_mirrored(Procedure(s));
    CHECK(any_line(out, "restarting the tracker"));
}

// ---------------------------------------------------------------- the on-chip calibration

TEST(the_chip_mode_runs_the_two_captured_passes_while_the_wearer_looks_at_each_target) {
    const auto out = run(Procedure(Settings{}.chip()), SimParams{});
    const auto& p = out.procedure;
    // the captured order, fresh sequence numbers
    const auto seq = cal::sequence();
    CHECK(out.chip_requests.size() == seq.size());
    for (std::size_t i = 0; i < out.chip_requests.size() && i < seq.size(); i++) {
        CHECK(out.chip_requests[i].step == seq[i]);
        CHECK(out.chip_requests[i].seq == cal::FIRST_SEQ + i);
    }
    // every target request went out while the eyes were on that target (shown where the request says)
    double prev_t = -1.0;
    for (const auto& r : out.chip_requests) {
        if (r.step.op != cal::OP_TARGET) {
            continue;
        }
        const Target& tg = p.chip_targets()[*r.step.target];
        CHECK(angle_between(r.gaze, tg.point()) < 1.0);
        CHECK(r.t - prev_t > 1.0); // the dot travels and settles between two targets
        prev_t = r.t;
    }
    // the profile of the last download, then the validation with the calibrated chip: its constant offset is removed
    CHECK(out.profile.has_value() && *out.profile == sim_profile_body());
    CHECK(p.chip_check.has_value());
    if (p.chip_check) {
        for (std::size_t e = 0; e < 2; e++) {
            const auto& x = p.chip_check->eyes[e];
            CHECK(x.has_value());
            if (!x) {
                continue;
            }
            CHECK(x->apply && !x->mirrored_x && !x->mirrored_y);
            CHECK(x->before_deg > 2.0);
            CHECK(x->after_deg < 0.8);
            // the offset is what the simulated chip put in (truth minus gaze)
            const auto off = SimParams{}.chip_offset_deg[e];
            CHECK_NEAR(deg(std::atan(x->offset.u)), -off.u, 0.4);
            CHECK_NEAR(deg(std::atan(x->offset.v)), -off.v, 0.4);
        }
    }
    // B at the end (the simulation's save): the host map is identity + drift, and the profile is kept
    CHECK(out.saved.has_value() && out.profile_kept);
    if (out.saved) {
        const auto c = em::Calibration::parse(*out.saved);
        CHECK(c.has_value());
        if (c) {
            for (std::size_t e = 0; e < 2; e++) {
                CHECK(c->eyes[e].map.is_identity());
                CHECK(std::fabs(c->eyes[e].drift.v) > 0.03);
            }
            CHECK(c->conv.flip_x);
        }
        CHECK(out.saved->find("on-chip calibration") != std::string::npos);
    }
    CHECK(!p.chip_lines().empty() && p.chip_lines()[0] == "CHIP CALIBRATION DONE");
    // then the check grid with the new calibration, and B there exits
    CHECK(p.phase.is(Phase::Kind::GridSaved) && out.quit);
    CHECK(p.scene().grid && p.scene().hud.back() == "A: CALIBRATE AGAIN    B: EXIT");
    if (out.saved) {
        const auto c = em::Calibration::parse(*out.saved);
        CHECK(c.has_value() && p.s.view.has_value());
        if (c && p.s.view) {
            for (std::size_t e = 0; e < 2; e++) {
                CHECK_NEAR(p.s.view->eyes[e].drift.u, c->eyes[e].drift.u, 1e-5);
                CHECK_NEAR(p.s.view->eyes[e].drift.v, c->eyes[e].drift.v, 1e-5);
            }
        }
    }
    // the dot sits where the wearer looks: the chip's constant offset (about 3 degrees) is gone through the new map
    CHECK(out.grid_gaze.has_value());
    if (out.grid_gaze) {
        CHECK(angle_between(*out.grid_gaze, out.grid_look) < 1.0);
        if (!(angle_between(*out.grid_gaze, out.grid_look) < 1.0)) {
            std::fprintf(stderr, "grid gaze %.2f deg off\n", angle_between(*out.grid_gaze, out.grid_look));
        }
    }
}

TEST(after_a_save_the_grid_runs_again_with_the_saved_calibration) {
    // the end screen, B: save, back to the grid; A there: a fresh run that keeps the new host map for its grid
    SimParams sp;
    sp.seed = 3;
    sp.stop_at_end = true;
    auto out = run(Procedure(Settings{}.chip()), sp);
    auto& p = out.procedure;
    CHECK(p.phase.is(Phase::Kind::Results));
    CHECK(p.scene().hud.back() == "A: AGAIN    B: SAVE");
    std::vector<Effect> fx = p.command(Cmd::Redo);
    const auto has = [&](Effect::Kind k) { return std::ranges::any_of(fx, [&](const Effect& e) { return e.kind == k; }); };
    CHECK(has(Effect::Kind::SaveCalibration) && has(Effect::Kind::KeepProfile) && !has(Effect::Kind::Quit));
    CHECK(p.phase.is(Phase::Kind::GridSaved) && p.status() == "SAVED: THE DOT USES THE NEW CALIBRATION");
    CHECK(p.calibration.has_value() && p.s.view == p.calibration);
    const auto saved_view = p.s.view;
    fx = p.command(Cmd::Next);
    CHECK(has(Effect::Kind::ChipRequest) && p.phase.is(Phase::Kind::ChipBusy));
    CHECK(p.s.view == saved_view && !p.calibration.has_value() && p.chip_res.empty());
    // B in that run: back to the grid after the save (not the first one), the saved map still in use
    fx = p.command(Cmd::Redo);
    CHECK(p.phase.is(Phase::Kind::GridSaved) && !has(Effect::Kind::Quit) && p.s.view == saved_view);
    CHECK(p.scene().hud.back() == "A: CALIBRATE AGAIN    B: EXIT");
}

TEST(recenter_through_the_grid_saves_and_returns_to_the_grid) {
    Settings s = Settings{}.pimax();
    s.mode = Mode::Recenter;
    em::Calibration c = em::Calibration::identity(s.conv, 0.064);
    s.existing = c;
    s.view = c;
    const auto out = run(Procedure(s), SimParams{});
    const auto& p = out.procedure;
    CHECK(p.rec_res.size() == 5);
    CHECK(out.saved.has_value() && out.quit && !out.profile_kept);
    CHECK(p.phase.is(Phase::Kind::GridSaved));
    CHECK(p.scene().hud.back() == "A: RECENTER AGAIN    B: EXIT");
    if (out.saved) {
        const auto saved = em::Calibration::parse(*out.saved);
        CHECK(saved.has_value() && p.s.view.has_value() && p.s.existing.has_value());
        if (saved && p.s.view && p.s.existing) {
            for (std::size_t k = 0; k < 2; k++) {
                CHECK_NEAR(p.s.view->eyes[k].drift.u, saved->eyes[k].drift.u, 1e-5);
                CHECK_NEAR(p.s.existing->eyes[k].drift.v, saved->eyes[k].drift.v, 1e-5);
            }
            CHECK(std::fabs(saved->eyes[0].drift.u - c.eyes[0].drift.u) > 1e-3); // the recentre moved it
        }
    }
    CHECK(out.grid_gaze.has_value());
    // the end screen of the recentre
    SimParams st;
    st.stop_at_end = true;
    auto e = run(Procedure(s), st).procedure;
    CHECK(e.phase.is(Phase::Kind::Results) && e.scene().hud.back() == "A: AGAIN    B: SAVE");
    // A: the five targets again
    (void)e.command(Cmd::Next);
    CHECK(e.phase.is(Phase::Kind::Point) && e.phase.set == Set::Recenter && e.phase.i == 0 && e.rec_res.empty());
}

TEST(the_chip_mode_end_screen_and_the_stop_button) {
    const Settings s = Settings{}.chip();
    std::vector<Effect> fx;
    const auto has = [&](Effect::Kind k) { return std::ranges::any_of(fx, [&](const Effect& e) { return e.kind == k; }); };
    // at the end: B saves the host map, keeps the profile and goes back to the grid; A calibrates again at once
    SimParams sp;
    sp.seed = 3;
    sp.stop_at_end = true;
    auto p = run(Procedure(s), sp).procedure;
    CHECK(p.phase.is(Phase::Kind::Results));
    fx = p.command(Cmd::Redo);
    CHECK(has(Effect::Kind::SaveCalibration) && has(Effect::Kind::KeepProfile) && !has(Effect::Kind::Quit));
    CHECK(p.phase.is(Phase::Kind::GridSaved));
    fx = p.command(Cmd::Redo); // B in the grid: exit
    CHECK(has(Effect::Kind::Quit) && !has(Effect::Kind::SaveCalibration));
    auto p2 = run(Procedure(s), sp).procedure;
    fx = p2.command(Cmd::Next);
    CHECK(!has(Effect::Kind::Quit) && !has(Effect::Kind::KeepProfile));
    CHECK(p2.phase.is(Phase::Kind::ChipBusy)); // the first start request is out again
    CHECK(std::ranges::any_of(fx, [](const Effect& e) { return e.kind == Effect::Kind::ChipRequest && e.step.op == cal::OP_START; }));
    // during the run: B finishes the open pass (0x3fc) and goes back to the grid without keeping anything
    Procedure q(s);
    q.set_chip_ready(true);
    fx = q.command(Cmd::Next);
    CHECK(has(Effect::Kind::ChipRequest) && fx.back().step.op == cal::OP_START);
    (void)q.chip_answer(cal::FIRST_SEQ, cal::Answer{cal::FIRST_SEQ, cal::OP_START, 1, 0, {}});
    CHECK(q.phase.is(Phase::Kind::Point) && q.phase.set == Set::Chip && q.phase.i == 0);
    CHECK(q.command(Cmd::Next).empty()); // A does nothing while it runs
    fx = q.command(Cmd::Redo);
    CHECK(!has(Effect::Kind::Quit) && !has(Effect::Kind::KeepProfile) && !has(Effect::Kind::SaveCalibration));
    CHECK(std::ranges::any_of(fx, [](const Effect& e) { return e.kind == Effect::Kind::ChipRequest && e.step.op == cal::OP_FINISH; }));
    CHECK(q.phase.is(Phase::Kind::Grid));
    // B at the start exits; A waits for the eye tracker
    Procedure r(s);
    fx = r.command(Cmd::Next);
    CHECK(fx.empty() && r.phase.is(Phase::Kind::Grid) && r.status() == "WAITING FOR THE EYE TRACKER");
    fx = r.command(Cmd::Redo);
    CHECK(has(Effect::Kind::Quit));
}

TEST(a_refused_chip_request_fails_the_calibration_and_finishes_the_open_pass) {
    SimParams sp;
    sp.chip_refuse = cal::OP_COMPUTE;
    sp.stop_at_end = true;
    const auto out = run(Procedure(Settings{}.chip()), sp);
    const auto& p = out.procedure;
    CHECK(p.chip_failed);
    CHECK(p.phase.is(Phase::Kind::Results));
    CHECK(!out.profile.has_value() && !out.profile_kept);
    CHECK(!out.saved.has_value());
    // start, five targets, the refused compute, then the finish of the open pass
    CHECK(out.chip_requests.size() == 8);
    if (out.chip_requests.size() == 8) {
        CHECK(out.chip_requests[6].step.op == cal::OP_COMPUTE);
        CHECK(out.chip_requests[7].step.op == cal::OP_FINISH);
    }
    CHECK(p.status().find("CALIBRATION FAILED") == 0);
    CHECK(any_line(out, "the open calibration pass was finished"));
    // the end screen: B only goes back to the grid (nothing to save)
    auto q = p;
    auto fx = q.command(Cmd::Redo);
    CHECK(q.phase.is(Phase::Kind::Grid) && std::ranges::none_of(fx, [](const Effect& e) {
              return e.kind == Effect::Kind::Quit || e.kind == Effect::Kind::SaveCalibration || e.kind == Effect::Kind::KeepProfile;
          }));
    // without stop_at_end the simulation goes on the same way: back to the grid, then B exits
    sp.stop_at_end = false;
    const auto all = run(Procedure(Settings{}.chip()), sp);
    CHECK(all.quit && !all.saved && !all.profile_kept && all.procedure.phase.is(Phase::Kind::Grid));
}

PIBRIDGE_TEST_MAIN()
