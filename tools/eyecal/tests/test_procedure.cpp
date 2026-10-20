// SPDX-License-Identifier: MIT
// The procedure's pieces that the simulation does not reach directly: the constant-offset check of the on-chip
// calibration (offset, mirrored axes, too few targets), the chip mode's state handling (stale and failed answers,
// shutdown, a link lost mid-run), the check grid (A / B in every state, the gaze dot through the host map, the
// answers of a stopped run, --recenter's grid), the scene of each stage, the HUD texts and the raw-log tags. (procedure.rs has no Rust tests: the
// sim tests cover it, and their twins are in test_sim.cpp.)
#include "procedure.h"

#include "pibridge/eye/calibration.h"
#include "pibridge/eyemath/correction.h"

#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>

using namespace pibridge::eyecal;
namespace cal = pibridge::eye::calibration;
namespace em = pibridge::eyemath;

namespace {

double rad(double d) { return d * std::numbers::pi / 180.0; }

const std::array<Vec3, 2> EYES = {Vec3{-0.032, 0.0, 0.0}, Vec3{0.032, 0.0, 0.0}};

/// A recorded validation window: each eye sees the target with this map of the true tangent.
PointRes window(const Target& t, const std::function<Tangent(std::size_t, Tangent)>& map) {
    PointRes r;
    r.target = t;
    std::vector<RawSample> w;
    for (int i = 0; i < 60; i++) {
        RawSample s;
        s.t = i / 120.0;
        for (std::size_t e = 0; e < 2; e++) {
            const auto truth = *truth_tangent(t, EYES[e]);
            const auto g = map(e, truth);
            s.eyes[e] = RawEye{em::dir_of(g.u, g.v), EYES[e], 3.5};
        }
        w.push_back(s);
    }
    r.eyes = {analyze_eye(w, 0, FixParams{}), analyze_eye(w, 1, FixParams{})};
    r.samples = w;
    r.ok = true;
    return r;
}

std::vector<PointRes> windows(const std::function<Tangent(std::size_t, Tangent)>& map) {
    std::vector<PointRes> v;
    for (const auto& t : recenter_targets()) {
        v.push_back(window(t, map));
    }
    return v;
}

bool has(const std::vector<Effect>& fx, Effect::Kind k) {
    return std::ranges::any_of(fx, [&](const Effect& e) { return e.kind == k; });
}

bool sends(const std::vector<Effect>& fx, std::uint32_t op) {
    return std::ranges::any_of(fx, [&](const Effect& e) { return e.kind == Effect::Kind::ChipRequest && e.step.op == op; });
}

/// Both eyes look at the head-frame point `p` (raw rays from the eye positions), 120 Hz from `t0` for `secs`.
std::vector<RawSample> looking_at(const Vec3& p, double t0, double secs) {
    std::vector<RawSample> v;
    for (int i = 0; i < static_cast<int>(secs * 120.0); i++) {
        RawSample s;
        s.t = t0 + i / 120.0;
        for (std::size_t e = 0; e < 2; e++) {
            const Vec3 d = {p[0] - EYES[e][0], p[1] - EYES[e][1], p[2] - EYES[e][2]};
            const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            s.eyes[e] = RawEye{{d[0] / n, d[1] / n, d[2] / n}, EYES[e], 3.5};
        }
        v.push_back(s);
    }
    return v;
}

double angle_deg(const Vec3& a, const Vec3& b) {
    const double na = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const double nb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    const double c = std::clamp((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb), -1.0, 1.0);
    return std::acos(c) * 180.0 / std::numbers::pi;
}

/// A chip-mode procedure that runs: A pressed, the start answered, the first target travelling.
Procedure running() {
    Procedure p(Settings{}.chip());
    p.set_chip_ready(true);
    (void)p.command(Cmd::Next);
    (void)p.chip_answer(cal::FIRST_SEQ, cal::Answer{cal::FIRST_SEQ, cal::OP_START, 1, 0, {}});
    return p;
}

Settings recenter_settings() {
    Settings s = Settings{}.pimax();
    s.mode = Mode::Recenter;
    s.existing = em::Calibration::identity(s.conv, 0.064);
    s.view = s.existing;
    return s;
}

} // namespace

TEST(a_constant_offset_is_found_and_removed) {
    // left eye 9.1 degrees low, right eye 13.7 low and 1 right (the offsets of 2026-10-18)
    const auto c = check_chip_validation(windows([](std::size_t e, Tangent t) {
                                             const double du = e == 0 ? 0.0 : 1.0;
                                             const double dv = e == 0 ? -9.1 : -13.7;
                                             return Tangent{std::tan(std::atan(t.u) + rad(du)), std::tan(std::atan(t.v) + rad(dv))};
                                         }),
                                         EYES);
    for (std::size_t e = 0; e < 2; e++) {
        CHECK(c.eyes[e].has_value());
        const auto& x = *c.eyes[e];
        CHECK(x.n == 5 && x.apply && !x.mirrored_x && !x.mirrored_y);
        CHECK(x.after_deg < 0.3);
        CHECK_NEAR(x.offset_deg, e == 0 ? 9.1 : 13.74, 0.3);
        CHECK(x.offset.v > 0.0); // truth minus gaze: the gaze is low
    }
}

TEST(no_offset_a_scattered_error_and_a_mirrored_axis_are_left_alone) {
    // right on: nothing to remove
    auto c = check_chip_validation(windows([](std::size_t, Tangent t) { return t; }), EYES);
    CHECK(c.eyes[0] && !c.eyes[0]->apply && c.eyes[0]->before_deg < 0.1);
    // a gain error (not constant): removing the median offset does not halve it
    c = check_chip_validation(windows([](std::size_t, Tangent t) { return Tangent{t.u * 0.5, t.v * 0.5}; }), EYES);
    CHECK(c.eyes[0] && !c.eyes[0]->apply && c.eyes[0]->before_deg > 1.5);
    // mirrored x (the targets were shown on the wrong side)
    c = check_chip_validation(windows([](std::size_t, Tangent t) { return Tangent{-t.u, t.v - 0.1}; }), EYES);
    CHECK(c.eyes[1] && c.eyes[1]->mirrored_x && !c.eyes[1]->mirrored_y && !c.eyes[1]->apply);
    // fewer than three targets: no verdict for that eye
    auto few = windows([](std::size_t, Tangent t) { return t; });
    few.resize(2);
    c = check_chip_validation(few, EYES);
    CHECK(!c.eyes[0] && !c.eyes[1]);
}

TEST(the_chip_mode_ignores_stale_answers_and_fails_cleanly) {
    Procedure p(Settings{}.chip());
    // no request out: an answer changes nothing
    CHECK(p.chip_answer(5, cal::Answer{5, cal::OP_START, 1, 0, {}}).empty());
    CHECK(p.phase.is(Phase::Kind::Grid));
    p.set_chip_ready(true);
    auto fx = p.command(Cmd::Start); // the trigger starts too
    CHECK(has(fx, Effect::Kind::ChipRequest) && p.chip_waiting());
    // the link was lost before the start was answered: failed, nothing to finish (no pass open)
    fx = p.chip_answer(0, std::unexpected(std::string("the eye chip link ended during the calibration")));
    CHECK(p.chip_failed && p.phase.is(Phase::Kind::Results) && !has(fx, Effect::Kind::ChipRequest));
    CHECK(p.status() == "CALIBRATION FAILED: THE EYE CHIP LINK ENDED DURING THE CALIBRATION");
    CHECK(p.shutdown().empty());
    // A again: a fresh run that starts at once (the eye tracker is still ready)
    fx = p.command(Cmd::Next);
    CHECK(!p.chip_failed && p.phase.is(Phase::Kind::ChipBusy) && has(fx, Effect::Kind::ChipRequest));
    // the start answered with another seq than it was sent with: refused, the open-pass finish is not needed
    fx = p.chip_answer(30, cal::Answer{31, cal::OP_START, 1, 0, {}});
    CHECK(p.chip_failed);
}

TEST(shutdown_finishes_an_open_pass_once) {
    Procedure p(Settings{}.chip());
    p.set_chip_ready(true);
    (void)p.command(Cmd::Next);
    // while the start is out, the app closes: a finish goes out anyway (the chip may open the pass)
    auto fx = p.shutdown();
    CHECK(fx.size() == 1 && fx[0].kind == Effect::Kind::ChipRequest && fx[0].step.op == cal::OP_FINISH);
    CHECK(p.shutdown().empty());
    // its answer is only printed
    fx = p.chip_answer(28, cal::Answer{28, cal::OP_FINISH, 1, 0, {}});
    CHECK(fx.size() == 1 && fx[0].kind == Effect::Kind::Print);
    // the host modes have nothing to finish
    Procedure h(Settings{}.pimax());
    CHECK(h.shutdown().empty());
}

TEST(the_chip_scene_travels_on_black_then_white) {
    Procedure p(Settings{}.chip());
    p.set_chip_ready(true);
    auto sc = p.scene();
    CHECK(sc.hud.back() == "A: CALIBRATE    B: EXIT" && sc.grid && sc.marks.empty());
    (void)p.command(Cmd::Next);
    sc = p.scene();
    CHECK(p.phase.is(Phase::Kind::ChipBusy) && !sc.marks.empty());
    // the start answered: the first target (centre) on the dark background, the green dot
    (void)p.chip_answer(27, cal::Answer{27, cal::OP_START, 1, 0, {}});
    sc = p.scene();
    CHECK(sc.background == BACKGROUND && sc.marks.size() == 3);
    CHECK(sc.marks[0].color[1] > 0.9F); // green
    CHECK(p.tag() == "Chip:0:+0,+0:Attract");
    // a pass 2 target is on white
    const auto& t = p.chip_targets();
    CHECK(t[5].bright && !t[4].bright);
    // no request is out while the dot travels: an answer is ignored
    CHECK(p.chip_answer(0, std::unexpected(std::string("x"))).empty() && p.phase.is(Phase::Kind::Point));
    // the end screen of a failed run says so and offers only exit
    Procedure f(Settings{}.chip());
    f.set_chip_ready(true);
    (void)f.command(Cmd::Next);
    (void)f.chip_answer(0, std::unexpected(std::string("x")));
    sc = f.scene();
    CHECK(sc.hud.size() >= 3);
    if (sc.hud.size() < 3) {
        return;
    }
    CHECK(sc.hud.back() == "A: AGAIN    B: BACK");
    CHECK(!sc.grid);
}

TEST(the_chip_mode_starts_in_the_check_grid_a_calibrates_b_exits) {
    Procedure p(Settings{}.chip());
    CHECK(p.grid_flow() && p.phase.is(Phase::Kind::Grid) && p.tag() == "Grid");
    auto sc = p.scene();
    CHECK(sc.grid && sc.marks.empty() && !sc.gaze); // no gaze yet: no dot
    CHECK(sc.hud.size() == 2 && sc.hud[0] == "WAITING FOR THE EYE TRACKER" && sc.hud[1] == "A: CALIBRATE    B: EXIT");
    // A waits for the eye tracker; the line goes away once it streams
    auto fx = p.command(Cmd::Next);
    CHECK(fx.empty() && p.phase.is(Phase::Kind::Grid) && p.status() == "WAITING FOR THE EYE TRACKER");
    p.set_chip_ready(true);
    CHECK(p.status().empty());
    // menu / Y does nothing in the grid
    CHECK(p.command(Cmd::Accept).empty() && p.phase.is(Phase::Kind::Grid));
    // the trigger starts like A: the captured start request
    fx = p.command(Cmd::Start);
    CHECK(sends(fx, cal::OP_START) && p.phase.is(Phase::Kind::ChipBusy) && p.chip_pending() == 1);
    // B in the grid exits (nothing is saved there)
    Procedure q(Settings{}.chip());
    fx = q.command(Cmd::Redo);
    CHECK(has(fx, Effect::Kind::Quit) && !has(fx, Effect::Kind::SaveCalibration) && !has(fx, Effect::Kind::KeepProfile));
}

TEST(b_during_the_run_stops_and_goes_back_to_the_grid) {
    // while the dot travels (no request out): the open pass is finished, back to the grid, nothing kept, no exit
    auto p = running();
    CHECK(p.phase.is(Phase::Kind::Point) && p.phase.set == Set::Chip);
    CHECK(p.command(Cmd::Next).empty() && p.command(Cmd::Start).empty() && p.command(Cmd::Accept).empty());
    auto fx = p.command(Cmd::Redo);
    CHECK(sends(fx, cal::OP_FINISH) && !has(fx, Effect::Kind::Quit) && !has(fx, Effect::Kind::KeepProfile) &&
          !has(fx, Effect::Kind::SaveCalibration));
    CHECK(p.phase.is(Phase::Kind::Grid) && p.status() == "STOPPED: NOT SAVED");
    CHECK(p.scene().hud.back() == "A: CALIBRATE    B: EXIT");
    // the finish is still out: A waits for its answer (a new run's answers must not meet it)
    CHECK(p.chip_pending() == 1);
    fx = p.command(Cmd::Next);
    CHECK(!has(fx, Effect::Kind::ChipRequest) && p.status() == "PLEASE WAIT" && p.phase.is(Phase::Kind::Grid));
    (void)p.chip_answer(cal::FIRST_SEQ + 1, cal::Answer{cal::FIRST_SEQ + 1, cal::OP_FINISH, 1, 0, {}});
    CHECK(p.chip_pending() == 0 && p.status().empty());
    // now A starts a fresh run
    fx = p.command(Cmd::Next);
    CHECK(sends(fx, cal::OP_START) && p.phase.is(Phase::Kind::ChipBusy) && p.chip_res.empty());
    // B while a request is out (the start): a finish goes out too; both answers are waited for
    fx = p.command(Cmd::Redo);
    CHECK(sends(fx, cal::OP_FINISH) && p.phase.is(Phase::Kind::Grid) && p.chip_pending() == 2);
    (void)p.chip_answer(cal::FIRST_SEQ, cal::Answer{cal::FIRST_SEQ, cal::OP_START, 1, 0, {}});
    CHECK(p.command(Cmd::Next).empty() && p.status() == "PLEASE WAIT");
    CHECK(p.phase.is(Phase::Kind::Grid));
    (void)p.chip_answer(cal::FIRST_SEQ + 1, cal::Answer{cal::FIRST_SEQ + 1, cal::OP_FINISH, 1, 0, {}});
    CHECK(sends(p.command(Cmd::Next), cal::OP_START));
}

TEST(a_failed_run_offers_again_and_back_never_save) {
    Procedure p(Settings{}.chip());
    p.set_chip_ready(true);
    (void)p.command(Cmd::Next);
    (void)p.chip_answer(cal::FIRST_SEQ, std::unexpected(std::string("x")));
    CHECK(p.chip_failed && p.phase.is(Phase::Kind::Results));
    // B: back to the grid, nothing saved or kept, no exit
    auto q = p;
    auto fx = q.command(Cmd::Redo);
    CHECK(q.phase.is(Phase::Kind::Grid) && q.status() == "NOT SAVED");
    CHECK(!has(fx, Effect::Kind::SaveCalibration) && !has(fx, Effect::Kind::KeepProfile) && !has(fx, Effect::Kind::Quit));
    // menu / Y the same
    auto r = p;
    fx = r.command(Cmd::Accept);
    CHECK(r.phase.is(Phase::Kind::Grid) && !has(fx, Effect::Kind::SaveCalibration));
    // A: again at once
    fx = p.command(Cmd::Next);
    CHECK(sends(fx, cal::OP_START) && !p.chip_failed && p.phase.is(Phase::Kind::ChipBusy));
}

TEST(the_grid_shows_the_gaze_through_the_host_map_pibridge_eyed_uses) {
    const Vec3 look = Target::at(8.0, -4.0).point();
    // no host map: the raw gaze (identity per eye) lands on the point looked at
    Procedure p(Settings{}.chip());
    (void)p.frame(1.0, std::nullopt, EYES, looking_at(look, 0.5, 0.5), std::nullopt);
    auto sc = p.scene();
    CHECK(sc.grid && sc.gaze.has_value() && p.gaze().has_value());
    if (!sc.gaze) {
        return;
    }
    CHECK(angle_deg(*sc.gaze, look) < 0.3);
    CHECK(std::fabs(std::sqrt((*sc.gaze)[0] * (*sc.gaze)[0] + (*sc.gaze)[1] * (*sc.gaze)[1] + (*sc.gaze)[2] * (*sc.gaze)[2]) -
                    TARGET_DIST_M) < 1e-6); // the dot hangs at the target distance
    // a host map with a drift of 3 degrees to the right for both eyes: the dot moves with it
    Settings s = Settings{}.chip();
    em::Calibration c = em::Calibration::identity(s.conv, 0.064);
    for (std::size_t e = 0; e < 2; e++) {
        c.eyes[e].origin_m = EYES[e];
        c.eyes[e].drift.u = std::tan(rad(3.0));
    }
    s.view = c;
    Procedure q(s);
    (void)q.frame(1.0, std::nullopt, EYES, looking_at(look, 0.5, 0.5), std::nullopt);
    CHECK(q.gaze().has_value());
    if (q.gaze()) {
        CHECK_NEAR(angle_deg(*q.gaze(), look), 3.0, 0.4);
        CHECK((*q.gaze())[0] / -(*q.gaze())[2] > look[0] / -look[2]); // to the right
    }
    // no sample for a while: no dot
    (void)q.frame(2.0, std::nullopt, EYES, {}, std::nullopt);
    CHECK(!q.scene().gaze && q.scene().hud[0] == "WAITING FOR THE EYE TRACKER");
}

TEST(recenter_starts_and_ends_in_the_grid) {
    Procedure p(recenter_settings());
    CHECK(p.grid_flow() && p.phase.is(Phase::Kind::Grid));
    auto sc = p.scene();
    CHECK(sc.grid && sc.hud.back() == "A: RECENTER    B: EXIT");
    // A: the five targets (no eye tracker check: the recentre sends nothing to the chip)
    auto fx = p.command(Cmd::Next);
    CHECK(p.phase.is(Phase::Kind::Point) && p.phase.set == Set::Recenter && p.phase.i == 0);
    CHECK(!has(fx, Effect::Kind::ChipRequest));
    // B during the targets: back to the grid, nothing saved
    fx = p.command(Cmd::Redo);
    CHECK(p.phase.is(Phase::Kind::Grid) && !has(fx, Effect::Kind::Quit) && !has(fx, Effect::Kind::SaveCalibration));
    // B in the grid exits
    fx = p.command(Cmd::Redo);
    CHECK(has(fx, Effect::Kind::Quit));
    // the --pattern grid recentre keeps the Rust tool's intro
    Settings g = recenter_settings();
    g.pattern = Pattern::Grid;
    CHECK(Procedure(g).phase.is(Phase::Kind::Intro) && !Procedure(g).grid_flow());
    // the host fit is not a grid flow either
    CHECK(!Procedure(Settings{}.pimax()).grid_flow() && Procedure(Settings{}.pimax()).phase.is(Phase::Kind::Intro));
}

PIBRIDGE_TEST_MAIN()
