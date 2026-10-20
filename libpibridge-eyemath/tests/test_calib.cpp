// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/calib.rs tests (MIT OR Apache-2.0).
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include "pibridge/eyemath/calib.h"
#include "pibridge/eyemath/rng.h"
#include "pibridge_test.h"
#include "test_util.h"

using namespace pibridge::eyemath;

namespace {

double rad(double deg) { return deg * (std::numbers::pi / 180.0); }

/// A fake eye: true tangent -> raw tangent (what the uncalibrated chip reports).
Tangent distort(Tangent t, std::size_t eye) {
    // gain 0.62, offsets per eye (kappa-like), upper field compressed (quadratic in v), slight cross talk
    const double u = t.u;
    const double v = t.v;
    const Tangent off = eye == 0 ? Tangent{0.05, -0.17} : Tangent{-0.07, -0.16};
    return {off.u + 0.62 * u + 0.03 * v, off.v + 0.02 * u + 0.6 * v - 0.5 * std::fmax(v, 0.0) * v};
}

std::vector<Tangent> grid(std::initializer_list<double> az, std::initializer_list<double> el) {
    std::vector<Tangent> v;
    for (double a : az) {
        for (double e : el) {
            v.push_back({std::tan(rad(a)), std::tan(rad(e))});
        }
    }
    return v;
}

std::vector<EyePoint> points(std::size_t eye, const std::vector<Tangent>& truths, double noise, Rng& rng) {
    std::vector<EyePoint> out;
    for (const auto& t : truths) {
        const auto r = distort(t, eye);
        // the Rust tuple literal evaluates the two gauss() calls left to right
        const double nu = noise * rng.gauss();
        const double nv = noise * rng.gauss();
        out.push_back({{r.u + nu, r.v + nv}, t, 1.0});
    }
    return out;
}

Vec3 add(Vec3 a, Vec3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec3 scale(Vec3 a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
Vec3 unit(Vec3 a) { return scale(a, 1.0 / std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])); }
double angle_deg(Vec3 a, Vec3 b) {
    const Vec3 c{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    return std::atan2(std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]), a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) *
           (180.0 / std::numbers::pi);
}
Vec3 f64v(Vec3f v) { return {double{v[0]}, double{v[1]}, double{v[2]}}; }
Vec3f f32v(Vec3 v) { return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])}; }

Sample sample(std::optional<Vec3> l, std::optional<Vec3> r) {
    Sample s;
    const std::array<std::optional<Vec3>, 2> dirs{l, r};
    for (std::size_t i = 0; i < 2; i++) {
        if (dirs[i]) {
            EyeRay e;
            e.origin = {i == 0 ? -0.03F : 0.03F, 0.0F, 0.02F};
            e.direction = f32v(*dirs[i]);
            e.origin_valid = true;
            e.direction_valid = true;
            e.pupil_mm = 3.0F;
            s.eyes[i] = e;
        }
    }
    return s;
}

bool contains(const std::string& s, std::string_view what) { return s.find(what) != std::string::npos; }

} // namespace

TEST(affine_recovers_gain_and_offset_exactly_without_noise) {
    const auto truths = grid({-15.0, 0.0, 15.0}, {-12.0, 0.0, 12.0});
    // affine-only distortion
    std::vector<EyePoint> pts;
    for (const auto& t : truths) {
        pts.push_back({{0.05 + 0.6 * t.u, -0.17 + 0.62 * t.v + 0.02 * t.u}, t, 1.0});
    }
    const auto c = fit_eye(pts, ModelKind::Affine, Ridge{0.0, 0.0});
    CHECK(c.has_value());
    for (const auto& p : pts) {
        CHECK(point_error_deg(*c, p) < 1e-6);
    }
}

TEST(selection_beats_raw_and_validates_on_new_points) {
    Rng seed(0x1234'5678'9abc'def0ULL);
    const auto cal_t = grid({-15.0, 0.0, 15.0}, {-12.0, 0.0, 12.0});
    const auto val_t = grid({-10.0, -4.0, 4.0, 10.0}, {-8.0, 3.0, 9.0});
    for (std::size_t eye = 0; eye < 2; eye++) {
        // 0.15 deg of noise on each target centre (the median of ~100 samples)
        const auto pts = points(eye, cal_t, rad(0.15), seed);
        const auto sel = select_model(pts);
        const auto* best = sel.best();
        CHECK(best != nullptr);
        if (best == nullptr) {
            return;
        }
        CHECK(sel.raw.mean > 8.0);
        std::vector<double> val;
        for (const auto& t : val_t) {
            const auto r = distort(t, eye);
            val.push_back(tangent_angle_deg(best->map.map_tangent(r.u, r.v), t));
        }
        const auto v = stats(val);
        CHECK(v.mean < 0.8);
        CHECK(best->kind != ModelKind::Offset); // gain 0.6 needs more than an offset
    }
}

TEST(one_bad_target_does_not_drag_the_fit) {
    Rng seed(42);
    const auto cal_t = grid({-15.0, 0.0, 15.0}, {-12.0, 0.0, 12.0});
    auto pts = points(0, cal_t, rad(0.1), seed);
    const auto clean = *fit_eye(pts, ModelKind::Affine, Ridge{});
    // the wearer looked 6 degrees away at one target
    pts[4].raw.u += std::tan(rad(6.0)) * 0.62;
    const auto robust = *fit_eye(pts, ModelKind::Affine, Ridge{});
    std::vector<double> w(pts.size(), 1.0);
    w[4] = 1.0;
    const auto plain = *detail::fit_weighted(pts, w, ModelKind::Affine, Ridge{});
    const auto err = [&](const Correction& c) {
        double s = 0.0;
        for (std::size_t i = 0; i < pts.size(); i++) {
            if (i != 4) {
                s += point_error_deg(c, pts[i]);
            }
        }
        return s / 8.0;
    };
    CHECK(err(robust) < err(plain) * 0.7);
    CHECK(err(robust) < err(clean) + 0.6);
}

TEST(quadratic_wins_when_the_upper_field_is_compressed_and_points_allow_it) {
    Rng seed(7);
    const auto cal_t = grid({-15.0, -7.5, 0.0, 7.5, 15.0}, {-12.0, 0.0, 12.0});
    const auto pts = points(1, cal_t, rad(0.05), seed);
    const auto sel = select_model(pts);
    CHECK(sel.best() != nullptr && sel.best()->kind == ModelKind::Quadratic);
    const auto aff = std::ranges::find_if(sel.candidates, [](const ModelFit& m) { return m.kind == ModelKind::Affine; });
    CHECK(aff != sel.candidates.end());
    CHECK(sel.best()->loo.mean < aff->loo.mean);
}

TEST(toml_round_trip) {
    FrameConv conv;
    conv.flip_x = true;
    auto c = Calibration::identity(conv, 0.0655);
    c.eyes[0].map.cx = {0.01, 1.2, 0.03, 0.0, 0.1, -0.2};
    c.eyes[0].map.model = "quadratic";
    c.eyes[1].map.cy = {-0.2, 0.01, 1.6, 0.0, 0.0, 0.0};
    c.eyes[1].drift = {0.001, -0.002};
    c.eyes[1].accuracy_deg = 0.8;
    c.filter.enabled = false;
    c.combine.depth_m = 2.0;
    c.created_unix = 1'791'956'330;
    c.notes = {"validation mean 1.0 deg"};
    const auto t = c.to_toml();
    CHECK(contains(t, "# validation mean 1.0 deg"));
    const auto back = Calibration::parse(t);
    CHECK(back.has_value());
    CHECK(back->conv == c.conv);
    CHECK(back->created_unix == c.created_unix);
    CHECK(!back->filter.enabled);
    for (std::size_t i = 0; i < 2; i++) {
        for (std::size_t k = 0; k < 6; k++) {
            CHECK_NEAR(back->eyes[i].map.cx[k], c.eyes[i].map.cx[k], 1e-9);
            CHECK_NEAR(back->eyes[i].map.cy[k], c.eyes[i].map.cy[k], 1e-9);
        }
        CHECK(back->eyes[i].map.model == c.eyes[i].map.model);
    }
    CHECK_NEAR(back->eyes[1].drift.v, -0.002, 1e-12);
    CHECK_NEAR(back->combine.depth_m, 2.0, 1e-12);
    CHECK(!Calibration::parse("version = 1\n[left]\nmodel = \"affine\"\n").has_value());
    CHECK(!Calibration::parse("version = 2\n[left]\n[right]\n").has_value());
    CHECK(!Calibration::parse("[left]\nmodel = \"cubic\"\n[right]\n").has_value());
}

TEST(combination_is_vergence_consistent_and_survives_one_eye) {
    const auto cal = Calibration::identity(FrameConv{}, 0.064);
    const Vec3 target{0.3, 0.1, -1.5};
    const auto dir_from = [&](Vec3 o) { return unit(add(target, scale(o, -1.0))); };
    const auto l = dir_from(cal.eyes[0].origin_m);
    const auto r = dir_from(cal.eyes[1].origin_m);
    const auto want = unit(target);
    CombineState st;
    auto s = sample(l, r);
    cal.apply_sample(s, st);
    auto g = *s.gaze;
    CHECK(g.eyes == 3);
    // the 1.5 m fixation depth vs the 1.53 m target: a few hundredths of a degree
    CHECK(angle_deg(f64v(g.direction), want) < 0.05);
    // the right eye blinks: the left eye alone gives the same direction (no half-vergence jump)
    s = sample(l, std::nullopt);
    cal.apply_sample(s, st);
    g = *s.gaze;
    CHECK(g.eyes == 1);
    CHECK(angle_deg(f64v(g.direction), want) < 0.05);
    // back from the blink: the right eye is held off for 2 samples, then used again
    for (int k = 0; k < 3; k++) {
        auto s2 = sample(l, r);
        cal.apply_sample(s2, st);
        CHECK(s2.gaze.has_value());
        CHECK(s2.gaze->eyes == (k < 2 ? 1 : 3));
        CHECK(s2.eyes[1].direction_valid); // the eye itself is still published
    }
}

TEST(disagreeing_eyes_fall_back_to_the_better_one) {
    auto cal = Calibration::identity(FrameConv{}, 0.064);
    cal.eyes[0].accuracy_deg = 0.5;
    cal.eyes[1].accuracy_deg = 2.0;
    CombineState st;
    const Vec3 good{0.0, 0.0, -1.0};
    const auto bad = dir_of(0.2, 0.0); // 11 degrees off
    auto s = sample(good, bad);
    cal.apply_sample(s, st);
    const auto g = *s.gaze;
    CHECK(g.eyes == 1); // left eye only
    CHECK(angle_deg(f64v(g.direction), good) < 1.5);
    s = sample(std::nullopt, std::nullopt);
    cal.apply_sample(s, st);
    CHECK(!s.gaze.has_value());
}

TEST(select_follows_the_variable) {
    eyemath_test::TempDir dir("pibridge-cal");
    std::filesystem::create_directories(dir.path / ".config/omarchyvr");
    const std::optional<std::filesystem::path> home = dir.path;
    CHECK(!Calibration::select(std::nullopt, home).value().has_value());
    const auto c = Calibration::identity(FrameConv{}, 0.064);
    eyemath_test::write_file(dir.path / ".config/omarchyvr/eye-calibration.toml", c.to_toml());
    CHECK(Calibration::select(std::nullopt, home).value().has_value());
    CHECK(!Calibration::select("none", home).value().has_value());
    CHECK(!Calibration::select("/nonexistent.toml", home).has_value());
}

TEST(origin_terms_round_trip_shift_and_gate) {
    auto c = Calibration::identity(FrameConv{}, 0.064);
    c.eyes[1].org = OriginTerms{{0.026, 0.003, 0.02}, {0.1, 0.0, -0.05}, {0.0, 0.12, 0.0}, 5.0, 3.3, -0.02, 0.01};
    const auto t = c.to_toml();
    CHECK(contains(t, "version = 2") && contains(t, "origin_gate_mm = 5.000"));
    const auto back = *Calibration::parse(t);
    CHECK(back.eyes[1].org.gate_mm == 5.0);
    CHECK(std::fabs(back.eyes[1].org.u[2] + 0.05) < 1e-9 && std::fabs(back.eyes[1].org.pupil_u + 0.02) < 1e-9);
    CHECK(!back.eyes[0].org.is_active());
    // a file without origin terms stays version 1
    CHECK(contains(Calibration::identity(FrameConv{}, 0.064).to_toml(), "version = 1"));
    // 1 mm of origin error in x moves u by 0.1
    const auto& e = back.eyes[1];
    const auto d = dir_of(0.0, 0.0);
    const Vec3 o{0.027, 0.003, 0.02};
    const auto t1 = *tangent_of(*e.apply_full(d, o, 3.3, {}));
    CHECK(std::fabs(t1.u - 0.1) < 1e-6 && std::fabs(t1.v) < 1e-6);
    // the pupil term is clamped
    const auto t2 = *tangent_of(*e.apply_full(d, o, 9.3, {}));
    CHECK_NEAR(t2.u, 0.1 - 0.02 * PUPIL_CLAMP_MM, 1e-6);
    // far from the reference: the eye is dropped, the other one carries the gaze alone
    auto s = sample(Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, -1.0});
    s.eyes[1].origin = {0.04F, 0.0F, 0.02F};
    CombineState st;
    back.apply_sample(s, st);
    CHECK(!s.eyes[1].direction_valid);
    CHECK(s.gaze.has_value() && s.gaze->eyes == 1);
}

// ---- C++ only: file compatibility with the Rust code ----

// Written by the Rust `Calibration::to_toml` (omarchyvr pimax-eye) for the calibration of the test
// `origin_terms_round_trip_shift_and_gate` plus a quadratic left eye, drift, notes, flips and filter settings
// (version 2; the generator is described in docs/libpibridge-eyemath.md).
constexpr std::string_view RUST_V2_FIXTURE = R"FIX(# pimax eye calibration: per-eye host-side maps (written by pimax-eyecal; the eye chip's own calibration is not touched)
# validation mean 0.71 deg (left 0.80, right 0.62)
# fit: quadratic / affine + origin
# per eye: u' = u . (1, u, v, u*u, u*v, v*v), v' = v . (...) on the tangent (x / -z, y / -z) of the raw head-frame direction; then drift and (version 2) origin_u/v . (origin - origin_ref) in mm are added
version = 2
created_unix = 1791956330
flip = "1,0,1"
swap = 0
offset_mm = [0.500000000, -1.250000000, 0.000000000]
depth_m = 2.0000
max_disagree_deg = 3.500
holdoff = 2

[left]
model = "quadratic"
u = [0.010000000, 1.200000000, 0.030000000, 0.000000000, 0.100000000, -0.200000000]
v = [-0.170000000, 0.020000000, 1.610000000, 0.000000000, -0.050000000, 0.300000000]
origin_m = [-0.032750000, 0.000000000, 0.000000000]
accuracy_deg = 1.0000
precision_deg = 0.5000
drift = [0.001200000, -0.000400000]

[right]
model = "affine"
u = [0.000000000, 1.000000000, 0.000000000, 0.000000000, 0.000000000, 0.000000000]
v = [-0.200000000, 0.010000000, 1.600000000, 0.000000000, 0.000000000, 0.000000000]
origin_m = [0.032750000, 0.000000000, 0.000000000]
accuracy_deg = 0.8000
precision_deg = 0.4100
drift = [0.001000000, -0.002000000]
origin_ref_m = [0.026000000, 0.003000000, 0.020000000]
origin_u = [0.100000000, 0.000000000, -0.050000000]
origin_v = [0.000000000, 0.120000000, 0.000000000]
origin_gate_mm = 5.000
pupil_ref_mm = 3.3000
pupil_uv = [-0.020000000, 0.010000000]

[filter]
kind = "one-euro"
min_cutoff_hz = 1.2000
beta = 0.02500
d_cutoff_hz = 2.0000
snap_deg = 3.000
)FIX";

TEST(rust_v2_fixture_round_trips_byte_for_byte) {
    auto c = Calibration::parse(RUST_V2_FIXTURE);
    CHECK(c.has_value());
    if (!c) {
        std::fprintf(stderr, "%s\n", c.error().message.c_str());
        return;
    }
    // the notes are comments: parse() drops them (as in Rust), so put them back before writing again
    CHECK(c->notes.empty());
    c->notes = {"validation mean 0.71 deg (left 0.80, right 0.62)", "fit: quadratic / affine + origin"};
    CHECK(c->to_toml() == RUST_V2_FIXTURE);
    if (c->to_toml() != RUST_V2_FIXTURE) {
        std::fprintf(stderr, "C++ wrote:\n%s\n", c->to_toml().c_str());
    }
    CHECK(c->eyes[1].org.is_active() && !c->eyes[0].org.is_active());
    CHECK_NEAR(c->eyes[1].org.gate_mm, 5.0, 0.0);
    CHECK_NEAR(c->eyes[1].org.pupil_ref_mm, 3.3, 1e-12);
    CHECK_NEAR(c->eyes[1].org.pupil_v, 0.01, 1e-12);
    CHECK(c->eyes[0].map.model == "quadratic");
    CHECK(c->conv.flip_x && !c->conv.flip_y && c->conv.flip_z && !c->conv.swap_eyes);
}

TEST(save_and_load_round_trip) {
    eyemath_test::TempDir dir("pibridge-cal-save");
    auto c = Calibration::identity(FrameConv{}, 0.064);
    c.eyes[0].org.gate_mm = 6.0;
    const auto p = dir.path / "eye-calibration.toml";
    CHECK(c.save(p).has_value());
    const auto back = Calibration::load(p);
    CHECK(back && back->to_toml() == c.to_toml());
    const auto missing = Calibration::load(dir.path / "missing.toml");
    CHECK(!missing && contains(missing.error().message, "No such file or directory (os error 2)"));
}

TEST(parse_errors_name_the_line) {
    const auto e = Calibration::parse("version = 1\n[left]\nu = [1, x]\n[right]\n");
    CHECK(!e && e.error().message == "line 3: u: invalid float literal");
    const auto e2 = Calibration::parse("version = 3\n");
    CHECK(!e2 && e2.error().message == "line 1: unsupported version");
    const auto e3 = Calibration::parse("depth_m =\n");
    CHECK(!e3 && e3.error().message == "line 1: depth_m: cannot parse float from empty string");
    // values are clamped like the Rust code
    const auto r = Calibration::parse("depth_m = 0.01\nholdoff = 99\n[left]\naccuracy_deg = -1\n[right]\n"
                                      "model = \"affine\"\n[filter]\nsnap_deg = 400\n");
    CHECK(r.has_value());
    if (!r) {
        return;
    }
    const auto& c = *r;
    CHECK(c.combine.depth_m == 0.2);
    CHECK(c.combine.holdoff == 30);
    CHECK(c.eyes[0].accuracy_deg == 0.0);
    CHECK(c.filter.snap_deg == 90.0);
}

TEST(pibridge_default_comes_first) {
    eyemath_test::TempDir dir("pibridge-cal-pl");
    const std::optional<std::filesystem::path> home = dir.path;
    auto old = Calibration::identity(FrameConv{}, 0.064);
    old.created_unix = 1;
    auto now = old;
    now.created_unix = 2;
    eyemath_test::write_file(dir.path / ".config/omarchyvr/eye-calibration.toml", old.to_toml());
    auto sel = Calibration::select(std::nullopt, home).value();
    CHECK(sel && sel->cal.created_unix == 1);
    // the folder of PiBridge's earlier name comes before omarchyvr's
    auto mid = old;
    mid.created_unix = 3;
    eyemath_test::write_file(dir.path / ".config/pimax-linux/eye-calibration.toml", mid.to_toml());
    sel = Calibration::select(std::nullopt, home).value();
    CHECK(sel && sel->cal.created_unix == 3);
    eyemath_test::write_file(dir.path / ".config/pibridge/eye-calibration.toml", now.to_toml());
    sel = Calibration::select(std::nullopt, home).value();
    CHECK(sel && sel->cal.created_unix == 2 && sel->path == dir.path / ".config/pibridge/eye-calibration.toml");
    // from_env with HOME pointing at the temporary directory (never the user's real one)
    const char* old_home = std::getenv("HOME");
    const std::string saved = old_home ? old_home : "";
    ::setenv("HOME", dir.path.c_str(), 1);
    ::unsetenv("PIBRIDGE_EYE_CALIBRATION");
    const auto env = Calibration::from_env().value();
    CHECK(env && env->cal.created_unix == 2);
    CHECK(Calibration::default_path() == dir.path / ".config/pibridge/eye-calibration.toml");
    ::setenv("PIBRIDGE_EYE_CALIBRATION", "off", 1);
    CHECK(!Calibration::from_env().value().has_value());
    ::unsetenv("PIBRIDGE_EYE_CALIBRATION");
    // the older name is still accepted (omarchyvr, pimax-eyecal), the PIBRIDGE_ one wins
    ::setenv("PIMAX_EYE_CALIBRATION", "off", 1);
    CHECK(!Calibration::from_env().value().has_value());
    ::setenv("PIBRIDGE_EYE_CALIBRATION", (dir.path / ".config/pibridge/eye-calibration.toml").c_str(), 1);
    CHECK(Calibration::from_env().value().has_value());
    ::unsetenv("PIBRIDGE_EYE_CALIBRATION");
    ::unsetenv("PIMAX_EYE_CALIBRATION");
    if (old_home) {
        ::setenv("HOME", saved.c_str(), 1);
    } else {
        ::unsetenv("HOME");
    }
}

PIBRIDGE_TEST_MAIN()
