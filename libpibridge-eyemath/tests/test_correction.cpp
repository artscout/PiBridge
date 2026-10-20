// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/correction.rs tests (MIT OR Apache-2.0).
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "pibridge/eyemath/correction.h"
#include "pibridge_test.h"
#include "test_util.h"

using namespace pibridge::eyemath;

namespace {

Correction shifted() {
    Correction c;
    c.model = "affine";
    c.cx = {0.02, 1.05, 0.01, 0.0, 0.0, 0.0};
    c.cy = {-0.03, -0.02, 0.95, 0.0, 0.0, 0.0};
    c.flips = std::array<bool, 3>{false, true, true};
    return c;
}

bool contains(const std::string& s, std::string_view what) { return s.find(what) != std::string::npos; }

std::string err_of(std::string_view text) {
    const auto r = Correction::parse(text);
    return r ? std::string() : r.error().message;
}

} // namespace

TEST(identity_changes_nothing_and_is_recognised) {
    const auto c = Correction::identity();
    CHECK(c.is_identity() && !shifted().is_identity());
    for (const auto& d : {Vec3{0.0, 0.0, -1.0}, dir_of(0.4, -0.3), dir_of(-1.2, 0.9)}) {
        const auto o = c.apply(d);
        for (std::size_t k = 0; k < 3; k++) {
            CHECK_NEAR(o[k], d[k], 1e-12);
        }
    }
}

TEST(affine_moves_tangents_as_written) {
    const auto c = shifted();
    const auto t = c.map_tangent(0.5, -0.25);
    CHECK_NEAR(t.u, 0.02 + 1.05 * 0.5 + 0.01 * -0.25, 1e-12);
    CHECK_NEAR(t.v, -0.03 - 0.02 * 0.5 + 0.95 * -0.25, 1e-12);
    const auto o = c.apply(dir_of(0.5, -0.25));
    const auto ot = *tangent_of(o);
    CHECK_NEAR(ot.u, t.u, 1e-12);
    CHECK_NEAR(ot.v, t.v, 1e-12);
    CHECK_NEAR(std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2]), 1.0, 1e-12);
}

TEST(backward_and_far_out_directions_are_left_alone) {
    const auto c = shifted();
    const Vec3 back{0.0, 0.0, 1.0};
    CHECK(c.apply(back) == back);
    const Vec3 sideways{1.0, 0.0, 0.0};
    CHECK(c.apply(sideways) == sideways);
    const auto far = dir_of(2.0, 0.0);
    CHECK(c.apply(far) == far);
    CHECK(c.apply({std::numeric_limits<double>::quiet_NaN(), 0.0, -1.0}).size() == 3);
}

TEST(toml_round_trip_and_padding) {
    const auto c = shifted();
    const std::vector<std::string> notes{"mean error 2.1 -> 0.7 deg"};
    const auto t = c.to_toml(notes);
    CHECK(t.starts_with("# pimax eye gaze correction"));
    CHECK(contains(t, "# mean error 2.1 -> 0.7 deg"));
    const auto back = Correction::parse(t);
    CHECK(back.has_value());
    for (std::size_t k = 0; k < 6; k++) {
        CHECK_NEAR(back->cx[k], c.cx[k], 1e-9);
        CHECK_NEAR(back->cy[k], c.cy[k], 1e-9);
    }
    CHECK((back->flips == std::array<bool, 3>{false, true, true}));
    CHECK(back->model == "affine");
    const auto short_ = Correction::parse("x = [0, 1, 0]\ny = [0, 0, 1]  # identity\n");
    CHECK(short_ && short_->is_identity());
}

TEST(parse_errors_are_specific) {
    CHECK(contains(err_of("x = [0, 1, 0]"), "missing y"));
    CHECK(contains(err_of("x = [1, 2]\ny = [0, 0, 1]"), "3 or 6"));
    CHECK(contains(err_of("x = 1\ny = [0,0,1]"), "[a, b"));
    CHECK(contains(err_of("garbage"), "key = value"));
    CHECK(!Correction::parse("x = [a, b, c]\ny = [0,0,1]").has_value());
}

TEST(select_follows_the_variable_and_the_default_file) {
    eyemath_test::TempDir dir("pibridge-corr");
    const auto cfgdir = dir.path / ".config/omarchyvr";
    std::filesystem::create_directories(cfgdir);
    const std::optional<std::filesystem::path> home = dir.path;
    // no file, no variable
    CHECK(Correction::select(std::nullopt, home).value() == std::nullopt);
    eyemath_test::write_file(cfgdir / "eye-correction.toml", shifted().to_toml());
    const auto d = Correction::select(std::nullopt, home).value();
    CHECK(d.has_value());
    CHECK_NEAR(d->cx[1], 1.05, 1e-9);
    // "none" disables even when the file exists; an empty value counts as unset
    CHECK(Correction::select("none", home).value() == std::nullopt);
    CHECK(Correction::select("off", home).value() == std::nullopt);
    CHECK(Correction::select("", home).value().has_value());
    // an explicit path wins, and a missing one is an error (not silently no correction)
    eyemath_test::write_file(dir.path / "other.toml", Correction::identity().to_toml());
    const auto o = Correction::select((dir.path / "other.toml").string(), home).value();
    CHECK(o && o->is_identity());
    CHECK(!Correction::select("/nonexistent/x.toml", home).has_value());
}

// C++ only: the pibridge default file is preferred over the pimax-linux one, and that over the omarchyvr one.
TEST(pibridge_default_comes_first) {
    eyemath_test::TempDir dir("pibridge-corr-pl");
    const std::optional<std::filesystem::path> home = dir.path;
    eyemath_test::write_file(dir.path / ".config/omarchyvr/eye-correction.toml", shifted().to_toml());
    CHECK(Correction::find_default(home) == dir.path / ".config/omarchyvr/eye-correction.toml");
    eyemath_test::write_file(dir.path / ".config/pimax-linux/eye-correction.toml", shifted().to_toml());
    CHECK(Correction::find_default(home) == dir.path / ".config/pimax-linux/eye-correction.toml");
    eyemath_test::write_file(dir.path / ".config/pibridge/eye-correction.toml", Correction::identity().to_toml());
    CHECK(Correction::find_default(home) == dir.path / ".config/pibridge/eye-correction.toml");
    CHECK(Correction::select(std::nullopt, home).value()->is_identity());
}

TEST(samples_are_corrected_in_gaze_and_valid_eyes_only) {
    const auto c = shifted();
    const auto d = dir_of(0.3, 0.1);
    const Vec3f df{static_cast<float>(d[0]), static_cast<float>(d[1]), static_cast<float>(d[2])};
    Sample s;
    s.gaze = Gaze{{0.0F, 0.0F, 0.01F}, df, 3};
    s.eyes[0].origin = {-0.03F, 0.0F, 0.0F};
    s.eyes[0].direction = df;
    s.eyes[0].origin_valid = true;
    s.eyes[0].direction_valid = true;
    s.eyes[0].pupil_mm = 4.0F;
    s.eyes[1].direction = df;
    c.apply_sample(s);
    const auto want = c.apply_f32(df);
    CHECK(s.gaze->direction == want);
    CHECK((s.gaze->origin == Vec3f{0.0F, 0.0F, 0.01F}));
    CHECK(s.eyes[0].direction == want);
    CHECK(s.eyes[1].direction == df); // an invalid eye keeps what it had
    CHECK((s.eyes[0].origin == Vec3f{-0.03F, 0.0F, 0.0F}));
}

TEST(flip_warning_only_when_they_differ) {
    const auto c = shifted();
    FrameConv conv;
    conv.flip_y = true; // measured with 0,1,1
    CHECK(!c.flip_warning(conv).has_value());
    conv.flip_y = false;
    const auto w = c.flip_warning(conv);
    CHECK(w && contains(*w, "0/1/1"));
    CHECK(!Correction::identity().flip_warning(conv).has_value());
}

PIBRIDGE_TEST_MAIN()
