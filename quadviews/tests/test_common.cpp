// SPDX-License-Identifier: MIT
#include <clocale>
#include "check.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>

#include "fovmath.h"
#include "gaze.h"
#include "settings.h"

using namespace qv;

static float deg(float d) {
    return d * float(M_PI) / 180.f;
}

TEST(settings_round_trip) {
    Settings s;
    CHECK(parseSettings(s.toLines(), {}) == s);
    s.peripheralDensity = 0.3f;
    s.tracking = Tracking::Fixed;
    s.offsetDeg = {1.5f, -2.f};
    s.peripheralFilter = Filter::Bilinear;
    s.gazeFilter = false;
    s.debugOverlay = true;
    s.gazeHoldMs = 250;
    s.edgeOpacity = 0.35f;
    CHECK(parseSettings(s.toLines(), {}) == s);
}

TEST(settings_bad_values_keep_defaults_and_clamp) {
    const Settings s = parseSettings(
        "peripheral_density=abc\nfoo=1\nsharpen=7\ngaze_width=0.01\ntracking=nose\ndebug_overlay=maybe\n"
        "gaze_hold_ms=-5\n",
        {});
    const Settings d;
    CHECK(s.peripheralDensity == d.peripheralDensity);
    CHECK(s.sharpen == 1.f);
    CHECK(s.gazeSize[0] == 0.1f);
    CHECK(s.tracking == d.tracking);
    CHECK(s.debugOverlay == d.debugOverlay);
    CHECK(s.gazeHoldMs == d.gazeHoldMs);
}

TEST(settings_numbers_ignore_the_locale) {
    // the settings app under ru_RU wrote decimal commas; the file must read the same as with points
    const Settings s = parseSettings("peripheral_density=0,13\nsharpen=0,6\noffset_y_deg=-2,5\n", {});
    CHECK_NEAR(s.peripheralDensity, 0.13, 1e-6);
    CHECK_NEAR(s.sharpen, 0.6, 1e-6);
    CHECK_NEAR(s.offsetDeg[1], -2.5, 1e-6);
    CHECK_NEAR(parseSettings("peripheral_density=0.05\n", {}).peripheralDensity, 0.05, 1e-6);
    CHECK_NEAR(parseSettings("peripheral_density=0.01\n", {}).peripheralDensity, 0.05, 1e-6);
    // and what is written has points whatever the process locale is
    std::setlocale(LC_NUMERIC, "ru_RU.UTF-8");
    Settings w;
    w.peripheralDensity = 0.13f;
    const std::string t = w.toLines();
    std::setlocale(LC_NUMERIC, "C");
    CHECK(t.find("peripheral_density=0.13") != std::string::npos);
    CHECK(parseSettings(t, {}) == w);
}

TEST(settings_sections_apply_to_their_app_only) {
    const std::string t = "gaze_width=0.4\n[app:Falcon BMS]\ngaze_width=0.3\n[exe:dcs]\nsharpen=0\n# x\n";
    const Settings bms = parseSettings(t, {"Falcon BMS 4.38", "Falcon_BMS"});
    CHECK_NEAR(bms.gazeSize[0], 0.3, 1e-6);
    CHECK_NEAR(bms.sharpen, Settings{}.sharpen, 1e-6);
    const Settings other = parseSettings(t, {"Other", "dcs_mt"});
    CHECK_NEAR(other.gazeSize[0], 0.4, 1e-6);
    CHECK_NEAR(other.sharpen, 0.0, 1e-6);
}

TEST(settings_save_keeps_sections) {
    const auto dir = std::filesystem::temp_directory_path() / "qv-vulkan-test";
    std::filesystem::create_directories(dir);
    const auto p = dir / "settings.conf";
    {
        std::ofstream o(p);
        o << "sharpen=0.1\n[app:Falcon BMS]\ngaze_width=0.3\n";
    }
    Settings s;
    s.sharpen = 0.9f;
    CHECK(saveSettings(p, s));
    CHECK_NEAR(loadSettings(p, {}).sharpen, 0.9, 1e-6);
    CHECK_NEAR(loadSettings(p, {"Falcon BMS", ""}).gazeSize[0], 0.3, 1e-6);
    std::filesystem::remove_all(dir);
}

TEST(settings_profiles) {
    const std::string t = "sharpen=0.2\n[app:Falcon BMS]\ngaze_width=0.3\n[exe:dcs]\nsharpen=0\n";
    const auto names = sectionNames(t);
    CHECK(names.size() == 2 && names[0] == "app:Falcon BMS" && names[1] == "exe:dcs");
    const Settings bms = parseSection(t, "app:Falcon BMS");
    CHECK_NEAR(bms.sharpen, 0.2, 1e-6); // global line applies
    CHECK_NEAR(bms.gazeSize[0], 0.3, 1e-6);
    Settings s = bms;
    s.offsetDeg[1] = -3.f;
    const std::string t2 = replaceSection(t, "app:Falcon BMS", s);
    CHECK(parseSection(t2, "app:Falcon BMS") == s);
    CHECK_NEAR(parseSection(t2, "exe:dcs").sharpen, 0.0, 1e-6); // the other profile untouched
    CHECK_NEAR(parseSettings(t2, {}).sharpen, 0.2, 1e-6);       // global untouched
    const std::string t3 = replaceSection(t2, "app:New", Settings{});
    CHECK(sectionNames(t3).size() == 3);
    const std::string t4 = removeSection(t3, "app:Falcon BMS");
    CHECK(sectionNames(t4).size() == 2);
    CHECK(parseSettings(t4, {"Falcon BMS", ""}).gazeSize[0] == Settings{}.gazeSize[0]);
    // the layer sees the profile written by the app
    CHECK(parseSettings(t2, {"Falcon BMS 4.38", "x"}) == s);
}

TEST(fov_round_trip) {
    const XrFovf f{deg(-50), deg(45), deg(48), deg(-52)};
    const XrFovf g = toFov(toTan(f));
    CHECK_NEAR(g.angleLeft, f.angleLeft, 1e-6);
    CHECK_NEAR(g.angleRight, f.angleRight, 1e-6);
    CHECK_NEAR(g.angleUp, f.angleUp, 1e-6);
    CHECK_NEAR(g.angleDown, f.angleDown, 1e-6);
}

TEST(focus_window_keeps_its_size_at_the_edge) {
    const Tan base{-1.f, 1.f, -1.f, 1.f};
    const Tan c = focusWindow(base, 0.f, 0.f, 0.4f, 0.3f);
    CHECK_NEAR(c.left, -0.4, 1e-6);
    CHECK_NEAR(c.right, 0.4, 1e-6);
    CHECK_NEAR(c.down, -0.3, 1e-6);
    CHECK_NEAR(c.up, 0.3, 1e-6);
    // looking far right and down: shifted back inside, same size
    const Tan e = focusWindow(base, 0.95f, -2.f, 0.4f, 0.3f);
    CHECK_NEAR(e.right, 1.0, 1e-6);
    CHECK_NEAR(e.right - e.left, 0.8, 1e-6);
    CHECK_NEAR(e.down, -1.0, 1e-6);
    CHECK_NEAR(e.up - e.down, 0.6, 1e-6);
    // the whole field
    const Tan w = focusWindow(base, 0.5f, 0.f, 1.f, 1.f);
    CHECK_NEAR(w.left, -1.0, 1e-6);
    CHECK_NEAR(w.right, 1.0, 1e-6);
}

TEST(uv_rect_rows_point_down) {
    const Tan o{-1.f, 1.f, -1.f, 1.f};
    const auto r = uvRect(o, {0.f, 0.5f, 0.5f, 1.f}); // right half of the top half... a window up-right
    CHECK_NEAR(r[0], 0.5, 1e-6);
    CHECK_NEAR(r[2], 0.75, 1e-6);
    CHECK_NEAR(r[1], 0.0, 1e-6);  // up = 1 is the top row
    CHECK_NEAR(r[3], 0.25, 1e-6); // down = 0.5
}

TEST(eye_tangent_with_offset_and_canted_eye) {
    // An eye 32 mm left, looking straight: a point 2 m ahead on the centre line is right of its axis.
    XrPosef eye{{0, 0, 0, 1}, {-0.032f, 0, 0}};
    float tx = 0, ty = 0;
    CHECK(eyeTangent(eye, {0, 0, -2}, tx, ty));
    CHECK_NEAR(tx, 0.016, 1e-6);
    CHECK_NEAR(ty, 0.0, 1e-6);
    // The same eye canted 10 degrees outward (yaw left): the point is 10 degrees further right.
    const float h = deg(10) / 2;
    eye.orientation = {0, std::sin(h), 0, std::cos(h)};
    CHECK(eyeTangent(eye, {-0.032f, 0, -2}, tx, ty));
    CHECK_NEAR(std::atan(tx), deg(10), 1e-5);
    // Behind the eye
    CHECK(!eyeTangent({{0, 0, 0, 1}, {0, 0, 0}}, {0, 0, 1}, tx, ty));
}

TEST(add_angle) {
    CHECK_NEAR(addAngle(0.f, 45.f), 1.0, 1e-5);
    CHECK_NEAR(std::atan(addAngle(std::tan(deg(10)), -4.f)), deg(6), 1e-5);
}

TEST(sizes_and_budget) {
    const Size rec{4000, 4000};
    CHECK(scaledSize(rec, {}, 0.4f) == (Size{1600, 1600}));
    CHECK(scaledSize(rec, {}, 1.f, 0.35f, 0.35f) == (Size{1400, 1400}));
    CHECK(scaledSize(rec, {3000, 3000}, 1.5f) == (Size{3000, 3000}));
    CHECK(scaledSize({3, 3}, {}, 0.1f) == (Size{16, 16}));
    CHECK(scaledSize({1001, 999}, {}, 1.f).w % 2 == 0);
    Settings s; // 0.4 peripheral, 1.0 focus at 0.35 x 0.35
    const Budget b = budget(rec, s, true);
    CHECK(b.stereoPixels == 2ull * 4000 * 4000);
    CHECK(b.quadPixels == 2ull * (1600 * 1600 + 1400 * 1400));
    CHECK_NEAR(b.saving, 1.0 - (0.16 + 0.1225), 1e-9);
}

TEST(rotate_inverse_round_trip) {
    const float h = deg(33) / 2;
    const XrQuaternionf q{0.2f * std::sin(h), 0.9f * std::sin(h), 0.4f * std::sin(h), std::cos(h)};
    const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const XrQuaternionf u{q.x / n, q.y / n, q.z / n, q.w / n};
    const XrVector3f v{0.3f, -1.2f, 2.f};
    const XrVector3f w = rotateInv(u, rotate(u, v));
    CHECK_NEAR(w.x, v.x, 1e-5);
    CHECK_NEAR(w.y, v.y, 1e-5);
    CHECK_NEAR(w.z, v.z, 1e-5);
}

TEST(one_euro_reduces_fixation_noise) {
    OneEuro f;
    OneEuroParams p;
    std::mt19937 rng(7);
    std::normal_distribution<double> n(0.0, std::tan(1.0 * M_PI / 180));
    double raw = 0, out = 0;
    int k = 0;
    for (int i = 0; i < 1200; i++) {
        double x = 0.1 + n(rng), y = -0.05 + n(rng);
        const double rx = x, ry = y;
        f.filter(p, i / 120.0, x, y);
        if (i > 240) {
            raw += std::hypot(rx - 0.1, ry + 0.05);
            out += std::hypot(x - 0.1, y + 0.05);
            k++;
        }
    }
    CHECK(out < raw / 4);
}

TEST(one_euro_snaps_on_saccades) {
    OneEuro f;
    OneEuroParams p;
    for (int i = 0; i < 120; i++) {
        double x = 0, y = 0;
        f.filter(p, i / 120.0, x, y);
    }
    const double b = std::tan(10 * M_PI / 180);
    double x1 = b, y1 = 0, x2 = b, y2 = 0;
    f.filter(p, 1.0, x1, y1);
    f.filter(p, 1.0 + 1 / 120.0, x2, y2);
    CHECK(x1 < b);
    CHECK_NEAR(x2, b, 1e-12);
}

TEST(gaze_hold_bridges_blinks_then_drops) {
    GazeTrack g;
    CHECK(g.update(0.0, GazeTrack::Dir{0.2, 0.1}, false, 600).has_value());
    const auto held = g.update(0.3, std::nullopt, false, 600);
    CHECK(held.has_value() && std::fabs(held->x - 0.2) < 1e-12);
    CHECK(!g.update(0.7, std::nullopt, false, 600).has_value());
    CHECK(!g.update(0.71, std::nullopt, false, 600).has_value());
}

QV_TEST_MAIN
