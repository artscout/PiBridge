// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/config.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/lens/config.h"
#include "pibridge_test.h"

using namespace pibridge::lens;
using pibridge::lens::testing::ASPECT;
using pibridge::lens::testing::env_of;

namespace {

using F = std::vector<float>;

bool scan_is(std::string_view s, float v, std::size_t n) {
    const auto r = detail::scan_float(s);
    return r && r->first == v && r->second == n;
}

} // namespace

TEST(defaults_match_the_old_c_defaults) {
    const auto c = LensConfig::defaults(ASPECT);
    const auto& l = c.eye[0];
    CHECK((l.fov == std::array<float, 4>{50.0f, 50.0f, 44.5f, 44.5f}));
    CHECK(l.cx == 0.0271f && l.cy == 0.009f);
    CHECK(c.eye[1].cx == -0.0271f);
    CHECK((l.base == std::array<float, 3>{0.65307f, 0.65307f, 0.65307f}));
    CHECK(l.scale == 1.0f);
    for (std::size_t ch = 0; ch < 3; ch++) {
        CHECK((l.k[ch] == Poly{0.60779f, -0.72489f, 0.58858f, 0.0f, 0.0f}));
    }
    CHECK((l.ca == std::array<float, 3>{1.0045f, 1.0f, 0.995f}));
    CHECK(l.fit_range == 0.0f);
    CHECK(!l.has_mask());
    CHECK(c.enabled);
}

TEST(the_old_example_file_parses_to_the_defaults) {
    const char* text = "[common]\nfov = 50 50 44.5 44.5\nbase = 0.65307\nscale = 1.0\n"
                       "k_g = 0.60779 -0.72489 0.58858\nk_r = 0.60779 -0.72489 0.58858\nk_b = 0.60779 -0.72489 0.58858\n"
                       "ca_r = 1.0045\nca_g = 1.0\nca_b = 0.995\n[left]\ncenter = 0.0271 0.009\n[right]\ncenter = -0.0271 "
                       "0.009\n";
    const auto [c, w] = parse(text, ASPECT);
    CHECK(w.empty());
    CHECK(c == LensConfig::defaults(ASPECT));
}

TEST(sections_comments_and_override_order) {
    const char* text = "fov = 40 41 42 43 # trailing comment\n; whole line\n[left]\nfov = 1 2 3 4\n[both]\nscale = "
                       "1.5\n[right]\nscale = 2\n";
    const auto [c, w] = parse(text, ASPECT);
    CHECK(w.empty());
    CHECK((c.eye[0].fov == std::array<float, 4>{1.0f, 2.0f, 3.0f, 4.0f}));
    CHECK((c.eye[1].fov == std::array<float, 4>{40.0f, 41.0f, 42.0f, 43.0f}));
    CHECK(c.eye[0].scale == 1.5f);
    CHECK(c.eye[1].scale == 2.0f);
}

TEST(k_accepts_three_to_five_numbers_and_zeroes_the_rest) {
    const auto [c, w] = parse("k = 1 2 3\nk_r = 1 2 3 4 5\nk_b = 9,8,7,6\n", ASPECT);
    CHECK(w.empty());
    CHECK((c.eye[0].k[1] == Poly{1, 2, 3, 0, 0}));
    CHECK((c.eye[0].k[0] == Poly{1, 2, 3, 4, 5}));
    CHECK((c.eye[0].k[2] == Poly{9, 8, 7, 6, 0}));
    // fewer than three numbers is an error and changes nothing
    const auto [c2, w2] = parse("k = 1 2\n", ASPECT);
    CHECK(w2.size() == 1);
    CHECK(c2.eye[0].k[0] == EyeConfig::defaults(0).k[0]);
}

TEST(bad_lines_are_reported_with_the_old_wording) {
    const auto [c, w] = parse("[nonsense]\njust words\nfov = 1 2\nunknown = 3\n", ASPECT);
    (void)c;
    std::vector<std::string> msgs;
    for (const auto& x : w) {
        msgs.push_back(x.message);
    }
    CHECK((msgs == std::vector<std::string>{
                       "lens.conf:1 unknown section [nonsense]",
                       "lens.conf:2 expected key = value",
                       "lens.conf:3 bad key/value 'fov = 1 2'",
                       "lens.conf:4 bad key/value 'unknown = 3'",
                   }));
}

TEST(cant_deg_key_and_env) {
    {
        const auto [c, w] = parse("cant_deg = 2.5\n[left]\ncant_deg = 3\n", ASPECT);
        CHECK(w.empty());
        CHECK(c.cant_deg == 3.0f);
    }
    CHECK(parse("", ASPECT).first.cant_deg == 0.0f);
    {
        const auto [c, w] = parse("cant_deg = 2\ncant_deg = 99\ncant_deg = x\n", ASPECT);
        CHECK(c.cant_deg == 2.0f && w.size() == 2);
    }
    const auto with = [](const char* v) { return [v](std::string_view) -> std::optional<std::string> { return v; }; };
    CHECK(!cant_override(pibridge::lens::testing::no_env()));
    const auto ok = cant_override(with("2.95"));
    CHECK(ok && ok->ok && ok->value == 2.95f);
    const auto bad = cant_override(with("abc"));
    CHECK(bad && !bad->ok);
    const auto big = cant_override(with("50"));
    CHECK(big && !big->ok);
}

TEST(distortion_switch) {
    for (const char* off : {"off", "OFF", "0", "false", "no"}) {
        CHECK(!parse(std::string("distortion = ") + off + "\n", ASPECT).first.enabled);
    }
    for (const char* on : {"on", "1", "true", "yes", "anything"}) {
        CHECK(parse(std::string("distortion = ") + on + "\n", ASPECT).first.enabled);
    }
}

TEST(number_scanner_follows_strtof_for_decimals) {
    CHECK(scan_is("  -1.5e2 rest", -150.0f, 8));
    CHECK(scan_is(".5x", 0.5f, 2));
    CHECK(scan_is("5.", 5.0f, 2));
    CHECK(scan_is("1e", 1.0f, 1));
    CHECK(scan_is("1e+", 1.0f, 1));
    CHECK(scan_is("+3", 3.0f, 2));
    CHECK(!detail::scan_float("abc"));
    CHECK(!detail::scan_float("-"));
    CHECK(!detail::scan_float("nan"));
    CHECK((detail::floats("1,2  3 ,4", 4, 4) == std::optional<F>(F{1, 2, 3, 4})));
    CHECK(!detail::floats("1 2 x", 3, 3));
    CHECK((detail::floats("1 2 3 4 5 6", 1, 3) == std::optional<F>(F{1, 2, 3})));
}

TEST(new_per_channel_keys) {
    const auto [c, w] = parse(
        "base_r = 0.7\nbase_b = 0.6\nca_r = 1.01\nshift_b = 0.01 -0.02\nk_g = 0.1 0.2 0.3 0.4 0.5\n", ASPECT);
    CHECK(w.empty());
    const auto& e = c.eye[0];
    CHECK((e.base == std::array<float, 3>{0.7f, 0.65307f, 0.6f}));
    CHECK(e.ca[0] == 1.01f);
    CHECK((e.shift[2] == std::array<float, 2>{0.01f, -0.02f}));
    CHECK((e.k[1] == Poly{0.1f, 0.2f, 0.3f, 0.4f, 0.5f}));
    // `base` after `base_r` resets all channels, later lines win
    const auto [c2, w2] = parse("base_r = 0.7\nbase = 0.5\n", ASPECT);
    (void)w2;
    CHECK((c2.eye[0].base == std::array<float, 3>{0.5f, 0.5f, 0.5f}));
}

TEST(mask_and_blackout_and_fit_range) {
    const auto [c, w] = parse("fit_range = 1.5\nblackout = 0.02\nmask = ellipse 1.1 0.95\nmask_feather = "
                              "0.05\n[right]\nblackout = 0.01 0.02 0.03 0.04\nmask = rect 1 0.9\n",
                              ASPECT);
    CHECK(w.empty());
    const auto& l = c.eye[0];
    CHECK(l.fit_range == 1.5f);
    CHECK((l.blackout == std::array<float, 4>{0.02f, 0.02f, 0.02f, 0.02f}));
    CHECK((l.mask == Mask{MaskShape::Ellipse, 1.1f, 0.95f, 2.0f}));
    CHECK(l.mask_feather == 0.05f);
    CHECK(l.has_mask());
    const auto& r = c.eye[1];
    CHECK((r.blackout == std::array<float, 4>{0.01f, 0.02f, 0.03f, 0.04f}));
    CHECK(r.mask.shape == MaskShape::Rect);
    const auto [c3, w3] = parse("mask = superellipse 1.1 1.0 4\n", ASPECT);
    CHECK(w3.empty());
    CHECK((c3.eye[0].mask == Mask{MaskShape::Superellipse, 1.1f, 1.0f, 4.0f}));
    const auto c4 = parse("mask = ellipse 1\n", ASPECT).first;
    CHECK(c4.eye[0].mask.a == 1.0f && c4.eye[0].mask.b == 1.0f);
    const auto c5 = parse("mask = ellipse 1\nmask = off\n", ASPECT).first;
    CHECK(c5.eye[0].mask == Mask::off());
    // invalid values are rejected and leave the setting alone
    for (const char* bad : {"mask = ellipse -1 1", "mask = blob 1 1", "blackout = -0.1", "blackout = 1 2",
                            "fit_range = -1", "mask_feather = -1"}) {
        CHECK(parse(bad, ASPECT).second.size() == 1);
    }
}

TEST(lut_keys) {
    const auto [c, w] =
        parse("lut = /a/b.bin\n[left]\nlut_margin = 0.3\nlut_interp = Bilinear\n[right]\nlut = none\n", 1.0f);
    CHECK(w.empty());
    CHECK(c.eye[0].lut == std::optional<std::string>("/a/b.bin"));
    CHECK(!c.eye[1].lut);
    CHECK(c.eye[0].lut_margin == 0.3f);
    CHECK(c.eye[1].lut_margin == 0.0f);
    CHECK(c.eye[0].lut_interp == Interp::Bilinear);
    CHECK(c.eye[1].lut_interp == Interp::Bicubic);
    CHECK(parse("lut =\nlut_margin = -1\nlut_interp = spline\n", 1.0f).second.size() == 3);
}

TEST(implausible_values_are_rejected_and_leave_the_setting_alone) {
    for (const char* bad : {"fov = 0 50 44.5 44.5", "fov = 90 50 44.5 44.5", "fov = 50 50 44.5 -3", "center = 5 0",
                            "base = 0", "base = -1", "base_r = 100", "scale = 0", "scale = 100", "ca_r = 0",
                            "ca_g = -1", "ca_b = 3"}) {
        const auto [c, w] = parse(bad, ASPECT);
        CHECK(w.size() == 1);
        CHECK(c == LensConfig::defaults(ASPECT));
    }
    // the values of the old example and ordinary tunings still pass
    const auto [c, w] =
        parse("fov = 60 60 55 55\ncenter = -0.1 0.2\nbase = 0.7\nscale = 1.2\nca_r = 1.02\nca_b = 0.98\n", ASPECT);
    CHECK(w.empty());
    CHECK((c.eye[0].fov == std::array<float, 4>{60.0f, 60.0f, 55.0f, 55.0f}));
}

TEST(path_resolution_and_off_switch) {
    CHECK(config_path(env_of({{"PIMAX_LENS_CONF", "/x/y.conf"}, {"HOME", "/h"}})) == "/x/y.conf");
    CHECK(config_path(env_of({{"XDG_CONFIG_HOME", "/xdg"}, {"HOME", "/h"}})) == "/xdg/omarchyvr/lens.conf");
    CHECK(config_path(env_of({{"HOME", "/h"}})) == "/h/.config/omarchyvr/lens.conf");
    CHECK(config_path(env_of({})) == "./.config/omarchyvr/lens.conf");
    CHECK(config_path(env_of({{"PIMAX_LENS_CONF", ""}, {"HOME", "/h"}})) == "/h/.config/omarchyvr/lens.conf");
    CHECK(forced_off(env_of({{"PIMAX_LENS_OFF", "1"}})));
    CHECK(forced_off(env_of({{"PIMAX_LENS_OFF", "yes"}})));
    CHECK(!forced_off(env_of({{"PIMAX_LENS_OFF", "0"}})));
    CHECK(!forced_off(env_of({{"PIMAX_LENS_OFF", ""}})));
    CHECK(!forced_off(env_of({})));
}

PIBRIDGE_TEST_MAIN()
