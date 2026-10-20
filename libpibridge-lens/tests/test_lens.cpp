// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/lib.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/lens/lens.h"
#include "pibridge/lens/mask.h"
#include "pibridge/lens/png.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::lens;
using pibridge::lens::testing::ASPECT;
using pibridge::lens::testing::env_of;
using pibridge::lens::testing::no_env;
using pibridge::lens::testing::tmpdir;
using pibridge::lens::testing::write;

namespace {

bool contains(const std::string& s, std::string_view x) {
    return s.find(x) != std::string::npos;
}

std::vector<std::uint8_t> read_bytes(const std::string& p) {
    const auto t = pibridge::cam::read_file(p);
    if (!t) {
        return {};
    }
    return {t->begin(), t->end()};
}

} // namespace

TEST(missing_file_uses_defaults_and_logs_like_the_c_driver) {
    const auto loaded = load_file(ASPECT, no_env(), "/nonexistent/lens.conf");
    CHECK(!loaded.file_found);
    CHECK(loaded.lens.config() == LensConfig::defaults(ASPECT));
    const auto& t = loaded.messages;
    CHECK(t[0].text == "pimax lens: no config at /nonexistent/lens.conf, using built-in defaults");
    CHECK(t[1].text == "pimax lens: eye 0 distortion on fov L50.0 R50.0 U44.5 D44.5 centre 0.0271,0.0090 base 0.6531 "
                       "scale 1.000");
    CHECK(t[2].text == "pimax lens: eye 0 k_r 0.6078 -0.7249 0.5886  k_g 0.6078 -0.7249 0.5886  k_b 0.6078 -0.7249 "
                       "0.5886  ca 1.0045 1.0000 0.9950");
    CHECK(t.size() == 6);
    CHECK(t[5].text == "pimax lens: cant 0.000 deg per eye (left view yawed +0.000, right view -0.000)");
}

TEST(file_warnings_and_forced_off) {
    const std::string dir = tmpdir("lens-test");
    const std::string p = dir + "/lens.conf";
    write(p, "fov = 40 40 40 40\nbogus line\n");
    const auto loaded = load_file(ASPECT, env_of({{"PIMAX_LENS_OFF", "1"}}), p);
    CHECK(loaded.file_found);
    CHECK(!loaded.lens.config().enabled);
    CHECK((loaded.lens.fov_deg(0) == std::array<float, 4>{40.0f, 40.0f, 40.0f, 40.0f}));
    CHECK(loaded.messages[0].text == "pimax lens: reading " + p);
    CHECK(loaded.messages[1].level == Level::Warn);
    CHECK(loaded.messages[1].text.ends_with("lens.conf:2 expected key = value"));
    CHECK(contains(loaded.messages[2].text, "distortion OFF"));
    pibridge::lens::testing::rmrf(dir);
}

TEST(extended_features_are_logged_only_when_used) {
    const auto cfg = parse("k_r = 1 2 3 4 5\nbase_b = 0.6\nshift_g = 0.01 0\nfit_range = 1.4\nmask = ellipse 1 "
                           "1\nblackout = 0.01\n",
                           ASPECT)
                         .first;
    const auto lines = eye_log_lines(cfg, 0);
    const auto any = [&](auto pred) {
        for (const auto& l : lines) {
            if (pred(l)) {
                return true;
            }
        }
        return false;
    };
    CHECK(any([](const std::string& l) { return contains(l, "k4 k5"); }));
    CHECK(any([](const std::string& l) { return contains(l, "base r 0.65307 g 0.65307 b 0.60000"); }));
    CHECK(any([](const std::string& l) { return contains(l, "shift r"); }));
    CHECK(any([](const std::string& l) { return contains(l, "fit_range 1.400"); }));
    CHECK(any([](const std::string& l) {
        return contains(l, "visibility mask: blackout L0.01") && contains(l, "ellipse 1 1");
    }));
    CHECK(eye_log_lines(LensConfig::defaults(ASPECT), 1).size() == 2);
}

TEST(lut_key_replaces_the_radial_model_and_reproduces_the_dump_at_the_nodes) {
    const std::string dir = tmpdir("lens-lut");
    if (!pibridge::lens::testing::sub_lut_file(dir)) {
        return;
    }
    const std::string conf = dir + "/lut.conf";
    write(conf, "cant_deg = 2\nfov = 40 40 40 40\ncenter = 0.3 0.3\nlut = sub.bin\n");
    const auto loaded = load_file(ASPECT, no_env(), conf);
    for (const auto& m : loaded.messages) {
        CHECK(m.level == Level::Info);
    }
    bool logged = false;
    for (const auto& m : loaded.messages) {
        logged = logged || (contains(m.text, "eye 0 LUT") && contains(m.text, "grid 17x17"));
    }
    CHECK(logged);
    const Lens& lens = loaded.lens;
    CHECK(lens.lut(0) != nullptr && lens.lut(1) != nullptr);
    // the LUT's FOV port wins over `fov`, the centre is the panel middle
    const auto f = lens.fov_deg(0);
    CHECK(std::fabs(f[0] - 52.577f) < 0.01f && std::fabs(f[1] - 47.540f) < 0.01f && std::fabs(f[2] - 44.546f) < 0.01f);
    CHECK(lens.eye(0).cx == 0.0f && lens.eye(0).cy == 0.0f);
    CHECK(lens.config().cant_deg == 2.0f);
    // compute() = the dump's output UV (image UV y runs down: 1 - V), through each eye's panel orientation
    const auto dump = *pibridge::lens::testing::fixture_dump();
    const std::size_t n = dump.eyes[0].grid_n;
    for (std::size_t e = 0; e < 2; e++) {
        const EyeDump& ed = dump.eyes[e];
        const Dm& d = ORIENTS[lens.lut(e)->meta(e).orient].d;
        for (const auto& [j, i] : {std::pair<std::size_t, std::size_t>{4, 4}, {8, 8}, {10, 6}, {7, 12}, {9, 3}}) {
            const double pu = static_cast<double>(i) / static_cast<double>(n - 1);
            const double pv = static_cast<double>(j) / static_cast<double>(n - 1);
            const auto [u, v] = panel_to_display(d, pu, pv);
            const auto uv = lens.compute(e, static_cast<float>(u), static_cast<float>(v));
            for (std::size_t c = 0; c < 3; c++) {
                const std::size_t k = (j * n + i) * 2;
                const auto ou = static_cast<float>(ed.grid[c][k]);
                const auto ov = static_cast<float>(ed.grid[c][k + 1]);
                CHECK(std::fabs(uv[c][0] - ou) < 2e-5f);
                CHECK(std::fabs(uv[c][1] - (1.0f - ov)) < 2e-5f);
            }
        }
    }
    // visibility: the display centre is visible, a far corner is outside the image
    const Uv c = lens.compute(0, 0.5f, 0.5f)[1];
    CHECK(0.0f <= c[0] && c[0] <= 1.0f && 0.0f <= c[1] && c[1] <= 1.0f);
    const Uv corner = lens.compute(0, 0.0f, 0.0f)[1];
    CHECK(!(0.0f <= corner[0] && corner[0] <= 1.0f) || !(0.0f <= corner[1] && corner[1] <= 1.0f));
    // custom visibility mask traced from the visible area
    CHECK(lens.has_mask(0));
    const auto m = *visibility_mesh(lens, 0, MaskKind::LineLoop, 32);
    CHECK(m.vertices.size() == 32);
    pibridge::lens::testing::rmrf(dir);
}

TEST(lut_margin_widens_the_fov_and_per_eye_lut_none_keeps_one_eye_radial) {
    const std::string dir = tmpdir("lens-lutm");
    const auto p = pibridge::lens::testing::sub_lut_file(dir);
    if (!p) {
        return;
    }
    const std::string conf = dir + "/m.conf";
    write(conf, "lut = " + *p + "\nlut_margin = 0.5\nlut_interp = bilinear\n[right]\nlut = none\n");
    const auto loaded = load_file(ASPECT, no_env(), conf);
    const Lens& lens = loaded.lens;
    CHECK(lens.lut(0) != nullptr && lens.lut(1) == nullptr);
    const auto f = lens.fov_deg(0);
    CHECK(std::fabs(f[0] - 53.077f) < 0.01f && std::fabs(f[2] - 45.046f) < 0.01f);
    CHECK(lens.eye(0).lut_interp == Interp::Bilinear);
    CHECK((lens.fov_deg(1) == std::array<float, 4>{50.0f, 50.0f, 44.5f, 44.5f}));
    CHECK(!lens.has_mask(1));
    pibridge::lens::testing::rmrf(dir);
}

TEST(missing_or_invalid_lut_warns_and_falls_back_to_the_radial_model) {
    const std::string dir = tmpdir("lens-lutf");
    const std::string bad = dir + "/bad.bin";
    write(bad, "garbage");
    const Lens reference = load_file(ASPECT, no_env(), "/nonexistent/lens.conf").lens;
    for (const auto& [name, target] : {std::pair<std::string, std::string>{"missing", dir + "/nope.bin"},
                                       std::pair<std::string, std::string>{"invalid", bad}}) {
        const std::string conf = dir + "/" + name + ".conf";
        write(conf, "lut = " + target + "\n");
        const auto loaded = load_file(ASPECT, no_env(), conf);
        std::vector<const Message*> warns;
        for (const auto& m : loaded.messages) {
            if (m.level == Level::Warn) {
                warns.push_back(&m);
            }
        }
        CHECK(warns.size() == 2); // one per eye
        CHECK(contains(warns[0]->text, "falling back to the radial model"));
        CHECK(loaded.lens.lut(0) == nullptr);
        CHECK((loaded.lens.fov_deg(0) == std::array<float, 4>{50.0f, 50.0f, 44.5f, 44.5f}));
        CHECK(loaded.lens.compute(0, 0.3f, 0.6f) == reference.compute(0, 0.3f, 0.6f));
        // the radial model's configuration is still logged
        bool logged = false;
        for (const auto& m : loaded.messages) {
            logged = logged || contains(m.text, "eye 0 distortion on");
        }
        CHECK(logged);
    }
    pibridge::lens::testing::rmrf(dir);
}

TEST(debug_grid_writes_two_pngs_and_a_summary) {
    const std::string dir = tmpdir("lens-grid");
    const Lens lens(LensConfig::defaults(ASPECT));
    const auto msgs = write_debug_grid(lens, dir, 16);
    CHECK(msgs.size() == 3);
    for (const auto& m : msgs) {
        CHECK(m.level == Level::Info);
    }
    for (const char* name : {"pimax_lens_grid_left.png", "pimax_lens_grid_right.png"}) {
        const auto d = png::decode(read_bytes(dir + "/" + name));
        CHECK(d && d->width == 240 && d->height == 222 && d->channels == 3);
    }
    const auto s = pibridge::cam::read_file(dir + "/pimax_lens_summary.txt");
    CHECK(s && contains(*s, "eye 0 (left)") && contains(*s, "eye 1 (right)"));
    pibridge::lens::testing::rmrf(dir);
    // unwritable location -> a warning, no panic
    const auto bad = write_debug_grid(lens, "/proc/definitely/not/here", 16);
    CHECK(bad[0].level == Level::Warn);
}

TEST(without_a_lens_conf_the_shipped_dream_air_profile_is_used_and_the_users_own_wins) {
    const std::string home = tmpdir("lens-ref");
    const auto [want, w] = parse(reference_lens_conf(), ASPECT);
    CHECK(w.empty()); // the reference profile parses cleanly
    CHECK(want != LensConfig::defaults(ASPECT));
    CHECK(reference_lens_conf().find("lut") == std::string_view::npos); // no external LUT file needed
    const Env env = env_of({{"HOME", home}});
    {
        const auto l = load(ASPECT, env);
        CHECK(l.reference && !l.file_found);
        CHECK(l.lens.config() == want);
        CHECK(contains(l.messages[0].text, "shipped Pimax Dream Air lens profile"));
    }
    // PIMAX_NO_REFERENCE=1: the bare code defaults, as before
    {
        const auto l = load(ASPECT, env_of({{"HOME", home}, {"PIMAX_NO_REFERENCE", "1"}}));
        CHECK(!l.reference && !l.file_found);
        CHECK(l.lens.config() == LensConfig::defaults(ASPECT));
    }
    // an explicit PIMAX_LENS_CONF that does not exist keeps the code defaults (the old driver behaviour)
    CHECK(!load(ASPECT, env_of({{"HOME", home}, {"PIMAX_LENS_CONF", "/nonexistent/lens.conf"}})).reference);
    // the user's own file wins over the reference
    if (std::system(("mkdir -p '" + home + "/.config/omarchyvr'").c_str()) != 0) {
    }
    write(home + "/.config/omarchyvr/lens.conf", "fov = 40 41 42 43\n");
    {
        const auto l = load(ASPECT, env);
        CHECK(l.file_found && !l.reference);
        CHECK((l.lens.config().eye[0].fov == std::array<float, 4>{40.0f, 41.0f, 42.0f, 43.0f}));
        // keys the file does not set come from the shipped Windows fit, not the old defaults
        CHECK(l.lens.config().cant_deg == want.cant_deg);
        CHECK(l.lens.config().eye[0].cx == want.eye[0].cx);
        CHECK(l.lens.config().eye[0].k == want.eye[0].k);
        CHECK(l.lens.config().eye[0].fit_range == want.eye[0].fit_range);
    }
    pibridge::lens::testing::rmrf(home);
}

// Extra (no Rust twin): the path helpers behind the LUT lookup and PIMAX_LENS_DEBUG_GRID follow Rust's Path rules.
TEST(path_helpers_follow_rust) {
    using detail::path_parent;
    CHECK(path_parent("/a/b.conf") == std::optional<std::string>("/a"));
    CHECK(path_parent("lens.conf") == std::optional<std::string>(""));
    CHECK(path_parent("/x") == std::optional<std::string>("/"));
    CHECK(path_parent("a//b") == std::optional<std::string>("a"));
    CHECK(path_parent("a/b/") == std::optional<std::string>("a"));
    CHECK(!path_parent("/"));
    CHECK(!path_parent(""));
    const std::string dir = tmpdir("lens-mkdir");
    CHECK(detail::create_dir_all(dir + "/x/y/z").empty());
    CHECK(detail::create_dir_all(dir + "/x/y").empty());
    write(dir + "/f", "x");
    CHECK(detail::create_dir_all(dir + "/f/g") == "Not a directory (os error 20)");
    pibridge::lens::testing::rmrf(dir);
}

PIBRIDGE_TEST_MAIN()
