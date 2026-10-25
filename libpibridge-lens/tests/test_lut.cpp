// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/lut.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/lens/lut.h"
#include "pibridge/lens/pvrdump.h"
#include "pibridge_test.h"

#include <cmath>
#include <dirent.h>

using namespace pibridge::lens;
using pibridge::lens::testing::fixture_dump;

namespace {

std::span<const std::uint8_t> sp(const std::vector<std::uint8_t>& b) {
    return {b.data(), b.size()};
}

// The full Windows dump of the lab headset (private, not in the repo): the first `pimax-distortion-*.json` in
// $PIBRIDGE_LENS_REAL_DUMP_DIR.
std::optional<Dump> real_dump() {
    const char* d = std::getenv("PIBRIDGE_LENS_REAL_DUMP_DIR");
    if (d == nullptr) {
        return std::nullopt;
    }
    DIR* dir = ::opendir(d);
    if (dir == nullptr) {
        return std::nullopt;
    }
    std::optional<Dump> out;
    while (const dirent* e = ::readdir(dir)) {
        const std::string n = e->d_name;
        if (n.starts_with("pimax-distortion-") && n.ends_with(".json")) {
            if (const auto t = pibridge::cam::read_file(std::string(d) + "/" + n)) {
                if (auto p = parse_dump(*t)) {
                    out = std::move(*p);
                }
            }
            break;
        }
    }
    ::closedir(dir);
    return out;
}

// Keep every `step`-th grid node (and nothing else changes).
Dump decimate(const Dump& d0, std::size_t step) {
    Dump d = d0;
    for (auto& e : d.eyes) {
        const std::size_t n = e.grid_n;
        const std::size_t m = (n - 1) / step + 1;
        for (auto& g : e.grid) {
            std::vector<double> out;
            out.reserve(m * m * 2);
            for (std::size_t j = 0; j < m; j++) {
                for (std::size_t i = 0; i < m; i++) {
                    const std::size_t k = (j * step * n + i * step) * 2;
                    out.push_back(g[k]);
                    out.push_back(g[k + 1]);
                }
            }
            g = std::move(out);
        }
        e.grid_n = m;
    }
    return d;
}

// Max / rms ray angle error of a coarse LUT against the fine LUT's own nodes (off the coarse nodes).
std::pair<double, double> coarse_vs_fine(const Lut& fine, const Lut& coarse, Interp interp, bool interior_only) {
    const std::size_t n = fine.n;
    double max = 0.0;
    double sum = 0.0;
    std::size_t cnt = 0;
    for (std::size_t e = 0; e < 2; e++) {
        const auto [up, dn, l, r] = fine.meta(e).fov_tan;
        const auto vis = [&](Tan t) { return t[0] >= -l && t[0] <= r && t[1] >= -dn && t[1] <= up; };
        for (std::size_t c = 0; c < CHANNELS; c++) {
            for (std::size_t j = 0; j < n; j++) {
                for (std::size_t i = 0; i < n; i++) {
                    const float u = static_cast<float>(i) / static_cast<float>(n - 1);
                    const float v = static_cast<float>(j) / static_cast<float>(n - 1);
                    const Tan want = fine.sample(e, c, u, v, Interp::Bilinear);
                    if (!vis(want)) {
                        continue;
                    }
                    const float m = 0.03f;
                    if (interior_only && !vis({want[0] * (1.0f + m), want[1] * (1.0f + m)})) {
                        continue;
                    }
                    const Tan got = coarse.sample(e, c, u, v, interp);
                    const double a =
                        detail::ray_angle_deg({double{want[0]}, double{want[1]}}, {double{got[0]}, double{got[1]}});
                    max = std::fmax(max, a);
                    sum += a * a;
                    cnt++;
                }
            }
        }
    }
    return {max, std::sqrt(sum / static_cast<double>(cnt))};
}

} // namespace

TEST(round_trip_through_bytes_is_exact) {
    const auto dump = fixture_dump();
    if (!dump) {
        return;
    }
    const auto lut = Lut::from_dump(*dump);
    CHECK(lut.has_value());
    // the Rust test names the fixture's serial; here it is compared with the dump (no serials in the tree)
    CHECK(lut->serial == dump->serial && !lut->serial.empty());
    const auto b = lut->to_bytes();
    CHECK(std::equal(b.begin(), b.begin() + 8, LUT_MAGIC));
    const auto back = Lut::from_bytes(sp(b));
    CHECK(back.has_value());
    CHECK(back->n == lut->n);
    CHECK(back->serial == lut->serial);
    for (std::size_t e = 0; e < 2; e++) {
        CHECK(back->meta(e) == lut->meta(e));
        for (std::size_t c = 0; c < CHANNELS; c++) {
            CHECK(back->raw(e, c) == lut->raw(e, c));
            CHECK(back->data(e, c) == lut->data(e, c));
        }
    }
    CHECK(back->to_bytes() == b);
    // exact FOV port of the Dream Air
    const auto f = back->fov_deg(0);
    CHECK(std::fabs(f[0] - 52.577f) < 0.01f && std::fabs(f[1] - 47.540f) < 0.01f && std::fabs(f[2] - 44.546f) < 0.01f);
}

TEST(corrupt_files_are_rejected) {
    const auto dump = fixture_dump();
    if (!dump) {
        return;
    }
    const auto b = Lut::from_dump(*dump)->to_bytes();
    CHECK(!Lut::from_bytes(sp(b).first(b.size() - 1)));
    const std::uint8_t nope[] = {'n', 'o', 'p', 'e'};
    CHECK(!Lut::from_bytes(nope));
    auto bad = b;
    bad[200] ^= 0x40;
    CHECK(Lut::from_bytes(sp(bad)).error().find("checksum") != std::string::npos);
    bad = b;
    bad[0] = 'X';
    CHECK(Lut::from_bytes(sp(bad)).error().find("magic") != std::string::npos);
    bad = b;
    bad[8] = 9;
    CHECK(Lut::from_bytes(sp(bad)).error().find("version") != std::string::npos);
}

// The display (u, v) -> tangent must reproduce the dump at the nodes, through the orientation of each eye.
TEST(orientation_of_both_eyes) {
    const auto dump = fixture_dump();
    if (!dump) {
        return;
    }
    const auto lut = *Lut::from_dump(*dump);
    const std::size_t n = lut.n;
    for (std::size_t e = 0; e < 2; e++) {
        const EyeDump& ed = dump->eyes[e];
        const Dm& d = ORIENTS[lut.meta(e).orient].d;
        const auto [up, dn, l, r] = ed.fov_tan;
        for (const auto& [j, i] : {std::pair<std::size_t, std::size_t>{0, 0}, {3, 11}, {16, 5}, {8, 8}, {12, 2}}) {
            const double pu = static_cast<double>(i) / static_cast<double>(n - 1);
            const double pv = static_cast<double>(j) / static_cast<double>(n - 1);
            const std::size_t k = (j * n + i) * 2;
            const double w0 = -l + ed.grid[1][k] * (l + r);
            const double w1 = -dn + ed.grid[1][k + 1] * (up + dn);
            const auto [u, v] = panel_to_display(d, pu, pv);
            const Tan got = lut.sample(e, 1, static_cast<float>(u), static_cast<float>(v), Interp::Bilinear);
            CHECK(std::fabs(double{got[0]} - w0) < 1e-5);
            CHECK(std::fabs(double{got[1]} - w1) < 1e-5);
        }
    }
    // Left eye: user x = panel v, user y = -panel u.
    CHECK((panel_to_display(ORIENTS[ORIENT_LEFT].d, 1.0, 0.5) == std::pair{0.5, 1.0}));
    CHECK((panel_to_display(ORIENTS[ORIENT_LEFT].d, 0.5, 1.0) == std::pair{1.0, 0.5}));
    // Right eye: user x = -panel v, user y = panel u: mirrored in both axes.
    CHECK((panel_to_display(ORIENTS[ORIENT_RIGHT].d, 1.0, 0.5) == std::pair{0.5, 0.0}));
    CHECK((panel_to_display(ORIENTS[ORIENT_RIGHT].d, 0.5, 1.0) == std::pair{0.0, 0.5}));
    // The two eyes are mirror images: left at display (u, v) equals right at (1-u, v) with x negated.
    const Tan a = lut.sample(0, 1, 0.3f, 0.4f, Interp::Bicubic);
    const Tan b = lut.sample(1, 1, 0.7f, 0.4f, Interp::Bicubic);
    CHECK(std::fabs(a[0] + b[0]) < 1e-4f && std::fabs(a[1] - b[1]) < 1e-4f);
    // Outer side: left eye sees more to the left, so display u = 0 maps to the most negative tangent.
    const float edge = lut.sample(0, 1, 0.0f, 0.5f, Interp::Bilinear)[0];
    const float other = lut.sample(0, 1, 1.0f, 0.5f, Interp::Bilinear)[0];
    CHECK(edge < 0.0f && other > 0.0f && std::fabs(edge) > std::fabs(other) - 0.2f);
}

TEST(fixture_lines_agree_with_the_grid) {
    // 17x17 grid (every 8th node): coarse, but bicubic must stay within a fraction of a degree
    const auto dump = fixture_dump();
    if (!dump) {
        return;
    }
    const auto lut = *Lut::from_dump(*dump);
    const auto bl = line_error(lut, *dump, Interp::Bilinear);
    const auto bc = line_error(lut, *dump, Interp::Bicubic);
    CHECK(bl.samples > 500);
    CHECK(bc.rms_deg < 0.3 && bl.rms_deg < 0.6);
}

TEST(interpolation_error_on_the_real_dump) {
    const auto dump = real_dump();
    if (!dump) {
        std::fprintf(stderr, "real dump not present, skipping\n");
        return;
    }
    const auto lut = *Lut::from_dump(*dump);
    const auto bl = line_error(lut, *dump, Interp::Bilinear);
    const auto bc = line_error(lut, *dump, Interp::Bicubic);
    // interpolation error between the nodes, estimated from a half resolution LUT against the full grid
    const auto coarse = *Lut::from_dump(decimate(*dump, 2));
    for (const bool interior : {false, true}) {
        const auto x = coarse_vs_fine(lut, coarse, Interp::Bilinear, interior);
        const auto y = coarse_vs_fine(lut, coarse, Interp::Bicubic, interior);
        std::fprintf(stderr, "half-res LUT, interior_only=%d: bilinear max %.4f rms %.4f; bicubic max %.4f rms %.4f\n",
                     interior, x.first, x.second, y.first, y.second);
    }
    CHECK(bl.rms_deg < 0.02 && bc.rms_deg < 0.02);
    CHECK(bl.max_deg < 0.12 && bc.max_deg < 0.12);
}

TEST(rotated_yaw_turns_every_ray_and_the_port) {
    const auto dump = fixture_dump();
    if (!dump) {
        std::fprintf(stderr, "fixture dump not present, skipping\n");
        return;
    }
    const auto lut = *Lut::from_dump(*dump);
    const double th = std::atan(0.0819);
    const auto rot = lut.rotated_yaw({th, -th});
    // through the bytes and back: still a valid file with the same grid
    const auto b = rot.to_bytes();
    const auto back = Lut::from_bytes(sp(b));
    CHECK(back.has_value() && back->n == lut.n && back->serial == lut.serial);
    for (std::size_t e = 0; e < 2; e++) {
        const double a = e == 0 ? th : -th;
        // each ray keeps its angle to the others: compare the 3D directions, turned back
        for (std::size_t c = 0; c < CHANNELS; c++) {
            const auto& r0 = lut.raw(e, c);
            const auto& r1 = rot.raw(e, c);
            double worst = 0;
            for (std::size_t k = 0; k < r0.size(); k++) {
                const double x = r1[k][0], y = r1[k][1];
                // turn (x, y, 1) back by -a
                const double xb = x * std::cos(a) - std::sin(a), zb = std::cos(a) + x * std::sin(a);
                worst = std::max({worst, std::fabs(xb / zb - double{r0[k][0]}), std::fabs(y / zb - double{r0[k][1]})});
            }
            CHECK(worst < 1e-4);
        }
        // the axis: a ray straight ahead before is turned by exactly `a`
        const auto [up, dn, l, r] = lut.meta(e).fov_tan;
        const auto [up2, dn2, l2, r2] = rot.meta(e).fov_tan;
        CHECK(std::fabs(std::atan(double{r2}) - (std::atan(double{r}) + a)) < 1e-5);
        CHECK(std::fabs(std::atan(double{l2}) - (std::atan(double{l}) - a)) < 1e-5);
        CHECK(up2 > up && dn2 > dn && std::fabs(double{up2} / up - double{dn2} / dn) < 1e-5);
    }
    // zero turn = the same rays
    const auto same = lut.rotated_yaw({0.0, 0.0});
    for (std::size_t e = 0; e < 2; e++) {
        CHECK(same.meta(e) == lut.meta(e));
        CHECK(same.raw(e, 1) == lut.raw(e, 1));
    }
}

// The Crystal Super profile (crystal-pimax-native-lut.bin, headset-verified 2026-10-22) is the like-Pimax LUT turned
// by +-atan(0.0819). Both are private (made from the user's own Pimax install, never in the repo): the test runs when
// $PIBRIDGE_LENS_EXT_DIR (default ~/gamedev/ext/lens) has them; the turn must reproduce the verified file bit for bit.
TEST(the_crystal_super_profile_is_the_like_pimax_lut_turned) {
    const char* d = std::getenv("PIBRIDGE_LENS_EXT_DIR");
    const char* home = std::getenv("HOME");
    const std::string dir = d != nullptr ? d : std::string(home != nullptr ? home : "") + "/gamedev/ext/lens";
    const auto src = Lut::load(dir + "/crystal-pimax-lut.bin");
    const auto want = Lut::load(dir + "/crystal-pimax-native-lut.bin");
    if (!src || !want) {
        std::fprintf(stderr, "private lens files not present in %s, skipping\n", dir.c_str());
        return;
    }
    const double th = std::atan(0.0819);
    const auto got = src->rotated_yaw({th, -th});
    for (std::size_t e = 0; e < 2; e++) {
        for (std::size_t i = 0; i < 4; i++) {
            CHECK(got.meta(e).fov_tan[i] == want->meta(e).fov_tan[i]);
        }
        double worst = 0;
        for (std::size_t c = 0; c < CHANNELS; c++) {
            const auto& g = got.raw(e, c);
            const auto& w = want->raw(e, c);
            for (std::size_t k = 0; k < g.size(); k++) {
                const double err = std::max(std::fabs(double{g[k][0]} - w[k][0]), std::fabs(double{g[k][1]} - w[k][1]));
                worst = std::max(worst, err);
            }
        }
        std::fprintf(stderr, "eye %zu: max node difference %.2e\n", e, worst);
        CHECK(worst == 0.0);
    }
}

PIBRIDGE_TEST_MAIN()
