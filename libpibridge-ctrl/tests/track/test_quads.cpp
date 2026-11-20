// SPDX-License-Identifier: MIT
// PiBridge: the LED quad index (quads.h) on synthetic views of the shipped reference controllers.
#include "pibridge/ctrl/track/quads.h"
#include "pibridge/ctrl/track/solve.h"
#include "pibridge_test.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

using namespace pibridge::ctrl::track;
namespace g = pibridge::cam::geom;

namespace {

struct View {
    std::vector<std::size_t> led;
    std::vector<g::Vec3> dir; // camera frame
};

/// The model seen by a camera at `cam_from_model`: the LEDs facing it (as the acquisition's facing test).
View view(const LedModel& m, const g::Pose& cam_from_model, double facing_deg) {
    View v;
    const double c = std::cos(facing_deg * M_PI / 180.0);
    for (std::size_t i = 0; i < m.leds.size(); i++) {
        if (const auto d = facing(cam_from_model, m.leds[i], c)) {
            v.led.push_back(i);
            v.dir.push_back(g::normalize(cam_from_model.apply(m.leds[i].p)));
        }
    }
    return v;
}

g::Pose random_pose(std::mt19937& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    g::Vec3 w{u(rng), u(rng), u(rng)};
    w = g::scale(g::normalize(w), M_PI * (0.5 * u(rng) + 0.5));
    std::uniform_real_distribution<double> z(0.25, 0.8);
    return g::Pose{g::rodrigues(w), {0.1 * u(rng), 0.1 * u(rng), z(rng)}};
}

/// Share of blob quads (anchor + 3 of its 5 nearest) whose true LEDs are among the index's hits.
double found_share(const LedModel& m, const QuadIndex& ix, int views, double noise_rad, std::size_t* hits_total,
                   std::size_t* quads_total) {
    std::mt19937 rng(7);
    std::normal_distribution<double> nd(0.0, noise_rad);
    std::size_t ok = 0, n = 0;
    for (int k = 0; k < views; k++) {
        const View v = view(m, random_pose(rng), 80.0);
        if (v.dir.size() < 4) {
            continue;
        }
        std::vector<g::Vec3> dir = v.dir;
        for (auto& d : dir) {
            d = g::normalize(g::add(d, {nd(rng), nd(rng), nd(rng)}));
        }
        for (std::size_t a = 0; a < dir.size(); a++) {
            std::vector<std::pair<double, std::size_t>> nb;
            for (std::size_t b = 0; b < dir.size(); b++) {
                if (b != a) {
                    nb.emplace_back(-g::dot(dir[a], dir[b]), b);
                }
            }
            std::sort(nb.begin(), nb.end());
            nb.resize(std::min<std::size_t>(nb.size(), 5));
            for (std::size_t i = 0; i < nb.size(); i++) {
                for (std::size_t j = i + 1; j < nb.size(); j++) {
                    for (std::size_t l = j + 1; l < nb.size(); l++) {
                        const std::array<std::size_t, 4> q{a, nb[i].second, nb[j].second, nb[l].second};
                        const std::array<g::Vec3, 4> d4{dir[q[0]], dir[q[1]], dir[q[2]], dir[q[3]]};
                        std::vector<QuadIndex::Hit> hits;
                        ix.lookup(d4, 0.08, hits);
                        *hits_total += hits.size();
                        n++;
                        for (const auto& h : hits) {
                            bool all = true;
                            for (int s = 0; s < 4; s++) {
                                all = all && v.led[q[h.slot[s]]] == h.led[s];
                            }
                            if (all) {
                                ok++;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    *quads_total = n;
    return n > 0 ? static_cast<double>(ok) / static_cast<double>(n) : 0.0;
}

} // namespace

TEST(quad_index_names_the_leds_of_both_reference_controllers) {
    for (const auto text : {reference_left(), reference_right()}) {
        const auto m = LedModel::parse(text);
        CHECK(m.has_value());
        const auto ix = QuadIndex::build(*m);
        std::size_t hits = 0, quads = 0;
        const double share = found_share(*m, *ix, 300, 0.0, &hits, &quads);
        std::size_t hits_n = 0, quads_n = 0;
        const double share_n = found_share(*m, *ix, 300, 1.0 / 230.0, &hits_n, &quads_n);
        std::fprintf(stderr, "%s: %zu entries; exact %.1f %% (hits/quad %.1f), 1 px noise %.1f %% (hits/quad %.1f)\n",
                     m->hand.c_str(), ix->entries(), 100.0 * share, static_cast<double>(hits) / quads, 100.0 * share_n,
                     static_cast<double>(hits_n) / quads_n);
        CHECK(share > 0.97);
        CHECK(share_n > 0.85);
    }
}

PIBRIDGE_TEST_MAIN()
