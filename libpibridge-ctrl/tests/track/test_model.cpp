// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/model.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/cam/io.h"
#include "pibridge/ctrl/track/model.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl::track;
using pibridge::ctrl::track::testing::tmp;

TEST(parses_windows_file) {
    const char* t = R"({"Hand":"Left","ModelPoints":{"Point0":[0.01,0,0,1,0,0],"Point1":[0,0.01,0,0,2,0],
            "Point2":[0,0,0.01,0,0,1],"Point3":[0.01,0.01,0,1,0,0]},
            "ImuToDevice":[1,0,0,0.5, 0,1,0,0, 0,0,1,0, 0,0,0,1],"SN":"x"})";
    const auto m = LedModel::parse(t);
    CHECK(m.has_value());
    if (!m) {
        return;
    }
    CHECK(m->leds.size() == 4);
    CHECK(m->hand == "Left");
    CHECK(std::fabs(m->leds[1].n[1] - 1.0) < 1e-12);
    CHECK((m->imu_to_device.t == pibridge::cam::geom::Vec3{0.5, 0.0, 0.0}));
    CHECK(m->neighbour_triples(1.0, 180.0).size() == 24);
}

TEST(the_shipped_reference_models_are_a_mirrored_left_right_pair) {
    const auto l = LedModel::parse(reference_left());
    const auto r = LedModel::parse(reference_right());
    CHECK(l && r);
    if (!l || !r) {
        return;
    }
    CHECK(l->hand == "Left" && r->hand == "Right");
    CHECK(l->leds.size() == 15 && r->leds.size() == 15);
    // left and right are mirror images in x
    for (std::size_t i = 0; i < l->leds.size(); i++) {
        const auto& a = l->leds[i].p;
        const auto& b = r->leds[i].p;
        CHECK(std::fabs(a[0] + b[0]) < 1e-6 && std::fabs(a[1] - b[1]) < 1e-6 && std::fabs(a[2] - b[2]) < 1e-6);
    }
}

TEST(the_users_own_models_win_and_the_reference_is_the_fallback) {
    const std::string home = tmp("models");
    CHECK(pibridge::cam::create_dir_all(home).has_value());
    const Env env = [&](std::string_view k) -> std::optional<std::string> {
        return k == "HOME" ? std::optional<std::string>(home) : std::nullopt;
    };
    // nothing of the user's own: the shipped reference
    auto m = LedModel::resolve(std::nullopt, std::nullopt, env);
    CHECK(m && !m->second.files);
    CHECK(m && m->first[0].hand == "Left" && m->first[1].hand == "Right");
    CHECK(m && m->second.describe().find("reference") != std::string::npos);
    // reference switched off: a plain error that names where the files go
    const Env off = [&](std::string_view k) -> std::optional<std::string> {
        if (k == "HOME") {
            return home;
        }
        if (k == "PIMAX_NO_REFERENCE") {
            return "1";
        }
        return std::nullopt;
    };
    const auto e = LedModel::resolve(std::nullopt, std::nullopt, off);
    CHECK(!e && e.error().find("controller models missing") != std::string::npos &&
          e.error().find(".config/omarchyvr/controllers/left.json") != std::string::npos);
    // the user's own, in Pimax's layout (one folder per controller serial; name order = left, right)
    const std::string dir = home + "/.config/omarchyvr/controllers";
    for (const auto& [sn, text] : {std::pair{"A1", reference_left()}, std::pair{"B2", reference_right()}}) {
        CHECK(pibridge::cam::create_dir_all(dir + "/" + sn).has_value());
        CHECK(pibridge::cam::write_file(dir + "/" + sn + "/controller_params.json", text).has_value());
    }
    m = LedModel::resolve(std::nullopt, std::nullopt, env);
    CHECK(m && m->second.files ==
                   (std::array<std::string, 2>{dir + "/A1/controller_params.json", dir + "/B2/controller_params.json"}));
    CHECK(m && m->first[1].hand == "Right");
    // an explicit --left wins over the folder
    const std::string l = home + "/given-left.json";
    CHECK(pibridge::cam::write_file(l, reference_left()).has_value());
    m = LedModel::resolve(l, std::nullopt, env);
    CHECK(m && m->second.files == (std::array<std::string, 2>{l, dir + "/B2/controller_params.json"}));
    // a broken own file is an error, not a silent fallback
    CHECK(pibridge::cam::write_file(dir + "/A1/controller_params.json", "{").has_value());
    CHECK(!LedModel::resolve(std::nullopt, std::nullopt, env));
    pibridge::ctrl::track::testing::rmrf(home);
}

TEST(the_hand_field_wins_over_the_folder_order) {
    const std::string dir = tmp("hand");
    for (const auto& [sn, text] : {std::pair{"A", reference_right()}, std::pair{"B", reference_left()}}) {
        CHECK(pibridge::cam::create_dir_all(dir + "/" + sn).has_value());
        CHECK(pibridge::cam::write_file(dir + "/" + sn + "/controller_params.json", text).has_value());
    }
    const auto p = LedModel::pair_in(dir);
    CHECK(p == (std::array<std::string, 2>{dir + "/B/controller_params.json", dir + "/A/controller_params.json"}));
    pibridge::ctrl::track::testing::rmrf(dir);
}

// not in the Rust file: folder names compare as path components ("a" before "a.b", unlike the joined strings)
TEST(folder_order_is_component_order) {
    const std::string dir = tmp("order");
    for (const auto& [sn, text] : {std::pair{"a.b", reference_left()}, std::pair{"a", reference_left()}}) {
        CHECK(pibridge::cam::create_dir_all(dir + "/" + sn).has_value());
        CHECK(pibridge::cam::write_file(dir + "/" + sn + "/controller_params.json", text).has_value());
    }
    const auto p = LedModel::pair_in(dir);
    CHECK(p && (*p)[0] == dir + "/a/controller_params.json");
    pibridge::ctrl::track::testing::rmrf(dir);
}

PIBRIDGE_TEST_MAIN()
