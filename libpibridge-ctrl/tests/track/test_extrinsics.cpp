// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-ctrltrack/src/extrinsics.rs (MIT OR Apache-2.0).
#include "common.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"
#include "pibridge/ctrl/track/extrinsics.h"
#include "pibridge_test.h"

#include <cmath>

using namespace pibridge::ctrl::track;
namespace g = pibridge::cam::geom;
namespace ex = pibridge::ctrl::track::extrinsics;
using pibridge::cam::DeviceCalib;
using pibridge::cam::RigConvention;
using pibridge::ctrl::track::testing::tmp;

namespace {
const Env none = [](std::string_view) -> std::optional<std::string> { return std::nullopt; };

double angle_deg(const g::Pose& a, const g::Pose& b) {
    const g::Mat3 d = g::mat_mul(g::transpose(a.r), b.r);
    const double tr = d[0][0] + d[1][1] + d[2][2];
    return pibridge::cam::rfmt::to_degrees(std::acos(pibridge::cam::rfmt::clamp((tr - 1.0) / 2.0, -1.0, 1.0)));
}

std::string replace(std::string s, const std::string& from, const std::string& to) {
    const auto p = s.find(from);
    if (p != std::string::npos) {
        s.replace(p, from.size(), to);
    }
    return s;
}
} // namespace

TEST(roundtrip) {
    const std::string p = tmp("ext.json");
    const g::Pose pose{g::rodrigues({0.01, -0.02, 0.03}), {0.1, 0.2, -0.3}};
    const ex::CamPoses v{{2, pose}};
    CHECK(ex::save(p, "UID", v, "test").has_value());
    const auto l = ex::load(p);
    CHECK(l.has_value());
    if (!l) {
        return;
    }
    CHECK(l->first == "UID");
    CHECK(l->second[0].first == 2);
    for (int i = 0; i < 3; i++) {
        CHECK(std::fabs(l->second[0].second.t[i] - pose.t[i]) < 1e-12);
        for (int j = 0; j < 3; j++) {
            CHECK(std::fabs(l->second[0].second.r[i][j] - pose.r[i][j]) < 1e-12);
        }
    }
    std::remove(p.c_str());
}

TEST(the_reference_fit_corrects_cams_1_to_3_by_the_documented_angles) {
    const auto p = ex::parse(ex::reference_json());
    CHECK(p.has_value());
    if (!p) {
        return;
    }
    CHECK(p->first == "reference");
    CHECK(ex::reference_json().find("/home/") == std::string_view::npos);
    const auto calib = DeviceCalib::parse(pibridge::cam::reference_xml());
    CHECK(calib.has_value());
    const std::pair<std::size_t, double> want[] = {{1, 0.46}, {2, 1.45}, {3, 3.20}};
    CHECK(p->second.size() == 3);
    for (std::size_t i = 0; i < p->second.size() && i < 3; i++) {
        const auto& [id, fitted] = p->second[i];
        CHECK(id == want[i].first);
        const g::Pose factory = calib->body_from_cam(id, RigConvention::CamFromCam0)->inverse();
        const double a = angle_deg(factory, fitted);
        CHECK(std::fabs(a - want[i].second) < 0.01);
    }
}

TEST(the_reference_fit_applies_only_to_the_reference_headset) {
    const auto calib = DeviceCalib::parse(pibridge::cam::reference_xml());
    CHECK(calib && ex::same_rig_as_reference(*calib));
    const auto r = ex::reference_for(*calib, none);
    CHECK(r && r->size() == 3);
    // the same headset under its own serial: the rig decides, not the uid
    const auto own = DeviceCalib::parse(replace(std::string(pibridge::cam::reference_xml()), "deviceUID=\"reference\"",
                                                "deviceUID=\"P511X0X000000000001\""));
    CHECK(own && !own->is_reference());
    CHECK(own && ex::reference_for(*own, none).has_value());
    // another headset (one camera rig differs): its factory extrinsics stay
    DeviceCalib other = *calib;
    other.cameras[3].rig_r[0][1] += 0.01;
    CHECK(!ex::reference_for(other, none));
    // PIMAX_NO_REFERENCE=1 turns the reference off
    const Env off = [](std::string_view k) -> std::optional<std::string> {
        return k == "PIMAX_NO_REFERENCE" ? std::optional<std::string>("1") : std::nullopt;
    };
    CHECK(!ex::reference_for(*calib, off));
}

TEST(resolve_prefers_the_users_own_fit_then_the_reference_then_factory) {
    const auto calib = DeviceCalib::parse(pibridge::cam::reference_xml());
    const std::string dir = tmp("resolve");
    CHECK(pibridge::cam::create_dir_all(dir).has_value());
    const std::string missing = dir + "/missing.json";
    // a clean install: no own file -> the shipped reference fit
    auto [v, note] = ex::resolve(*calib, missing, none);
    CHECK(v.size() == 3);
    CHECK(note.find("shipped Dream Air reference") != std::string::npos);
    // the user's own fit wins
    const std::string own = dir + "/own.json";
    const g::Pose pose{g::rodrigues({0.0, 0.01, 0.0}), {0.0, 0.0, 0.0}};
    const ex::CamPoses one{{2, pose}};
    CHECK(ex::save(own, "", one, "test").has_value());
    auto r2 = ex::resolve(*calib, own, none);
    CHECK(r2.first.size() == 1);
    CHECK(r2.second.find("own.json") != std::string::npos);
    // a fit for another headset is ignored, the reference applies
    CHECK(ex::save(own, "P511X0X000000000002", one, "test").has_value());
    CHECK(ex::resolve(*calib, own, none).first.size() == 3);
    // another headset without a fit of its own: factory
    DeviceCalib other = *calib;
    other.cameras[2].rig_t[0] += 0.01;
    auto r3 = ex::resolve(other, missing, none);
    CHECK(r3.first.empty());
    CHECK(r3.second == "factory extrinsics");
    pibridge::ctrl::track::testing::rmrf(dir);
}

PIBRIDGE_TEST_MAIN()
