// SPDX-License-Identifier: MIT
// Twins of omarchyvr-anchor world.rs `lease_tests` and `feed_tests`, plus the ring layout and the offset file.
#include "pibridge/anchor/rustx.h"
#include "pibridge/anchor/world.h"
#include "pibridge_test.h"

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <string>

using namespace pibridge;
using anchor::Pose;
using anchor::Role;
namespace g = cam::geom;

namespace {

std::string tmpdir(const std::string& tag) {
    const char* t = std::getenv("TMPDIR");
    const std::string d = std::string(t != nullptr ? t : "/tmp") + "/omarchyvr-lease-" + tag + "-" +
                          std::to_string(anchor::rs::process_id());
    std::filesystem::remove_all(d);
    std::filesystem::create_directories(d);
    return d;
}

Pose pose(double x) {
    return Pose{g::rodrigues({0.0, 0.1, 0.0}), {x, -0.9, 0.2}};
}

auto never = [](std::uint32_t) { return false; };
auto ok_evict = [](std::uint32_t) -> std::expected<void, std::string> { return {}; };
auto no_evict = [](std::uint32_t) -> std::expected<void, std::string> {
    std::fprintf(stderr, "nothing to end\n");
    std::abort();
};

bool contains(const std::string& s, const char* what) {
    return s.find(what) != std::string::npos;
}

anchor::HeadRing ring(std::uint32_t pid, std::uint64_t newest) {
    const Pose p{g::rodrigues({0.0, 0.0, 0.0}), {0.0, 1.6, 0.0}};
    std::vector<anchor::RingSample> s;
    if (newest != 0) {
        s = {{newest - 3000000, p, true}, {newest, p, true}};
    }
    return *anchor::HeadRing::parse(anchor::ring_bytes(pid, s));
}

} // namespace

TEST(lease_text_round_trip_and_schema_1) {
    const anchor::Lease l{"roommap", 1092498, 77, Role::Setup, 2};
    CHECK(anchor::Lease::parse(l.text()) == l);
    // the app's first line is unchanged (gui-core parse_owner reads the first two words)
    CHECK(l.text().starts_with("roommap 1092498\n"));
    const auto old = anchor::Lease::parse("anchor 42\n");
    CHECK(old && old->kind == "anchor" && old->session == 42 && old->writer == 0 && !old->role && old->schema == 1);
}

TEST(no_lease_no_write) {
    const std::string d = tmpdir("nowrite");
    const std::string p = d + "/pimax-anchor.offset";
    const auto e = anchor::write_offset_as(p, 5, 100, pose(0.0));
    CHECK(!e && e.error().kind == cam::ErrorKind::PermissionDenied);
    CHECK(!anchor::read_offset(p));
    CHECK(anchor::take_lease_as(p, 5, 100, Role::Reloc, never, ok_evict).has_value());
    CHECK(anchor::write_offset_as(p, 5, 100, pose(0.0)).has_value());
    // a lease of another session does not let one write this session's offset
    CHECK(!anchor::write_offset_as(p, 6, 100, pose(0.0)));
    std::filesystem::remove_all(d);
}

TEST(a_second_writer_is_refused) {
    const std::string d = tmpdir("second");
    const std::string p = d + "/pimax-anchor.offset";
    auto alive = [](std::uint32_t pid) { return pid == 200 || pid == 300; };
    CHECK(anchor::take_lease_as(p, 9, 200, Role::Follow, alive, no_evict).has_value());
    CHECK(anchor::write_offset_as(p, 9, 200, pose(0.1)).has_value());
    // auto (300) while follow (200) runs: refused, and it cannot write
    const auto e = anchor::take_lease_as(p, 9, 300, Role::Auto, alive, no_evict);
    CHECK(!e && contains(e.error(), "floor follow"));
    CHECK(!anchor::write_offset_as(p, 9, 300, pose(0.5)));
    CHECK(std::abs(anchor::read_offset(p)->second.t[0] - 0.1) < 1e-9);
    CHECK(anchor::holds_lease_as(p, 9, 200) && !anchor::holds_lease_as(p, 9, 300));
    std::filesystem::remove_all(d);
}

TEST(user_action_preempts_and_tag_rule) {
    const std::string d = tmpdir("preempt");
    const std::string p = d + "/pimax-anchor.offset";
    std::uint32_t ended = 0;
    CHECK(anchor::take_lease_as(p, 9, 200, Role::Auto, never, ok_evict).has_value());
    const auto note = anchor::take_lease_as(
        p, 9, 400, Role::Setup, [](std::uint32_t pid) { return pid == 200; },
        [&](std::uint32_t pid) -> std::expected<void, std::string> {
            ended = pid;
            return {};
        });
    CHECK(ended == 200);
    CHECK(note && *note && contains(**note, "automatic room placement"));
    CHECK(!anchor::write_offset_as(p, 9, 200, pose(0.3))); // the evicted auto cannot write
    CHECK(anchor::write_offset_as(p, 9, 400, pose(0.1)).has_value());
    // the setup ended (400 dead): the AprilTag may not take a room-map session, auto may not write again
    auto e = anchor::take_lease_as(p, 9, 500, Role::Tag, never, ok_evict);
    CHECK(!e && contains(e.error(), "Clear offset"));
    e = anchor::take_lease_as(p, 9, 600, Role::Auto, never, ok_evict);
    CHECK(!e && contains(e.error(), "already placed"));
    // another user action (Recognise room) and the floor follow may take it over from a dead holder
    CHECK(anchor::take_lease_as(p, 9, 700, Role::Follow, never, ok_evict).has_value());
    // a new session: everything starts free
    CHECK(anchor::take_lease_as(p, 10, 600, Role::Auto, never, ok_evict).has_value());
    std::filesystem::remove_all(d);
}

TEST(live_ring_of_the_running_monado) {
    const auto r = ring(43262, 100000000000ull);
    const auto f = anchor::check_feed(r, {43262}, 101000000000ull, [](std::uint32_t) { return true; });
    CHECK(f.kind == anchor::Feed::Kind::Live && f.pid == 43262);
}

TEST(foreign_writer_is_named) {
    // the 2026-10-16 case: SteamVR's driver (pid 182552) recreated the ring and closed; Monado 43262 runs on
    const auto r = ring(182552, 100000000000ull);
    const auto f = anchor::check_feed(r, {43262}, 101000000000ull, never);
    CHECK(f.kind == anchor::Feed::Kind::Foreign && f.pid == 182552 && f.monado == 43262 && !f.alive);
    CHECK(!f.live_pid());
    const std::string t = f.plain();
    CHECK(contains(t, "182552") && contains(t, "43262") && contains(t, "another program"));
}

TEST(stale_and_missing_and_no_monado) {
    const auto r = ring(43262, 100000000000ull);
    const auto f = anchor::check_feed(r, {43262}, 172000000000ull, [](std::uint32_t) { return true; });
    CHECK(f.kind == anchor::Feed::Kind::Stale && f.pid == 43262 && f.age_s && std::abs(*f.age_s - 72.0) < 0.1);
    CHECK(contains(f.plain(), "stopped 72 s ago"));
    const auto s = anchor::check_feed(ring(43262, 0), {43262}, 1, [](std::uint32_t) { return true; });
    CHECK(s.kind == anchor::Feed::Kind::Stale && !s.age_s);
    CHECK(anchor::check_feed(std::nullopt, {43262}, 1, never).kind == anchor::Feed::Kind::Missing);
    const auto n = anchor::check_feed(r, {}, 1, never);
    CHECK(n.kind == anchor::Feed::Kind::NoMonado && n.pid == 43262);
}

TEST(stale_foreign_file_on_disk) {
    const std::string d = tmpdir("feed");
    const std::string p = d + "/pimax-anchor.head";
    const std::uint64_t now = anchor::rs::monotonic_ns();
    const Pose ps{g::rodrigues({0.0, 0.3, 0.0}), {0.1, 1.5, -0.2}};
    CHECK(cam::write_file(p, anchor::ring_bytes(999999999, {{now - 60000000000ull, ps, true}})).has_value());
    const auto r = anchor::HeadRing::read(p);
    CHECK(r.has_value());
    // no monado-service with that pid can exist: foreign (or no Monado at all on a test machine)
    const auto f = anchor::check_feed(r, anchor::tracker_pids(), now, anchor::pid_alive);
    CHECK((f.kind == anchor::Feed::Kind::Foreign && f.pid == 999999999 && !f.alive) ||
          (f.kind == anchor::Feed::Kind::NoMonado && f.pid == 999999999));
    // the same file written by "the running Monado" is stale
    const auto s = anchor::check_feed(r, {999999999}, now, [](std::uint32_t) { return true; });
    CHECK(s.kind == anchor::Feed::Kind::Stale && s.age_s && *s.age_s > 59.0);
    std::filesystem::remove_all(d);
}

// Not in the Rust tests: the files on disk and the byte layout.

TEST(offset_file_names_and_bytes) {
    const std::string d = tmpdir("files");
    const std::string p = d + "/pimax-anchor.offset";
    CHECK(anchor::take_lease_as(p, 4242, 77, Role::Apply, never, ok_evict).has_value());
    const Pose o{g::rodrigues({0.0, 0.3, 0.0}), {0.1, -0.2, 0.3}};
    CHECK(anchor::write_offset_as(p, 4242, 77, o).has_value());
    CHECK(*cam::read_to_string(p) ==
          "pimax-anchor-offset 1\npid 4242\npose 0.100000 -0.200000 0.300000 0.0000000 0.1494381 0.0000000 0.9887711\n");
    CHECK(*cam::read_to_string(d + "/omarchyvr-anchor.offset.owner") ==
          "roommap 4242\nlease 2\nwriter 77\nrole apply\n");
    CHECK(cam::path_exists(d + "/omarchyvr-anchor.offset.lock"));
    CHECK(!cam::path_exists(d + "/pimax-anchor.offset.tmp") && !cam::path_exists(d + "/omarchyvr-anchor.offset.owner.tmp"));
    const auto back = anchor::read_offset(p);
    CHECK(back && back->first == 4242 && g::norm(g::sub(back->second.t, o.t)) < 1e-6);
    std::filesystem::remove_all(d);
}

TEST(ring_layout) {
    std::vector<anchor::RingSample> s;
    for (int i = 0; i < 130; i++) {
        s.push_back({1000u + static_cast<std::uint64_t>(i) * 3000000u,
                     Pose{g::rodrigues({0.0, 0.01 * i, 0.0}), {0.01 * i, 1.6, 0.0}}, i % 2 == 0});
    }
    const std::string b = anchor::ring_bytes(4321, s);
    CHECK(b.size() == 16 + 128 * 40);
    std::uint32_t h[4];
    std::memcpy(h, b.data(), 16);
    CHECK(h[0] == 0x434E4150 && h[1] == 1 && h[2] == 0 && h[3] == 4321);
    // slot 5: ts, pos x as f32, flags
    std::uint64_t ts;
    std::memcpy(&ts, b.data() + 16 + 5 * 40, 8);
    float x;
    std::memcpy(&x, b.data() + 16 + 5 * 40 + 8, 4);
    std::uint32_t fl;
    std::memcpy(&fl, b.data() + 16 + 5 * 40 + 36, 4);
    CHECK(ts == 1000 + 15000000 && x == 0.05f && fl == 0);
    const auto r = anchor::HeadRing::parse(b);
    CHECK(r && r->samples.size() == 128 && r->pid == 4321);
    // interpolation half way between two valid samples (6 ms apart: within a 100 ms gap)
    const auto at = r->at(1000 + 3000000, 100000000);
    CHECK(at && std::abs(at->t[0] - 0.01) < 1e-6);
    CHECK(!r->at(1000 + 3000000, 1000));
}

TEST(recenter_puts_the_head_at_the_origin_looking_forward) {
    for (double yaw : {0.0, 0.7, -2.9, 3.1}) {
        const Pose head{g::mat_mul(g::rodrigues({0.0, yaw, 0.0}), g::rodrigues({0.2, 0.0, 0.1})), {1.3, 1.7, -2.4}};
        const Pose o = anchor::recenter_offset(head);
        const Pose app = o.compose(head);
        CHECK(std::abs(app.t[0]) < 1e-12 && std::abs(app.t[2]) < 1e-12 && std::abs(app.t[1] - 1.7) < 1e-12);
        const g::Vec3 f = g::mat_vec(app.r, {0.0, 0.0, -1.0});
        CHECK(std::abs(f[0]) < 1e-9 && f[2] < 0.0); // forward projected on the floor is -Z
        CHECK(std::abs(o.r[1][1] - 1.0) < 1e-12);  // a rotation about +Y only
    }
}

TEST(backend_names_and_paths) {
    using namespace pibridge::anchor;
    CHECK(parse_backend("steamvr") == Backend::SteamVr && parse_backend("monado") == Backend::Monado);
    CHECK(!parse_backend("SteamVR") && !parse_backend("") && !parse_backend("both"));
    CHECK(backend_name(Backend::SteamVr) == "steamvr" && backend_name(Backend::Monado) == "monado");
    ::setenv("XDG_RUNTIME_DIR", "/run/x", 1);
    ::unsetenv("PIMAX_ANCHOR_FILE");
    ::unsetenv("PIMAX_ANCHOR_HEAD_FILE");
    ::unsetenv("PIBRIDGE_BACKEND");
    CHECK(!backend_from_env());
    CHECK(offset_path() == "/run/x/pimax-anchor.offset" && ring_path() == "/run/x/pimax-anchor.head");
    ::setenv("PIBRIDGE_BACKEND", "monado", 1);
    CHECK(backend_from_env() == Backend::Monado);
    CHECK(offset_path() == "/run/x/pibridge/monado/anchor.offset" && ring_path() == "/run/x/pibridge/monado/anchor.head");
    ::setenv("PIBRIDGE_BACKEND", "steamvr", 1);
    CHECK(offset_path() == "/run/x/pibridge/steamvr/anchor.offset");
    ::setenv("PIBRIDGE_BACKEND", "nope", 1);
    std::string bad;
    CHECK(!backend_from_env(&bad) && bad == "nope");
    CHECK(offset_path() == "/run/x/pimax-anchor.offset");
    // the explicit file still wins over the backend
    ::setenv("PIBRIDGE_BACKEND", "monado", 1);
    ::setenv("PIMAX_ANCHOR_FILE", "/elsewhere/o", 1);
    CHECK(offset_path() == "/elsewhere/o");
    ::unsetenv("PIMAX_ANCHOR_FILE");
    ::unsetenv("PIBRIDGE_BACKEND");
}

PIBRIDGE_TEST_MAIN()
