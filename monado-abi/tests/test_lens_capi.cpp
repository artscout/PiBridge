// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-lens/src/capi.rs (MIT OR Apache-2.0), through libpibridge_pimax.a and the
// verbatim header.
#include "pimax_lens_rs.h"

#include "pibridge_test.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

using Log = std::vector<std::pair<int, std::string>>;

void collect(void* user, int level, const char* msg) {
    static_cast<Log*>(user)->emplace_back(level, msg);
}

std::string tmpdir(const char* name) {
    const char* d = std::getenv("TMPDIR");
    std::string p = std::string(d != nullptr && *d != 0 ? d : "/tmp") + "/pibridge-lens-capi-" +
                    std::to_string(::getpid()) + "-" + name;
    ::mkdir(p.c_str(), 0755);
    return p;
}

bool exists(const std::string& p) {
    struct stat st {};
    return ::stat(p.c_str(), &st) == 0;
}

} // namespace

TEST(create_compute_destroy_with_defaults) {
    ::setenv("PIMAX_LENS_CONF", "/nonexistent/lens.conf", 1);
    ::unsetenv("PIMAX_LENS_OFF");
    ::unsetenv("PIMAX_LENS_DEBUG_GRID");
    Log log;
    pimax_lens_rs* l = pimax_lens_rs_create(3840.0f / 3552.0f, collect, &log);
    CHECK(l != nullptr);
    CHECK((log[0] == std::pair<int, std::string>{
                         0, "pimax lens: no config at /nonexistent/lens.conf, using built-in defaults"}));
    for (const auto& [lv, m] : log) {
        CHECK(lv == 0);
    }
    float fov[4] = {};
    CHECK(pimax_lens_rs_get_fov(l, 0, fov) == 0);
    // no file: the shipped Dream Air Windows fit (left eye), not the old angle table (2026-10-17)
    CHECK(fov[0] == 52.58f && fov[1] == 47.54f && fov[2] == 44.55f && fov[3] == 44.55f);
    float uv[6] = {};
    CHECK(pimax_lens_rs_compute(l, 1, 0.5f, 0.5f, uv) == 0);
    // the Rust test compares with Lens::compute of the handle; here: the same call twice (deterministic)
    float again[6] = {};
    CHECK(pimax_lens_rs_compute(l, 1, 0.5f, 0.5f, again) == 0);
    for (int i = 0; i < 6; i++) {
        CHECK(uv[i] == again[i]);
    }
    CHECK(pimax_lens_rs_has_mask(l, 0) == 0);
    // errors
    CHECK(pimax_lens_rs_get_fov(l, 2, fov) == -1);
    CHECK(pimax_lens_rs_compute(nullptr, 0, 0.0f, 0.0f, uv) == -1);
    CHECK(pimax_lens_rs_compute(l, 0, 0.0f, 0.0f, nullptr) == -1);
    std::uint32_t counts[2] = {};
    CHECK(pimax_lens_rs_visibility_mask(l, 0, 0, nullptr, 0, nullptr, 0, counts) == -3);
    pimax_lens_rs_destroy(l);
    pimax_lens_rs_destroy(nullptr);
    ::unsetenv("PIMAX_LENS_CONF");
}

TEST(mask_mesh_two_call_protocol_and_debug_grid_env) {
    const std::string dir = tmpdir("mask");
    const std::string conf = dir + "/lens.conf";
    if (std::FILE* f = std::fopen(conf.c_str(), "wb")) {
        std::fputs("mask = ellipse 1.0 0.9\n", f);
        std::fclose(f);
    }
    ::setenv("PIMAX_LENS_CONF", conf.c_str(), 1);
    ::setenv("PIMAX_LENS_DEBUG_GRID", (dir + "/grid").c_str(), 1);
    ::setenv("PIMAX_LENS_DEBUG_GRID_DIV", "32", 1);
    Log log;
    pimax_lens_rs* l = pimax_lens_rs_create(3840.0f / 3552.0f, collect, &log);
    CHECK(pimax_lens_rs_has_mask(l, 1) == 1);
    std::uint32_t counts[2] = {};
    for (int kind = 0; kind < 3; kind++) {
        CHECK(pimax_lens_rs_visibility_mask(l, 1, kind, nullptr, 0, nullptr, 0, counts) == -2);
        CHECK(counts[0] > 0 && counts[1] > 0);
        std::vector<float> v(counts[0] * 2);
        std::vector<std::uint32_t> idx(counts[1]);
        CHECK(pimax_lens_rs_visibility_mask(l, 1, kind, v.data(), counts[0], idx.data(), counts[1], counts) == 0);
        for (const auto i : idx) {
            CHECK(i < counts[0]);
        }
    }
    CHECK(pimax_lens_rs_visibility_mask(l, 1, 9, nullptr, 0, nullptr, 0, counts) == -1);
    pimax_lens_rs_destroy(l);
    bool written = false;
    for (const auto& [lv, m] : log) {
        written = written || m.find("debug grid written") != std::string::npos;
    }
    CHECK(written);
    CHECK(exists(dir + "/grid/pimax_lens_grid_left.png"));
    CHECK(exists(dir + "/grid/pimax_lens_summary.txt"));
    for (const char* k : {"PIMAX_LENS_CONF", "PIMAX_LENS_DEBUG_GRID", "PIMAX_LENS_DEBUG_GRID_DIV"}) {
        ::unsetenv(k);
    }
    if (std::system(("rm -rf '" + dir + "'").c_str()) != 0) {
    }
}

TEST(abi_version_is_exported) {
    CHECK(pimax_lens_rs_abi_version() == PIMAX_LENS_RS_ABI_VERSION);
}

PIBRIDGE_TEST_MAIN()
