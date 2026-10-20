// SPDX-License-Identifier: MIT
// Twins of the tests in omarchyvr pimax-cam/src/imulog.rs (MIT OR Apache-2.0).
#include "pibridge/cam/imulog.h"
#include "pibridge/cam/io.h"
#include "pibridge_test.h"

#include "common.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <dirent.h>
#include <thread>
#include <unistd.h>

using namespace pibridge::cam;
using namespace pibridge::cam::imulog;

namespace {

void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

std::vector<std::string> names(const std::string& d) {
    std::vector<std::string> v;
    if (DIR* dir = ::opendir(d.c_str())) {
        while (const dirent* e = ::readdir(dir)) {
            const std::string n = e->d_name;
            if (n != "." && n != "..") {
                v.push_back(n);
            }
        }
        ::closedir(dir);
    }
    std::sort(v.begin(), v.end());
    return v;
}

void rm_rf(const std::string& p) {
    [[maybe_unused]] const int rc = std::system(("rm -rf '" + p + "'").c_str());
}

std::string link_target(const std::string& p) {
    char b[4096];
    const ssize_t n = ::readlink(p.c_str(), b, sizeof b - 1);
    return n < 0 ? std::string() : std::string(b, static_cast<std::size_t>(n));
}

} // namespace

TEST(writes_imu_and_poses_then_pauses_until_the_flag_exists) {
    const std::string dir = testing::tmp("imulog");
    rm_rf(dir);
    CHECK(create_dir_all(dir).has_value());
    const std::string p = dir + "/head-imu.csv";
    const std::string flag = dir + "/on";
    {
        // 1 s always, then only with the flag
        auto l = HeadImuLog::start(p, 1.0 / 60.0, flag);
        CHECK(l != nullptr);
        l->imu(1'000'000'000, {0.0, 9.8, 0.0}, {0.01, 0.0, 0.0});
        l->vio(1'010'000'000, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0});
        l->imu(3'000'000'000, {1.0, 9.8, 0.0}, {0.0, 0.0, 0.0}); // after the minutes, no flag: dropped
        sleep_ms(50);
        CHECK(write_file(flag, "").has_value());
        l->imu(5'000'000'000, {2.0, 9.8, 0.0}, {0.0, 0.0, 0.0}); // flag checked again (> 1 s later): written
    }
    sleep_ms(200);
    const auto h = parse(*read_to_string(p));
    CHECK(h.vio_ts == std::vector<std::int64_t>{1'010'000'000});
    std::vector<std::int64_t> ts;
    for (const auto& x : h.imu) {
        ts.push_back(x.ts);
    }
    CHECK((ts == std::vector<std::int64_t>{1'000'000'000, 5'000'000'000}));
    rm_rf(dir);
}

TEST(every_start_keeps_the_last_three_logs) {
    const std::string root = testing::tmp("imulog-rot");
    rm_rf(root);
    const std::string dir = root + "/session";
    const std::string keep = root + "/state/omarchyvr/head-imu";
    CHECK(create_dir_all(dir).has_value());
    const std::string p = dir + "/head-imu.csv";
    CHECK(!rotate(p, keep, KEEP)); // nothing to keep
    for (int i = 0; i < 5; i++) {
        const std::string text = "session " + std::to_string(i) + "\n";
        CHECK(write_file(p, text).has_value());
        // within the same second the copies get -1, -2, ...
        const auto kept = rotate(p, keep, KEEP);
        CHECK(kept.has_value());
        CHECK(!path_exists(p));
        CHECK(path_parent(*kept) == std::optional<std::string>(keep)); // kept on disk, not in the session dir
        CHECK(*read_to_string(*kept) == text);
        CHECK(*read_to_string(dir + "/head-imu.csv.prev") == text); // prev = the newest
    }
    std::vector<std::string> stamped;
    for (const auto& n : names(keep)) {
        if (n.starts_with("head-imu.2")) {
            stamped.push_back(n);
        }
    }
    CHECK(stamped.size() == 3);
    CHECK(names(dir) == std::vector<std::string>{"head-imu.csv.prev"}); // nothing else left in the session dir
    std::vector<std::string> texts;
    for (const auto& n : stamped) {
        texts.push_back(*read_to_string(keep + "/" + n));
    }
    CHECK(std::find(texts.begin(), texts.end(), "session 4\n") != texts.end() &&
          std::find(texts.begin(), texts.end(), "session 0\n") == texts.end());
    // start() keeps it by itself (the move runs on a thread: the link is right at once, then points into keep)
    CHECK(write_file(p, "session 5\n").has_value());
    CHECK(HeadImuLog::start_keeping(p, 1.0, std::nullopt, keep) != nullptr);
    // The keeper thread re-points the link (remove, then symlink) while this reads it: the Rust test reads it at
    // once; the twin waits for the link to be readable and then for it to point into keep (both libraries do the
    // rename first and the relink after it, so "no .keeping file left" alone can be seen a moment too early).
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    auto prev = [&] { return read_to_string(dir + "/head-imu.csv.prev").value_or(""); };
    while (prev().empty() && std::chrono::steady_clock::now() < end) {
        sleep_ms(1);
    }
    CHECK(prev() == "session 5\n");
    for (;;) {
        bool keeping = false;
        for (const auto& n : names(dir)) {
            keeping = keeping || n.ends_with(".keeping");
        }
        const bool linked = link_target(dir + "/head-imu.csv.prev").starts_with(keep);
        if ((!keeping && linked) || std::chrono::steady_clock::now() >= end) {
            break;
        }
        sleep_ms(10);
    }
    CHECK(prev() == "session 5\n");
    CHECK(link_target(dir + "/head-imu.csv.prev").starts_with(keep));
    CHECK(names(keep).size() == 3);
    rm_rf(root);
}

TEST(the_kept_logs_go_to_the_state_dir) {
    auto env = [](std::vector<std::pair<std::string, std::string>> m) {
        return [m](std::string_view k) -> std::optional<std::string> {
            for (const auto& [n, v] : m) {
                if (n == k) {
                    return v;
                }
            }
            return std::nullopt;
        };
    };
    CHECK(keep_dir(env({{"HOME", "/home/u"}})) == std::optional<std::string>("/home/u/.local/state/omarchyvr/head-imu"));
    CHECK(keep_dir(env({{"HOME", "/home/u"}, {"XDG_STATE_HOME", "/s"}})) ==
          std::optional<std::string>("/s/omarchyvr/head-imu"));
    CHECK(!keep_dir(env({})));
}

PIBRIDGE_TEST_MAIN()
