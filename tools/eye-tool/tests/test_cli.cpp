// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/cli.rs tests and tests/cli.rs (MIT OR Apache-2.0). The tests/cli.rs cases on the
// private fixture captures are skipped; the commands are checked end to end on a capture made from the public base
// list instead.
#include "cli.h"

#include "pibridge/eye/frames.h"
#include "pibridge/eye/pcap.h"

#include "pibridge_test.h"
#include "testutil.h"

#include <sstream>
#include <sys/stat.h>

using namespace pibridge::eye;
using namespace testutil;
namespace fs = std::filesystem;

namespace {

struct Out {
    int code;
    std::string out;
    std::string err;
};

Out run_args(const std::vector<std::string>& v) {
    std::ostringstream o, e;
    const int c = eyetool::run(v, o, e);
    return {c, o.str(), e.str()};
}

unsigned mode_of(const fs::path& p) {
    struct stat st{};
    return ::stat(p.c_str(), &st) == 0 ? (st.st_mode & 0777) : 0;
}

// A Windows-like capture of the base list's start: vendor controls, every request with its response, then gaze.
fs::path base_capture(const fs::path& dir) {
    const auto steps = frames::parse(frames::base_list()).value();
    pcap::PcapWriter w(65535);
    double t = 1.0;
    for (const auto& st : steps) {
        t += 0.001;
        if (const auto* c = std::get_if<frames::Control>(&st.action)) {
            w.write_packet(ctrl(t, c->setup, c->data));
        } else if (const auto* r = std::get_if<frames::Ttp>(&st.action)) {
            for (const auto& f : r->frames) {
                w.write_packet(bulk_out(t, f, f.size()));
            }
            w.write_packet(bulk_in(t + 0.0005, response(r->seq, r->op).to_frames(Direction::In, 4087)[0]));
        }
    }
    w.write_packet(bulk_in(t + 0.01, notification().to_frames(Direction::In, 4087)[0]));
    const auto p = dir / "base-start.pcap";
    CHECK(w.save(p).has_value());
    return p;
}

} // namespace

TEST(usage_and_errors) {
    CHECK(run_args({}).code == 1);
    CHECK(run_args({"help"}).code == 0);
    auto r = run_args({"bogus"});
    CHECK(r.code == 1 && r.err.find("unknown command") != std::string::npos);
    r = run_args({"plan", "--device", "x"});
    CHECK(r.code == 1 && r.err.find("bad number") != std::string::npos);
    r = run_args({"plan", "--device"});
    CHECK(r.code == 1 && r.err.find("needs a value") != std::string::npos);
    r = run_args({"plan", "file", "--nope"});
    CHECK(r.code == 1 && r.err.find("unexpected argument") != std::string::npos);
    CHECK(run_args({"pcap"}).code == 1);
    CHECK(run_args({"pcap", "/nonexistent/file.pcap"}).code == 2);
    r = run_args({"stream"});
    CHECK(r.code == 1 && r.err.find("not part of this tool") != std::string::npos);
    CHECK(run_args({"frames", "x.pcap"}).code == 1); // -o is required
    CHECK(run_args({"profile", "x.setpm"}).code == 1);
}

TEST(args_accept_equals_form) {
    eyetool::Args a({"--seconds=3", "file", "--print"});
    std::string e;
    CHECK(a.number<double>("--seconds", e) == std::optional<double>(3.0));
    CHECK(e.empty());
    CHECK(a.flag("--print"));
    CHECK(a.positional() == std::optional<std::string>("file"));
    CHECK(a.finish().empty());
}

TEST(pcap_command_reports_the_full_size_stream) { skip_lab_data("pcap_command_reports_the_full_size_stream"); }
TEST(pcap_command_reports_the_worn_stream_states) { skip_lab_data("pcap_command_reports_the_worn_stream_states"); }
TEST(plan_command_lists_the_cut_uploads) { skip_lab_data("plan_command_lists_the_cut_uploads"); }

TEST(frames_makes_a_private_list_that_loads_through_the_unpinned_guard) {
    const auto dir = temp_dir("eyetool-frames");
    const auto cap = base_capture(dir);
    const auto dest = dir / "eye-start-test.txt";
    const auto r = run_args({"frames", cap.string(), "-o", dest.string()});
    CHECK(r.code == 0);
    CHECK(r.out.find("23 steps from device address 30") != std::string::npos);
    CHECK(r.out.find("auth steps left out") != std::string::npos);
    CHECK(mode_of(dest) == 0600);
    const auto steps = frames::parse(read_text(dest));
    CHECK(steps.has_value() && steps->size() == 23);
    CHECK(frames::Guard::make_with(*steps, false).has_value());
    // the made list carries the base list's transfers byte for byte
    const auto base = frames::parse(frames::base_list()).value();
    std::vector<frames::Action> want, got;
    for (const auto& s : base) {
        if (!std::holds_alternative<frames::Enumeration>(s.action)) {
            want.push_back(s.action);
        }
    }
    for (const auto& s : *steps) {
        got.push_back(s.action);
    }
    CHECK(got == want);
    // an existing file with a wider mode is narrowed
    CHECK(::chmod(dest.c_str(), 0644) == 0);
    CHECK(run_args({"frames", cap.string(), "-o=" + dest.string(), "--device", "30"}).code == 0);
    CHECK(mode_of(dest) == 0600);
    fs::remove_all(dir);
}

TEST(plan_and_pcap_report_the_base_capture) {
    const auto dir = temp_dir("eyetool-plan");
    const auto cap = base_capture(dir);
    auto r = run_args({"plan", cap.string()});
    CHECK(r.code == 0);
    CHECK(r.out.find("23 steps (19 TTP requests)") != std::string::npos);
    CHECK(r.out.find("=> complete: every byte is in the capture (0 bytes missing)") != std::string::npos);
    r = run_args({"pcap", cap.string(), "--plan"});
    CHECK(r.code == 0);
    CHECK(r.out.find("replay plan: 23 steps, 19 TTP requests, 0 bytes missing -> COMPLETE") != std::string::npos);
    CHECK(r.out.find("gaze stream: 1 notifications") != std::string::npos);
    r = run_args({"pcap", cap.string(), "--csv", "x.csv"});
    CHECK(r.code == 1);
    fs::remove_all(dir);
}

TEST(frames_rejects_a_capture_without_ttp_traffic) {
    const auto dir = temp_dir("eyetool-noise");
    pcap::PcapWriter w(512);
    w.write_packet(pcap::UsbPacket::build(0.0, 2, 5, 0x81, pcap::Transfer::Interrupt, true, std::nullopt,
                                          Bytes{1, 2, 3}, 3));
    const auto f = dir / "noise.pcap";
    CHECK(w.save(f).has_value());
    auto r = run_args({"frames", f.string(), "-o", (dir / "out.txt").string()});
    CHECK(r.code == 2 && r.err.find("no eye chip start") != std::string::npos);
    CHECK(!fs::exists(dir / "out.txt"));
    r = run_args({"plan", f.string()});
    CHECK(r.code == 2 && r.err.find("no device with TTP traffic") != std::string::npos);
    fs::remove_all(dir);
}

TEST(profile_takes_the_body_out_of_a_setpm) {
    const auto dir = temp_dir("eyetool-profile");
    Bytes body(37401, 0x5a);
    const std::uint32_t lead = 37401u - 168;
    for (int i = 0; i < 4; i++) {
        body[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(lead >> (8 * i));
    }
    Bytes setpm;
    for (const std::uint32_t w : {4u, 1u, 37405u, 0x12440846u}) {
        for (int i = 0; i < 4; i++) {
            setpm.push_back(static_cast<std::uint8_t>(w >> (8 * i)));
        }
    }
    setpm.insert(setpm.end(), body.begin(), body.end());
    const auto src = dir / "calibration.setpm";
    write_bytes(src, setpm);
    const auto dest = dir / "eye-profile-test.bin";
    auto r = run_args({"profile", src.string(), "-o", dest.string()});
    CHECK(r.code == 0);
    CHECK(r.out.find("eye profile of 37401 bytes") != std::string::npos);
    CHECK(read_bytes(dest) == body);
    CHECK(mode_of(dest) == 0600);
    // a broken file is refused and nothing is written
    setpm[8] = 0;
    write_bytes(src, setpm);
    const auto dest2 = dir / "bad.bin";
    r = run_args({"profile", src.string(), "-o", dest2.string()});
    CHECK(r.code == 2 && r.err.find("header does not match") != std::string::npos);
    CHECK(!fs::exists(dest2));
    CHECK(run_args({"profile", (dir / "missing.setpm").string(), "-o", dest2.string()}).code == 2);
    fs::remove_all(dir);
}

PIBRIDGE_TEST_MAIN()
