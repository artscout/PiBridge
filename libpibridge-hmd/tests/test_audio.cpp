// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-tools/src/audio.rs tests (MIT OR Apache-2.0), plus to_headset/restore against a
// fake pactl (no PipeWire is touched).
#include "pibridge_test.h"

#include "pibridge/hmd/audio.h"

#include <map>

using namespace pibridge::hmd::audio;

TEST(picks_the_headset_nodes_by_prefix) {
    const std::string sinks = "59\talsa_output.usb-Pimax_Corporation_Pimax_Crystal_Super_X-04.pro-output-1\tPipeWire\ts16le\n"
                              "72\talsa_output.usb-Logitech_PRO_X_2-00.analog-stereo\tPipeWire\ts16le\n";
    CHECK(pick(sinks, SINK_PREFIX) ==
          std::optional<std::string>("alsa_output.usb-Pimax_Corporation_Pimax_Crystal_Super_X-04.pro-output-1"));
    const std::string sources = "1\talsa_output.usb-Pimax_Corporation_A.pro-output-1.monitor\tPipeWire\n"
                                "60\talsa_input.usb-Pimax_Corporation_A-04.pro-input-0\tPipeWire\n";
    CHECK(pick(sources, SOURCE_PREFIX) == std::optional<std::string>("alsa_input.usb-Pimax_Corporation_A-04.pro-input-0"));
    CHECK(!pick("72\talsa_output.usb-Logitech-00\tPipeWire\n", SINK_PREFIX));
}

/// A tiny PipeWire: default sink/source and the node lists.
struct FakePactl {
    std::string sink = "desk-speakers";
    std::string source = "desk-mic";
    bool headset = true;
    std::vector<std::string> calls;

    Runner runner() {
        return [this](const std::vector<std::string>& a) -> std::optional<std::string> {
            std::string c;
            for (const auto& x : a) {
                c += (c.empty() ? "" : " ") + x;
            }
            calls.push_back(c);
            if (c == "get-default-sink") return sink;
            if (c == "get-default-source") return source;
            if (c == "list short sinks") {
                return std::string("1\tdesk-speakers\tPipeWire\n") +
                       (headset ? "2\talsa_output.usb-Pimax_Corporation_X.pro-output-1\tPipeWire\n" : "");
            }
            if (c == "list short sources") {
                return std::string("3\tdesk-mic\tPipeWire\n") +
                       (headset ? "4\talsa_input.usb-Pimax_Corporation_X.pro-input-0\tPipeWire\n" : "");
            }
            if (a.size() == 2 && a[0] == "set-default-sink") {
                sink = a[1];
                return std::string{};
            }
            if (a.size() == 2 && a[0] == "set-default-source") {
                source = a[1];
                return std::string{};
            }
            return std::nullopt;
        };
    }
};

TEST(to_headset_and_back) {
    FakePactl p;
    const auto [msg, before] = to_headset(p.runner());
    CHECK(msg == "audio: headset is the default output and input");
    CHECK(p.sink == "alsa_output.usb-Pimax_Corporation_X.pro-output-1");
    CHECK(p.source == "alsa_input.usb-Pimax_Corporation_X.pro-input-0");
    CHECK(before.sink == std::optional<std::string>("desk-speakers"));
    CHECK(before.our_source == std::optional<std::string>("alsa_input.usb-Pimax_Corporation_X.pro-input-0"));
    CHECK(restore(before, p.runner()) == "audio: back to output desk-speakers, input desk-mic");
    CHECK(p.sink == "desk-speakers" && p.source == "desk-mic");
}

TEST(restore_leaves_a_device_the_user_picked) {
    FakePactl p;
    const auto [msg, before] = to_headset(p.runner());
    p.sink = "usb-headphones"; // the user changed the output meanwhile
    CHECK(restore(before, p.runner()) == "audio: back to input desk-mic");
    CHECK(p.sink == "usb-headphones");
}

TEST(no_headset_nodes) {
    FakePactl p;
    p.headset = false;
    const auto [msg, before] = to_headset(p.runner());
    CHECK(msg == "audio: no headset output/input found (pactl list short sinks)");
    CHECK(!before.our_sink && !before.our_source);
    CHECK(restore(before, p.runner()) == "audio: nothing to give back");
    CHECK(p.sink == "desk-speakers");
}

TEST(pactl_missing_is_harmless) {
    const Runner none = [](const std::vector<std::string>&) { return std::optional<std::string>{}; };
    const auto [msg, before] = to_headset(none);
    CHECK(msg.starts_with("audio: no headset"));
    CHECK(before == Before{});
}

PIBRIDGE_TEST_MAIN()
