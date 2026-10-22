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

TEST(volume_text_and_calls) {
    CHECK(parse_volume("Volume: front-left: 27525 /  42% / -22.49 dB,   front-right: 27525 /  42% / -22.49 dB") ==
          std::optional<std::string>("42%"));
    CHECK(parse_volume("Volume: mono: 65536 / 100% / 0.00 dB") == std::optional<std::string>("100%"));
    CHECK(!parse_volume("no percent here"));
    std::vector<std::vector<std::string>> calls;
    const Runner r = [&calls](const std::vector<std::string>& a) {
        calls.push_back(a);
        if (a[0] == "get-sink-volume") {
            return std::optional<std::string>("Volume: front-left: 27525 /  42% / -22.49 dB");
        }
        return std::optional<std::string>("");
    };
    CHECK(sink_volume("hs", r) == std::optional<std::string>("42%"));
    CHECK(set_sink_volume("hs", "55%", r));
    CHECK((calls.back() == std::vector<std::string>{"set-sink-volume", "hs", "55%"}));
}

TEST(volume_buttons) {
    CHECK((volume_button("[00:24:31.512,207] <info> app: vol up 1") == VolumeButton{1, true}));
    CHECK((volume_button("[00:24:31.640,001] <info> app: vol up 0") == VolumeButton{1, false}));
    CHECK((volume_button("[00:24:33.100,000] <info> app: vol down 1") == VolumeButton{-1, true}));
    CHECK((volume_button("<info> app: vol down 0") == VolumeButton{-1, false}));
    CHECK(volume_button("<error> app: tx queue full ret 0").dir == 0);
    std::string set;
    const Runner r = [&set](const std::vector<std::string>& a) {
        if (a[0] == "get-sink-volume") {
            return std::optional<std::string>("Volume: front-left: 64225 /  98% / -0.53 dB");
        }
        set = a[2];
        return std::optional<std::string>("");
    };
    CHECK(step_sink_volume("hs", 1, r) == std::optional<std::string>("99%"));
    CHECK(step_sink_volume("hs", 5, r) == std::optional<std::string>("100%")); // capped
    CHECK(step_sink_volume("hs", -2, r) == std::optional<std::string>("96%"));
    CHECK(set == "96%");
}

PIBRIDGE_TEST_MAIN()
