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

// `pactl list cards` (C locale) of the test PC with the original Crystal on: the NVIDIA audio card, two monitors and
// the headset (its monitor name, from the ELD, is "Pimax"), cut down to the lines that matter.
static const std::string CARDS = "Card #61\n"
                                 "\tName: alsa_card.pci-0000_01_00.1\n"
                                 "\tProperties:\n"
                                 "\t\tdevice.product.name = \"GB202 High Definition Audio Controller\"\n"
                                 "\tProfiles:\n"
                                 "\t\toutput:hdmi-stereo: Digital Stereo (HDMI) Output (sinks: 1, sources: 0)\n"
                                 "\tActive Profile: output:hdmi-stereo\n"
                                 "\tPorts:\n"
                                 "\t\thdmi-output-0: HDMI / DisplayPort (type: HDMI, priority: 5900, available)\n"
                                 "\t\t\tProperties:\n"
                                 "\t\t\t\tdevice.product.name = \"Q32G3WG3R3\"\n"
                                 "\t\t\tPart of profile(s): output:hdmi-stereo\n"
                                 "\t\thdmi-output-2: HDMI / DisplayPort 3 (type: HDMI, priority: 5700, available)\n"
                                 "\t\t\tProperties:\n"
                                 "\t\t\t\tport.type = \"hdmi\"\n"
                                 "\t\t\t\tdevice.product.name = \"Pimax\"\n"
                                 "\t\t\tPart of profile(s): output:hdmi-stereo-extra2, output:hdmi-surround-extra2\n"
                                 "\n"
                                 "Card #447\n"
                                 "\tName: alsa_card.usb-PIMAX_Pimax_Crystal_7f348d4c-00\n"
                                 "\tActive Profile: input:analog-stereo\n";

TEST(finds_the_crystal_displayport_output) {
    const auto dp = parse_dp_port(CARDS);
    CHECK(dp == std::optional<DpPort>(DpPort{"alsa_card.pci-0000_01_00.1", "output:hdmi-stereo-extra2", "output:hdmi-stereo"}));
    CHECK(dp && dp->sink() == "alsa_output.pci-0000_01_00.1.hdmi-stereo-extra2");
    CHECK(!parse_dp_port("Card #1\n\tName: alsa_card.x\n\tPorts:\n\t\thdmi-output-0: HDMI\n"
                         "\t\t\t\tdevice.product.name = \"CQ32G4\"\n\t\t\tPart of profile(s): output:hdmi-stereo\n"));
}

/// The original Crystal: no USB output node, a microphone, and the GPU audio card whose profile picks the sink.
struct FakeCrystalPactl {
    std::string sink = "alsa_output.pci-0000_01_00.1.hdmi-stereo"; // the monitor's sound
    std::string source = "desk-mic";
    std::string profile = "output:hdmi-stereo";

    Runner runner() {
        return [this](const std::vector<std::string>& a) -> std::optional<std::string> {
            if (a == std::vector<std::string>{"get-default-sink"}) return sink;
            if (a == std::vector<std::string>{"get-default-source"}) return source;
            if (a == std::vector<std::string>{"list", "short", "sinks"}) {
                return "1\talsa_output.pci-0000_01_00.1." + profile.substr(7) + "\tPipeWire\n";
            }
            if (a == std::vector<std::string>{"list", "short", "sources"}) {
                return std::string("3\tdesk-mic\tPipeWire\n4\talsa_input.usb-PIMAX_Pimax_Crystal_7f348d4c-00.analog-stereo\tPipeWire\n");
            }
            if (a == std::vector<std::string>{"list", "cards"}) {
                std::string c = CARDS;
                c.replace(c.find("Active Profile: output:hdmi-stereo\n"), 35, "Active Profile: " + profile + "\n");
                return c;
            }
            if (a.size() == 3 && a[0] == "set-card-profile" && a[1] == "alsa_card.pci-0000_01_00.1") {
                profile = a[2];
                return std::string{};
            }
            if (a.size() == 2 && a[0] == "set-default-sink" && a[1] == "alsa_output.pci-0000_01_00.1." + profile.substr(7)) {
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

TEST(crystal_sound_goes_to_its_displayport_and_back) {
    FakeCrystalPactl p;
    const auto [msg, before] = to_headset(p.runner());
    CHECK(msg == "audio: headset is the default output and input");
    CHECK(p.profile == "output:hdmi-stereo-extra2");
    CHECK(p.sink == "alsa_output.pci-0000_01_00.1.hdmi-stereo-extra2");
    CHECK(p.source == "alsa_input.usb-PIMAX_Pimax_Crystal_7f348d4c-00.analog-stereo");
    CHECK(headset_sink(p.runner()) == std::optional<std::string>("alsa_output.pci-0000_01_00.1.hdmi-stereo-extra2"));
    // the card's profile first: the monitor's sink exists again before it is made the default
    CHECK(restore(before, p.runner()) ==
          "audio: back to card output:hdmi-stereo, output alsa_output.pci-0000_01_00.1.hdmi-stereo, input desk-mic");
    CHECK(p.profile == "output:hdmi-stereo" && p.sink == "alsa_output.pci-0000_01_00.1.hdmi-stereo");
    CHECK(!headset_sink(p.runner()));
}
