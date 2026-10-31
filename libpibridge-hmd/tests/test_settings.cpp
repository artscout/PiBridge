// SPDX-License-Identifier: MIT
// pibridge::hmd::Settings: the shared settings file of docs/CONVENTIONS.md.
#include "common.h"
#include "pibridge_test.h"

#include "pibridge/hmd/settings.h"

#include <clocale>
#include <cstdlib>

using namespace pibridge::hmd;

TEST(defaults) {
    const Settings s = Settings::parse("");
    CHECK(s.follow == "vrserver,monado-service"); // either VR runtime
    CHECK(s.refresh_hz == 90);
    CHECK(s.audio_to_headset);
    CHECK(s.eye_tracking);
    CHECK(s.eye_start_list.empty());
    CHECK(s.eye_profile.empty());
    CHECK(s.eye_flip_x);
    CHECK(s.warnings.empty());
}

TEST(reads_every_key) {
    const Settings s = Settings::parse("# pibridge settings\n"
                                       "follow = monado-service\n"
                                       "refresh_hz=72\n"
                                       "audio_to_headset=off\n"
                                       "  eye_tracking = OFF  \n"
                                       "eye_start_list=/data/eye start #1.txt\n"
                                       "eye_profile=/data/profile.bin\n"
                                       "eye_flip_x=0\n"
                                       "future_key=whatever\n"
                                       "\n");
    CHECK(s.follow == "monado-service");
    CHECK(s.refresh_hz == 72);
    CHECK(!s.audio_to_headset);
    CHECK(!s.eye_tracking);
    CHECK(s.eye_start_list == "/data/eye start #1.txt"); // the value is the rest of the line
    CHECK(s.eye_profile == "/data/profile.bin");
    CHECK(!s.eye_flip_x);
    CHECK(s.warnings.empty()); // unknown keys are ignored silently
}

TEST(empty_follow_means_always_on_and_later_lines_win) {
    const Settings s = Settings::parse("follow=vrserver\nfollow=\nrefresh_hz=72\nrefresh_hz=90\n");
    CHECK(s.follow.empty());
    CHECK(s.refresh_hz == 90);
}

TEST(booleans_and_numbers) {
    for (const char* on : {"on", "1", "true", "yes", "On", "TRUE"}) {
        CHECK(Settings::parse(std::string("audio_to_headset=off\naudio_to_headset=") + on).audio_to_headset);
    }
    for (const char* off : {"off", "0", "false", "no", "Off"}) {
        CHECK(!Settings::parse(std::string("eye_flip_x=") + off).eye_flip_x);
    }
    CHECK(Settings::parse("refresh_hz=90.0").refresh_hz == 90);
    CHECK(Settings::parse("refresh_hz=72.0\n").refresh_hz == 72);
}

TEST(vr_backend) {
    CHECK(Settings::parse("").vr_backend == "steamvr");
    CHECK(Settings::parse("vr_backend=monado\n").vr_backend == "monado");
    CHECK(Settings::parse("vr_backend=other\n").vr_backend == "steamvr");
}

TEST(bad_values_keep_the_default_with_a_warning) {
    const Settings s = Settings::parse("refresh_hz=120\naudio_to_headset=maybe\nno equals sign\nrefresh_hz=72,0\n");
    CHECK(s.refresh_hz == 90);
    CHECK(s.audio_to_headset);
    CHECK(s.warnings.size() == 4);
    CHECK(s.warnings.size() == 4 && s.warnings[0].starts_with("line 1: refresh_hz=120"));
    CHECK(s.warnings.size() == 4 && s.warnings[2].starts_with("line 3: no '='"));
}

TEST(numbers_ignore_the_locale) {
    // a comma locale must not change how "90.0" is read (std::from_chars is locale independent)
    if (std::setlocale(LC_NUMERIC, "de_DE.UTF-8") == nullptr) {
        std::setlocale(LC_NUMERIC, "C");
    }
    CHECK(Settings::parse("refresh_hz=72.0").refresh_hz == 72);
    std::setlocale(LC_NUMERIC, "C");
}

TEST(load_and_paths) {
    const auto dir = testutil::tmpdir("settings");
    CHECK(Settings::load(dir / "missing.conf").warnings.empty()); // a missing file is the defaults
    std::ofstream(dir / "pibridge.conf") << "follow=\r\nrefresh_hz=72\r\n";
    const Settings s = Settings::load(dir / "pibridge.conf");
    CHECK(s.follow.empty()); // CRLF files work too
    CHECK(s.refresh_hz == 72);
    ::setenv("XDG_CONFIG_HOME", dir.c_str(), 1);
    CHECK(Settings::default_path() == dir / "pibridge" / "pibridge.conf");
    // the file of the earlier name is read only while the new one is missing
    CHECK(Settings::legacy_path() == dir / "pimax-linux" / "pimax-linux.conf");
    std::filesystem::create_directories(dir / "pimax-linux");
    std::ofstream(dir / "pimax-linux" / "pimax-linux.conf") << "refresh_hz=72\n";
    CHECK(Settings::load_or_legacy(Settings::default_path()).refresh_hz == 72);
    CHECK(Settings::load_or_legacy(dir / "other.conf").refresh_hz == 90);
    std::filesystem::create_directories(dir / "pibridge");
    std::ofstream(dir / "pibridge" / "pibridge.conf") << "follow=x\n";
    CHECK(Settings::load_or_legacy(Settings::default_path()).refresh_hz == 90);
    ::unsetenv("XDG_CONFIG_HOME");
    ::setenv("HOME", "/nonexistent-home", 1);
    CHECK(Settings::default_path() == std::filesystem::path("/nonexistent-home/.config/pibridge/pibridge.conf"));
    std::filesystem::remove_all(dir);
}

PIBRIDGE_TEST_MAIN()
