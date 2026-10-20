// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/src/{descriptor,module}.rs and devices/dream_air.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/descriptor.h"
#include "pibridge_env.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include <unistd.h>

#ifndef PIBRIDGE_INSTALLED_DEVICES_DIR
#define PIBRIDGE_INSTALLED_DEVICES_DIR "/usr/local/share/pibridge/devices"
#endif

namespace pibridge::hmd {
namespace fs = std::filesystem;

namespace {

std::unexpected<SpecError> fail(std::string m) { return std::unexpected(SpecError{std::move(m)}); }

std::expected<Support, SpecError> parse_support(const std::optional<std::string>& s) {
    const std::string v = s.value_or("unknown");
    if (v == "works") return Support::Works;
    if (v == "experimental") return Support::Experimental;
    if (v == "none") return Support::None;
    if (v == "unknown") return Support::Unknown;
    return fail("support \"" + v + "\" (works|experimental|none|unknown)");
}

std::expected<std::uint16_t, SpecError> u16_of(std::int64_t v, const std::string& what) {
    if (v < 0 || v > 0xffff) {
        return fail(what + " " + std::to_string(v) + " out of u16");
    }
    return static_cast<std::uint16_t>(v);
}

std::expected<const toml::Section*, SpecError> table(const std::vector<toml::Section>& secs, std::string_view name) {
    for (const auto& s : secs) {
        if (!s.array && s.name == name) {
            return &s;
        }
    }
    return fail("missing [" + std::string(name) + "]");
}

} // namespace

std::string_view support_label(Support s) {
    switch (s) {
    case Support::Works: return "works";
    case Support::Experimental: return "experimental";
    case Support::None: return "not on this model";
    case Support::Unknown: return "unknown";
    }
    return "unknown";
}

bool is_onboarding_step(std::string_view id) {
    static constexpr std::string_view steps[] = {"detect", "checks", "fixes", "udev", "ipd",
                                                 "pair",   "eye_cal", "room",  "done"};
    return std::find(std::begin(steps), std::end(steps), id) != std::end(steps);
}

std::uint8_t hand_slot(Hand h) { return h == Hand::Left ? SLOT_LEFT : SLOT_RIGHT; }

std::expected<Descriptor, SpecError> Descriptor::parse(std::string_view src) {
    const auto parsed = toml::parse(src);
    if (!parsed) {
        return fail(parsed.error().message());
    }
    const auto& secs = *parsed;
#define PIBRIDGE_TRY(var, expr)                                                                                              \
    auto var##_r = (expr);                                                                                             \
    if (!var##_r) return std::unexpected(var##_r.error());                                                             \
    auto var = std::move(*var##_r)

    PIBRIDGE_TRY(dev, table(secs, "device"));
    const auto need = [](const toml::Section& s, std::string_view k) -> std::expected<std::string, SpecError> {
        if (auto v = s.str(k)) {
            return *v;
        }
        return fail("[" + s.name + "] lacks " + std::string(k));
    };
    Descriptor d;
    PIBRIDGE_TRY(status, need(*dev, "status"));
    if (status == "real") {
        d.status = DeviceStatus::Real;
    } else if (status == "mock") {
        d.status = DeviceStatus::Mock;
    } else {
        return fail("status \"" + status + "\"");
    }
    const auto pids = [&](std::string_view k) -> std::expected<std::vector<std::uint16_t>, SpecError> {
        std::vector<std::uint16_t> v;
        for (const auto p : dev->ints(k)) {
            auto x = u16_of(p, std::string(k));
            if (!x) {
                return std::unexpected(x.error());
            }
            v.push_back(*x);
        }
        return v;
    };
    PIBRIDGE_TRY(disp, table(secs, "display"));
    PIBRIDGE_TRY(trk, table(secs, "tracking"));
    PIBRIDGE_TRY(ctl, table(secs, "controllers"));
    PIBRIDGE_TRY(eye, table(secs, "eye_tracking"));
    PIBRIDGE_TRY(ipd, table(secs, "ipd"));
    PIBRIDGE_TRY(cam, table(secs, "cameras"));
    PIBRIDGE_TRY(aud, table(secs, "audio"));
    auto& c = d.caps;
    PIBRIDGE_TRY(s_disp, parse_support(disp->str("status")));
    c.display = s_disp;
    if (const auto mode = disp->ints("mode"); mode.size() == 2) {
        c.mode = std::pair{static_cast<std::uint32_t>(mode[0]), static_cast<std::uint32_t>(mode[1])};
    }
    for (const auto v : disp->ints("refresh_hz")) {
        c.refresh_hz.push_back(static_cast<std::uint32_t>(v));
    }
    c.edid_vendor = disp->str("edid_vendor");
    if (const auto p = disp->integer("edid_product")) {
        PIBRIDGE_TRY(ep, u16_of(*p, "edid_product"));
        c.edid_product = ep;
    }
    c.display_needs = disp->strs("needs");
    PIBRIDGE_TRY(s_trk, parse_support(trk->str("status")));
    c.tracking = s_trk;
    c.head_tracking = trk->str("head").value_or("unknown");
    PIBRIDGE_TRY(s_ctl, parse_support(ctl->str("status")));
    c.controllers = s_ctl;
    for (const auto v : ctl->ints("slots")) {
        c.controller_slots.push_back(static_cast<std::uint8_t>(v));
    }
    c.pairing = ctl->str("pairing").value_or("unknown");
    PIBRIDGE_TRY(s_eye, parse_support(eye->str("status")));
    c.eye_tracking = s_eye;
    if (const auto v = eye->integer("usb_vid"), p = eye->integer("usb_pid"); v && p) {
        PIBRIDGE_TRY(ev, u16_of(*v, "usb_vid"));
        PIBRIDGE_TRY(epid, u16_of(*p, "usb_pid"));
        c.eye_usb = std::pair{ev, epid};
    }
    PIBRIDGE_TRY(s_ipd, parse_support(ipd->str("status")));
    c.ipd = s_ipd;
    c.ipd_motor = ipd->boolean("motor").value_or(false);
    {
        const auto a = ipd->number("min_mm");
        const auto b = ipd->number("max_mm");
        if (a && b && *a < *b) {
            c.ipd_range_mm = std::pair{static_cast<float>(*a), static_cast<float>(*b)};
        } else if (a || b) {
            return fail("[ipd] min_mm/max_mm");
        }
    }
    PIBRIDGE_TRY(s_cam, parse_support(cam->str("status")));
    c.cameras = s_cam;
    c.camera_count = static_cast<std::uint32_t>(cam->integer("count").value_or(0));
    PIBRIDGE_TRY(s_aud, parse_support(aud->str("status")));
    c.audio = s_aud;
    c.audio_sink_match = aud->strs("sink_match");
    c.audio_source_match = aud->strs("source_match");

    for (const auto& s : secs) {
        if (!s.array) {
            continue;
        }
        if (s.name == "hid") {
            PIBRIDGE_TRY(spec, CommandSpec::from_section(s));
            if (d.allowlist.get(spec.id)) {
                return fail("hid id " + spec.id + " twice");
            }
            d.allowlist.commands.push_back(std::move(spec));
        } else if (s.name == "hid_denied") {
            PIBRIDGE_TRY(prefix, need(s, "prefix"));
            PIBRIDGE_TRY(why, need(s, "why"));
            PIBRIDGE_TRY(den, Denied::parse(prefix, why));
            d.allowlist.denied.push_back(std::move(den));
        } else {
            return fail("unknown [[" + s.name + "]]");
        }
    }
    PIBRIDGE_TRY(id, need(*dev, "id"));
    PIBRIDGE_TRY(name, need(*dev, "name"));
    PIBRIDGE_TRY(vendor, need(*dev, "vendor"));
    d.id = id;
    d.name = name;
    d.vendor = vendor;
    const auto vid = dev->integer("usb_vid");
    if (!vid) {
        return fail("[device] lacks usb_vid");
    }
    PIBRIDGE_TRY(uv, u16_of(*vid, "usb_vid"));
    d.usb_vid = uv;
    PIBRIDGE_TRY(up, pids("usb_pids"));
    d.usb_pids = std::move(up);
    PIBRIDGE_TRY(upc, pids("usb_pid_candidates"));
    d.usb_pid_candidates = std::move(upc);
    d.command_interface = static_cast<std::uint8_t>(dev->integer("command_interface").value_or(0));
    PIBRIDGE_TRY(calsec, table(secs, "calibration"));
    d.calibration_paths = calsec->strs("paths");
    PIBRIDGE_TRY(onb, table(secs, "onboarding"));
    d.onboarding = onb->strs("steps");
#undef PIBRIDGE_TRY
    if (auto v = d.validate(); !v) {
        return std::unexpected(v.error());
    }
    return d;
}

std::expected<void, SpecError> Descriptor::validate() const {
    const auto e = [&](const std::string& m) { return fail(id + ": " + m); };
    if (status == DeviceStatus::Mock) {
        if (!allowlist.commands.empty()) {
            return e("a mock may not declare HID commands");
        }
        if (!usb_pids.empty()) {
            return e("a mock may not claim USB ids (use usb_pid_candidates)");
        }
        const std::pair<const char*, Support> all[] = {{"display", caps.display},   {"tracking", caps.tracking},
                                                       {"controllers", caps.controllers}, {"ipd", caps.ipd},
                                                       {"cameras", caps.cameras},   {"audio", caps.audio},
                                                       {"eye", caps.eye_tracking}};
        for (const auto& [n, s] : all) {
            if (s == Support::Works || s == Support::Experimental) {
                return e(std::string("a mock may not promise ") + n);
            }
        }
    } else if (usb_pids.empty()) {
        return e("a real module needs usb_pids");
    }
    for (const auto& s : onboarding) {
        if (!is_onboarding_step(s)) {
            return e("unknown onboarding step " + s);
        }
    }
    if (caps.ipd_motor && !allowlist.get("ipd_motor")) {
        return e("ipd.motor = true needs an ipd_motor command");
    }
    if (caps.pairing == "start_pair" && !allowlist.get("start_pair")) {
        return e("pairing = start_pair needs a start_pair command");
    }
    return {};
}

std::expected<Descriptor, SpecError> Descriptor::load(const fs::path& path) {
    std::ifstream f(path);
    if (!f) {
        return fail(path.string() + ": cannot read");
    }
    std::stringstream ss;
    ss << f.rdbuf();
    auto d = parse(ss.str());
    if (!d) {
        return fail(path.filename().string() + ": " + d.error().msg);
    }
    return d;
}

bool Descriptor::matches_usb(std::uint16_t vid, std::uint16_t pid) const {
    return vid == usb_vid && std::find(usb_pids.begin(), usb_pids.end(), pid) != usb_pids.end();
}

std::expected<std::vector<std::uint8_t>, Refused> Descriptor::ipd_report(float mm) const {
    if (!caps.ipd_range_mm) {
        Refused e{Refused::Kind::UnknownCommand};
        e.cmd = "ipd_motor";
        return std::unexpected(e);
    }
    if (!std::isfinite(mm)) {
        Refused e{Refused::Kind::OutOfRange};
        e.cmd = "ipd_motor";
        e.param = "ipd";
        return std::unexpected(e);
    }
    const auto [lo, hi] = *caps.ipd_range_mm;
    const auto v = static_cast<std::uint32_t>(std::round(std::clamp(mm, lo, hi) * 100.0f));
    return allowlist.build("ipd_motor", {{"ipd", v}});
}

std::expected<std::vector<std::uint8_t>, Refused> Descriptor::pair_report(Hand hand) const {
    if (!allowlist.get("start_pair")) {
        Refused e{Refused::Kind::UnknownCommand};
        e.cmd = "start_pair";
        return std::unexpected(e);
    }
    return allowlist.build("start_pair", {{"slot", hand_slot(hand)}});
}

std::expected<std::vector<std::uint8_t>, Refused> Descriptor::keepalive_report(std::uint8_t hz) const {
    return allowlist.build("keepalive", {{"rate", hz}});
}

std::vector<fs::path> devices_dirs() {
    std::vector<fs::path> v;
    if (const char* e = pibridge::env("DEVICES_DIR"); e && *e) {
        v.emplace_back(e);
    }
    v.emplace_back(PIBRIDGE_INSTALLED_DEVICES_DIR);
    std::error_code ec;
    const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        v.push_back(exe.parent_path().parent_path() / "share/pibridge/devices");
    }
    return v;
}

std::expected<Descriptor, SpecError> load_descriptor(const Model& model, const std::vector<fs::path>& dirs) {
    const std::string file = std::string(model.id) + ".toml";
    for (const auto& d : dirs) {
        std::error_code ec;
        if (fs::exists(d / file, ec)) {
            return Descriptor::load(d / file);
        }
    }
    return fail("no device description " + file + " (looked in the pibridge devices folders)");
}

std::expected<std::shared_ptr<const Allowlist>, SpecError> load_allowlist(const Model& model,
                                                                         const std::vector<fs::path>& dirs) {
    auto d = load_descriptor(model, dirs);
    if (!d) {
        return std::unexpected(d.error());
    }
    return std::make_shared<const Allowlist>(std::move(d->allowlist));
}

Detected detect(const std::vector<Descriptor>& descs, std::uint16_t vid, std::uint16_t pid) {
    for (const auto& d : descs) {
        if (d.matches_usb(vid, pid)) {
            return Detected{Detected::Kind::Module, &d};
        }
    }
    for (const auto& d : descs) {
        if (d.usb_vid == vid &&
            std::find(d.usb_pid_candidates.begin(), d.usb_pid_candidates.end(), pid) != d.usb_pid_candidates.end()) {
            return Detected{Detected::Kind::Possibly, &d};
        }
    }
    return {};
}

} // namespace pibridge::hmd
