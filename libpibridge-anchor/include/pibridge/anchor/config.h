// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-anchor/src/config.rs (MIT OR Apache-2.0).
//
// `anchor.toml`: the AprilTag and the pose it had in the app world at calibration time.
//
//   [tag]
//   family = "tag36h11"
//   id = 0
//   size_m = 0.15
//   [reference]            # world_from_tag in the app world (after any offset that was active), quat x y z w
//   pos = [0.0, 0.0, 0.0]
//   quat = [0.0, 0.0, 0.0, 1.0]
//   saved = "unix 1760000000"
//   [[window]]             # optional: window poses relative to the tag (tag_from_window)
//   name = "terminal"
//   pos = [..]
//   quat = [..]
//
// Path: PIBRIDGE_ANCHOR_CONFIG, else PIMAX_ANCHOR_CONFIG (the Rust name), else the first existing of
// ~/.config/pibridge/anchor.toml, ~/.config/pimax-linux/anchor.toml, ~/.config/omarchyvr/anchor.toml (docs/CONVENTIONS.md),
// else ~/.config/pibridge/anchor.toml. The Rust tool knows only PIMAX_ANCHOR_CONFIG and the omarchyvr path.
#pragma once

#include "pibridge/cam/geom.h"

#include <cstdint>
#include <expected>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::anchor {

struct WindowRel {
    std::string name;
    cam::geom::Pose tag_from_window;
    bool operator==(const WindowRel&) const = default;
};

struct Anchor {
    std::uint32_t id = 0;
    double size_m = 0.15;
    cam::geom::Pose reference;
    std::string saved;
    std::vector<WindowRel> windows;

    std::string to_toml() const;
    static std::expected<Anchor, std::string> parse(std::string_view text);
    static std::expected<Anchor, std::string> load();
    /// Save atomically (`anchor.toml.tmp` + rename); the path written.
    std::expected<std::string, std::string> save() const;
    bool operator==(const Anchor&) const = default;
};

/// The config path (see above).
std::string anchor_config_path();

} // namespace pibridge::anchor
