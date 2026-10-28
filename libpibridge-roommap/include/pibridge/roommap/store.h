// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/store.rs (MIT OR Apache-2.0).
//
// Files: the map directory, relocalisation results and the Monado world offset (written through libpibridge-anchor, the
// PIMAX_ANCHOR=1 hook). Map directory: map.bin, roommap.toml (what it was built from), reloc-<source>.toml and
// last-reloc.toml (results), correction.csv (the mapping walk's loop-closure correction), floor.toml, room-setup.toml.
// Base directory: PIBRIDGE_ROOMMAP_DIR, else PIMAX_ROOMMAP_DIR (the Rust name), else the backend's own
// ~/.local/share/pibridge/rooms/<steamvr|monado> (PIBRIDGE_BACKEND), else ~/.local/share/pibridge/roommap, falling back
// to ~/.local/share/omarchyvr/roommap when only that one exists (docs/CONVENTIONS.md).
#pragma once

#include "pibridge/anchor/backend.h"
#include "pibridge/cam/io.h"
#include "pibridge/roommap/loopclose.h"
#include "pibridge/roommap/session.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::roommap::store {

using session::Alignment;

/// <HOME>/.local/share/pibridge/rooms/<backend>: one backend's rooms (anchor/backend.h).
std::string backend_rooms_dir(anchor::Backend b);
/// The rooms folder: PIBRIDGE_ROOMMAP_DIR / PIMAX_ROOMMAP_DIR, else the backend's (PIBRIDGE_BACKEND: backend_rooms_dir;
/// the first SteamVR use copies the rooms saved before the split), else (no backend: old tools) the shared folder.
std::string base_dir();
/// Directory of the map `name` (a name with a '/' is a path).
std::string map_dir(const std::string& name);

/// A relocalisation result (room_from_slam = the offset to write).
struct RelocResult {
    std::string map;
    std::string source;
    Alignment align;
    std::size_t frames = 0;
    std::size_t inliers = 0;
    double rms_deg = 0;
    double sigma_deg = 0;
    double sigma_m = 0;
    double fix_after_s = 0;
    std::uint32_t pid = 0;

    std::string to_toml() const;
    static std::expected<RelocResult, std::string> parse(std::string_view s);
    static std::expected<RelocResult, std::string> load(const std::string& path);
    std::expected<void, std::string> save(const std::string& path) const;
};

/// The offset Monado applies for a relocalisation.
cam::geom::Pose offset_pose(const Alignment& a);
/// Write the offset file for Monado `pid` (atomic, needs the writer lease); the path.
cam::Result<std::string> write_offset(const Alignment& a, std::uint32_t pid);
/// a * b (both yaw + translation).
Alignment compose(const Alignment& a, const Alignment& b);
Alignment inverse(const Alignment& a);
/// roommap.toml text.
std::string meta_text(const std::string& source, std::size_t keyframes, std::size_t points, std::uint64_t created_unix);
/// correction.csv (ts_ns, yaw_rad, tx, ty, tz).
std::expected<void, std::string> save_corrections(const std::string& dir, const std::vector<loopclose::Correction>& c);
/// The last correction.csv row when correction.csv belongs to the current map.bin (written with or after it).
std::optional<Alignment> last_correction(const std::string& dir);
/// The room fix of a map built in this session (the offset at mapping, moved by the loop closure).
Alignment fix_after_mapping(const Alignment& offset_at_mapping, const std::optional<Alignment>& correction);
/// Load correction.csv (empty when absent).
std::vector<loopclose::Correction> load_corrections(const std::string& dir);

} // namespace pibridge::roommap::store
