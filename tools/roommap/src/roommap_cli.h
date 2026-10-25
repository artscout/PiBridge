// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-roommap/src/main.rs (MIT OR Apache-2.0): what pibridge-roommap's command files share
// (argument helpers, the rig, the map directory, the writer lease, the offset writer, build and relocalize).
#pragma once

#include "pibridge/anchor/world.h"
#include "pibridge/cam/calib.h"
#include "pibridge/roommap/guide_ipc.h"
#include "pibridge/roommap/light.h"
#include "pibridge/roommap/session.h"
#include "pibridge/roommap/stereo.h"
#include "pibridge/roommap/store.h"

#include <cstdint>
#include <expected>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::roommap::cli {

template <class T>
using R = std::expected<T, std::string>;
using Args = std::vector<std::string>;

R<std::optional<std::string>> opt(Args& v, const std::string& name);
bool flag(Args& v, const std::string& name);
R<void> done(const Args& v);
/// `--name VALUE` parsed like Rust's `str::parse` (f64, f32, usize, u32), or the default; "NAME: bad number \"x\"".
R<double> num_f64(Args& v, const std::string& name, double d);
R<float> num_f32(Args& v, const std::string& name, float d);
R<std::size_t> num_usize(Args& v, const std::string& name, std::size_t d);
/// The calibration (--calib or the default search, printed on stderr) and the rig of cameras 0 and 1.
R<std::pair<cam::DeviceCalib, stereo::Rig>> rig(Args& v);
/// --map DIR, else --name NAME (default "home") in the room-map base directory.
R<std::string> map_dir(Args& v);
std::uint64_t unix_now();
std::string fmt_align(const session::Alignment& a);
std::vector<std::string> s(std::initializer_list<const char*> v);

/// Take (or keep) this process's writer lease of Monado session `pid`; the first role taken stays this process's role.
R<void> take_lease(std::uint32_t pid, anchor::Role role);
/// The role this process writes as (`role` unless setup / auto / follow set one).
anchor::Role writer_role(anchor::Role def);
/// Write the offset of a relocalisation result (floor, room setup), with the lease of `role`.
R<void> write_offset(const std::string& dir, const store::RelocResult& res, std::optional<std::uint32_t> pid, bool dry,
                     bool no_floor, anchor::Role role);
/// `build` with live progress for the room setup's panel (live source only). `stop_early` (optional) lets the
/// headset end the look-around walk before the timer / coverage target (the Finish chip).
R<void> cmd_build_with(Args v, const std::function<void(const light::MapProgress&)>* progress,
                       const std::function<bool()>* stop_early = nullptr,
                       // the growing scan cloud (room frame, metres) for the headset's "scanning" overlay, fed ~a few
                       // times a second during the walk; nullptr when no headset is watching (plain `build`).
                       const std::function<void(const std::vector<guide::P3f>&)>* scan = nullptr);
R<void> cmd_relocalize(Args v);
/// Why a live capture got no head poses, in plain words.
std::string no_poses_text(const std::string& what);

R<void> cmd_floor(Args v);
R<void> cmd_setup(Args v);
R<void> cmd_auto(Args v);
R<std::string> write_stage_bounds(std::uint32_t pid, std::array<double, 2> play_area);

} // namespace pibridge::roommap::cli
