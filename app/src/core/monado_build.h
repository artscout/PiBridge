// SPDX-License-Identifier: MIT
// Which Monado build a start uses (Development page, "Monado build"), what it is, and whether it may realtime.
//
// Two builds are A/B'd (docs/monado-abi.md, "The PiBridge-linked Monado (M9)"): the live one that links omarchyvr's Rust
// archives (~/gamedev/ovr-build/monado) and the one that links PiBridge's C++ libpibridge_pimax.a
// (~/gamedev/pibridge-monado-build). The choice is the existing key rec_monado_service (the monado-service binary); the
// SteamVR driver dir (steamvr-monado/) and the OpenXR runtime file derive from it. Record tracking, "Monado
// directly" and the Tracking page's SteamVR + Monado start all use it.
//
// What the build is comes from the binary itself:
//   - the PIBRIDGE_PIMAX_ABI marker string ("PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 5ffbcc5") is only in the PiBridge-linked
//     build: "C++ (PiBridge)"; without it "Rust";
//   - the required markers of omarchyvr's preflight (omarchyvr-gui-core builds.rs / preflight.rs, plus the two the
//     Crystal settings need): a build that lost one is refused in plain words (omarchyvr lesson 2026-10-15: a build
//     configured from scratch had the Pimax driver off).
// Scanning a 200+ MB binary takes a moment, so the result is cached by path, size and modification time.
//
// cap_sys_nice: monado-service raises its compositor threads to realtime priority; without the capability it runs
// with normal priority (judder under load). Every relink drops it. The state is read from the file's
// security.capability attribute (no libcap); "Fix permissions" runs `pkexec setcap cap_sys_nice=ep <binary>` through
// the Runner, only when the user presses it (never automatically, never in a test: FakeRunner).
#pragma once

#include <QByteArray>
#include <QString>
#include <QStringList>
#include <functional>
#include <optional>

#include "runner.h"

namespace pibridge::app {

// The markers every Monado build must carry (omarchyvr's preflight list, plus PIMAX_AE_MODEL and
// PIMAX_SLAM_DRIFT_HOLD for the Crystal Super's camera exposure and drift-hold switches).
QStringList requiredMonadoMarkers();
inline constexpr char kPiBridgeAbiMarker[] = "PIBRIDGE_PIMAX_ABI";

enum class BuildKind { Unknown, Rust, PiBridge };

struct MonadoBuildInfo {
    bool checked = false;     // the binary was read (false: not found / not readable / not looked at)
    bool checking = false;    // being read on a worker thread right now (monadoBuildInfoAsync)
    QString path;             // monado-service
    QString error;            // why it could not be read
    BuildKind kind = BuildKind::Unknown;
    QString abiText;          // the whole PIBRIDGE_PIMAX_ABI string, "" in a Rust build
    QStringList missing;      // required markers not in the binary
    bool ok() const { return checked && missing.isEmpty(); }
};

// Scans `data` (a whole binary) for the markers (pure; tested on made-up bytes).
MonadoBuildInfo scanMonadoBinary(const QByteArray& data, const QString& path = {});
// Reads and scans the file (streamed, cached by path + size + mtime). An empty path gives an unchecked info.
MonadoBuildInfo monadoBuildInfo(const QString& monadoService);
// The same without blocking: the cached info when it is fresh; otherwise a worker thread scans the file (one per
// path at a time) and the result says `checking` until the cache has it (`done` runs on that thread then). Used by
// the app so a 200+ MB binary is never read on the UI thread.
MonadoBuildInfo monadoBuildInfoAsync(const QString& monadoService, std::function<void()> done = {});
// "C++ (PiBridge)", "Rust", "unknown".
QString buildKindWords(BuildKind k);
// One plain line for the page: "C++ (PiBridge): PIBRIDGE_PIMAX_ABI 1 spec omarchyvr 5ffbcc5. All 7 required markers present."
QString buildSummary(const MonadoBuildInfo& b);
// The blocking reason when a required marker is missing ("" when the build is fine or was not checked).
QString buildBlockingReason(const MonadoBuildInfo& b);

// ---------------------------------------------------------------- the A/B choice

struct MonadoBuildChoice {
    QString id;      // "auto" | "rust" | "pibridge"
    QString label;   // "Rust-linked (live)" ...
    QString service; // the monado-service of that build ("" for auto)
};
// The builds the selector offers under `home`: auto (the key cleared), the live Rust-linked build and the
// PiBridge-linked one (each with its monado-service path, whether it exists or not).
std::vector<MonadoBuildChoice> monadoBuildChoices(const QString& home);
// Which choice a rec_monado_service value is ("" = auto; a path of one of the builds; anything else: "other").
QString monadoBuildChoiceOf(const QString& confValue, const QString& home);
// The Monado build folder that holds `monadoService` (<build>/src/xrt/targets/service/monado-service, or an installed
// <prefix>/bin/monado-service -> <prefix>/lib/omarchyvr/monado or the prefix): the first of it and up to four
// folders above that has steamvr-monado/; "" when none.
QString steamVrDriverDirFor(const QString& monadoService);

// ---------------------------------------------------------------- cap_sys_nice

// CAP_SYS_NICE = 23 (linux/capability.h).
inline constexpr int kCapSysNice = 23;
struct CapState {
    bool readable = false;   // the attribute could be read (absent counts as readable: no capabilities)
    bool permitted = false;  // cap_sys_nice in the permitted set
    bool effective = false;  // the file's effective flag
    bool ok() const { return permitted && effective; } // "cap_sys_nice=ep"
};
// Parses a security.capability value (struct vfs_cap_data, revisions 1..3, little endian); an empty value = no
// capabilities.
CapState parseFileCaps(const QByteArray& xattr);
// getxattr(path, "security.capability").
CapState fileCaps(const QString& path);
// "Realtime priority allowed (cap_sys_nice=ep)." / "Missing: ..." in plain words.
QString capWords(const CapState& c, bool fileExists);

// The command "Fix permissions" runs: pkexec setcap cap_sys_nice=ep <monadoService>.
QStringList capFixArgs(const QString& monadoService);
inline constexpr char kPkexec[] = "pkexec";
// Runs it through `runner` (only when the user pressed the button) and calls `done` with plain words. pkexec's
// exit codes: 126 = the prompt was dismissed, 127 = not authorised / no agent.
void runCapFix(Runner& runner, const QString& monadoService, std::function<void(bool ok, const QString& words)> done);

} // namespace pibridge::app
