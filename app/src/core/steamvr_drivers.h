// SPDX-License-Identifier: MIT
// SteamVR + Monado (Tracking page, inside-out): which SteamVR driver claims the headset for one session, and putting
// everything back afterwards.
//
// In Lighthouse mode SteamVR's headset driver is CustomHeadsetOpenVR (CHOVR, core/chovr.h). driver_monado.so (the
// Monado build's steamvr-monado/ dir, which runs the Pimax driver inside vrserver) also claims the headset, so for an
// inside-out session:
//   - the Monado build's steamvr-monado dir is registered in openvrpaths.vrpath "external_drivers" (appended, unless
//     it is already there); a steamvr-monado dir of another Monado build there (a second driver named "monado") is
//     taken out for the session;
//   - CHOVR is switched off with SteamVR's own per-driver switch in steamvr.vrsettings, `"driver_CustomHeadsetOpenVR":
//     {"enable": false}`; a `"driver_monado": {"enable": false}` is set to true.
// Nothing else in either file is touched: key order, other keys and numbers stay as they are (libpibridge-room's Json).
// Before the first change: both files are copied to <file>.bak-<yyyyMMdd-HHmmss>, and the state file
// ($XDG_STATE_HOME/pibridge/steamvr-drivers.json) records exactly what is changed (the previous value of each key, or
// that it was absent; the entries added or taken out with their places), written before the files, so a crash in
// between still restores. Every file is written atomically (a temporary file renamed over it).
// restore() puts back exactly the recorded previous state of the keys PiBridge owns (other keys SteamVR added meanwhile
// stay) and removes the state file; it is idempotent (a second call, or one without a state file, does nothing).
// Neither apply() nor restore() runs while vrserver runs: SteamVR keeps both files in memory and writes them back
// when it closes, so the caller passes its process check and the switch refuses in plain words.
#pragma once

#include <QString>
#include <QStringList>
#include <ctime>
#include <expected>
#include <functional>

namespace pibridge::app {

inline constexpr char kChovrDriverName[] = "CustomHeadsetOpenVR";
inline constexpr char kMonadoDriverName[] = "monado";

// <config>/steamvr.vrsettings: the first "config" entry of the vrpath file, else ~/.local/share/Steam/config.
QString steamVrSettingsFileFor(const QString& vrpathFile, const QString& home);
// $XDG_STATE_HOME/pibridge/steamvr-drivers.json.
QString steamVrDriverStateFile();
// The "name" of <dir>/driver.vrdrivermanifest ("" when missing or broken).
QString driverManifestName(const QString& driverDir);

struct DriverSwitchFiles {
    QString vrpath;     // ~/.config/openvr/openvrpaths.vrpath
    QString vrsettings; // <SteamVR config>/steamvr.vrsettings
    QString state;      // steamVrDriverStateFile()
};

// What a session needs: register `driverDir` (named `driverName`), switch off the drivers named in `disable`.
struct DriverPlan {
    QString driverDir;                                    // <build>/steamvr-monado
    QString driverName = QString::fromLatin1(kMonadoDriverName);
    QStringList disable{QString::fromLatin1(kChovrDriverName)};
};

// ---------------------------------------------------------------- what SteamVR may pass on

// SteamVR's own variables and the ones PiBridge starts it with (omarchyvr steamvr.rs STEAMVR_ONLY, incident
// 2026-10-16: a Steam started by vrserver's `xdg-open steam://...` inherited SteamAppId=250820 and froze). The shims
// below take them out again for anything SteamVR opens.
QStringList steamVrOnlyVars();
// Of those, the ones that get the user's own value back (when the user has one) instead of being removed.
QStringList steamVrRestoredVars();
// $XDG_STATE_HOME/pibridge/steamvr-shim.
QString steamVrShimDir();
// The text of one shim: `exec env -u VAR... [VAR='user value'...] <real> "$@"`.
QString steamVrShimText(const QString& realProgram, const QStringList& userEnv);
// Writes the shims for xdg-open, steam, gio and gtk-launch (the ones `which` finds) into `dir`. `userEnv` is
// PiBridge's own environment (KEY=value). Returns the PATH SteamVR gets (`dir` first), or "" when no shim was written.
QString writeSteamVrShims(const QString& dir, const QStringList& userEnv,
                          const std::function<QString(const QString&)>& which);

class SteamVrDriverSwitch {
public:
    // `vrserverRunning`: the process check (both calls refuse while it says true). `now`: names the backups.
    SteamVrDriverSwitch(DriverSwitchFiles files, std::function<bool()> vrserverRunning,
                        std::function<std::time_t()> now = {});

    // The changes apply() would make, one plain line each (the page shows them); nothing is written.
    std::expected<QStringList, QString> preview(const DriverPlan& p) const;
    // Make the changes (see the top). Refused while a restore is pending (pending()) or vrserver runs.
    std::expected<QStringList, QString> apply(const DriverPlan& p);
    // Put back what the state file says; true when something was put back, false when there was nothing to do.
    std::expected<bool, QString> restore();
    // A state file exists: the files are switched (a session, or a crash before its restore).
    bool pending() const;
    const DriverSwitchFiles& files() const { return f_; }

private:
    DriverSwitchFiles f_;
    std::function<bool()> vrserverRunning_;
    std::function<std::time_t()> now_;
};

} // namespace pibridge::app
