// SPDX-License-Identifier: MIT
// Ported from ZoneTweak main.go, area.go, features.go and tool.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
#include "pibridge/room/editor.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>

namespace pibridge::room {

namespace {

constexpr Vec3 kUp{0, 1, 0};
constexpr double kPi = std::numbers::pi;

std::string upper(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
}

std::string onoff(bool b) { return b ? "ON" : "OFF"; }

double remainder2pi(double a) { return std::remainder(a, 2 * kPi); }

} // namespace

// ------------------------------------------------------------------------------------------------ zone file

FileZoneControl::FileZoneControl(std::string settingsPath, std::string stateDir, std::function<std::time_t()> clock)
    : file_(std::move(settingsPath)), stateDir_(std::move(stateDir)), clock_(std::move(clock)) {}

ZoneValues FileZoneControl::values() {
    const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (!cache_ || now - cachedAt_ > 1.0) {
        auto v = file_.read();
        cache_ = v ? *v : ZoneValues{};
        cachedAt_ = now;
    }
    return *cache_;
}

std::expected<double, std::string> FileZoneControl::setWarnDistance(std::optional<double> metres) {
    cache_.reset();
    return file_.setWarnDistance(metres, clock_());
}

std::expected<void, std::string> FileZoneControl::setZoneOn(bool on) {
    cache_.reset();
    return file_.setZoneOn(on, stateDir_, clock_());
}

// ------------------------------------------------------------------------------------------------ hands

std::optional<Pt2> Hand::rawLaser() const {
    if (!aimValid) {
        return std::nullopt;
    }
    const Vec3 o = aim.pos();
    const Vec3 dir = aim.axis(2) * -1;
    if (dir.y > -0.03) {
        return std::nullopt;
    }
    const double t = -o.y / dir.y;
    if (t > 15 || t < 0) {
        return std::nullopt;
    }
    return Pt2{o.x + dir.x * t, o.z + dir.z * t};
}

void Hand::updateLaser(double dt) {
    const auto raw = rawLaser();
    if (!raw) {
        lf.reset();
        lok = false;
        return;
    }
    lc = lf.step(*raw, dt);
    lok = true;
}

std::optional<Pt2> Hand::pointer(bool laserMode) const {
    if (!laserMode) {
        return cursor();
    }
    if (!lok) {
        return std::nullopt;
    }
    return lc;
}

Rigid2 handPose2(const Mat34& m) {
    const Vec3 f = m.axis(2) * -1;
    const Vec3 p = m.pos();
    return rigidFromYaw(std::atan2(f.z, f.x), p.x, p.z);
}

Rigid2 rigidOfPose(const Mat34& m) { return {m.m[0][0], m.m[0][2], m.m[2][0], m.m[2][2], m.m[0][3], m.m[2][3]}; }

// ------------------------------------------------------------------------------------------------ editor

Editor::Editor(Store& store, RoomLib l, EditorHost host) : st(store), lib(std::move(l)), host_(std::move(host)) {
    if (!host_.now) {
        host_.now = [] { return 0.0; };
    }
    if (!host_.wallClock) {
        host_.wallClock = [] { return std::time(nullptr); };
    }
    hands.resize(2);
    hands[0].idx = 0;
    hands[1].idx = 1;
    buildPages();
    loadFromStore();
}

void Editor::haptic(int hand, int us) {
    if (host_.haptic) {
        host_.haptic(hand, us / 100, 0.6F);
    }
}

void Editor::say(const std::string& s) {
    status_ = upper(s);
    statusAt_ = host_.now();
}

void Editor::push() {
    undo.push_back(pts);
    if (undo.size() > 200) {
        undo.erase(undo.begin());
    }
    redo.clear();
    dirty = true;
}

void Editor::releaseAll() {
    for (Hand& h : hands) {
        h.grab = -1;
        h.rayGrab = false;
    }
}

void Editor::setPts(Poly p, const std::string& msg) {
    if (p.size() < 3) {
        say("That would leave fewer than 3 points");
        return;
    }
    push();
    pts = std::move(p);
    theta = domAngle(pts);
    releaseAll();
    if (!msg.empty()) {
        say(msg);
    }
}

void Editor::doUndo() {
    if (undo.empty()) {
        say("Nothing to undo");
        return;
    }
    redo.push_back(pts);
    pts = undo.back();
    undo.pop_back();
    releaseAll();
    dirty = true;
    say("Undo");
}

void Editor::doRedo() {
    if (redo.empty()) {
        say("Nothing to redo");
        return;
    }
    undo.push_back(pts);
    pts = redo.back();
    redo.pop_back();
    releaseAll();
    dirty = true;
    say("Redo");
}

std::string Editor::fmtLen(double m) const {
    if (feet) {
        return std::format("{:.1f} FT", m * 3.28084);
    }
    return std::format("{:.2f} M", m);
}

std::string Editor::fmtArea(double a) const {
    if (feet) {
        return std::format("{:.0f} SQ FT", a * 10.7639);
    }
    return std::format("{:.1f} M2", a);
}

void Editor::loadFromStore() {
    pts = st.stagePts();
    orig = pts;
    height = st.height();
    theta = domAngle(pts);
    undo.clear();
    redo.clear();
    dirty = false;
    area = st.playArea();
    center = st.adjust();
    centerSet = true;
    centerDirty = areaDirty = false;
    releaseAll();
}

Rigid2 Editor::centerPose() {
    if (!centerSet) {
        center = st.adjust();
        centerSet = true;
    }
    return center;
}

std::array<Pt2, 4> Editor::areaCorners(const Rigid2& g, double w, double h) const {
    return {g.apply({-w / 2, -h / 2}), g.apply({w / 2, -h / 2}), g.apply({w / 2, h / 2}), g.apply({-w / 2, h / 2})};
}

bool Editor::save() {
    const bool movedCenter = centerDirty;
    SaveExtras ex;
    if (movedCenter) {
        ex.standing = st.standingForCenter(centerPose());
    }
    if (areaDirty || movedCenter) {
        ex.playArea = area;
    }
    if (keepFloor && floorY != 0) {
        ex.floorDelta = floorY;
    }
    auto r = st.save(pts, height, ex, host_.wallClock());
    if (!r) {
        say("Save failed: " + r.error());
        return false;
    }
    if (movedCenter) {
        st.setAdj(center);
    }
    if (ex.floorDelta != 0) {
        floorY = 0; // the runtime's floor follows once it reloads the file
    }
    centerDirty = areaDirty = false;
    orig = pts;
    dirty = false;
    savedAny = true;
    std::string extra = host_.afterSave ? host_.afterSave(*r) : std::string();
    say(std::format("Saved to {} file(s){}", r->written.size(), extra.empty() ? std::string() : ". " + extra));
    return true;
}

void Editor::useSpace(const Room& r) {
    loadSpace(r);
    if (save()) {
        lib.setActive(r.name);
        say("Using " + r.name);
    }
}

void Editor::loadSpace(const Room& r) {
    Poly p = r.points;
    if (r.frame == "raw" || r.frame.empty()) {
        p = st.toStageAll(r.points);
    }
    setPts(p, "");
    if (r.height > 0) {
        height = r.height;
    }
    if (r.playArea) {
        area = *r.playArea;
        areaDirty = true;
    }
    if (r.standing) {
        center = st.centerForStanding(*r.standing);
        centerDirty = true;
    }
}

std::expected<void, std::string> Editor::saveAsNew(const std::string& name) {
    Room r;
    r.name = name;
    r.universe = st.universe();
    r.height = height;
    r.points = st.toFileAll(pts);
    r.frame = "raw";
    Standing s = st.standingForCenter(centerPose());
    s.y = st.standY();
    r.standing = s;
    r.playArea = area;
    if (host_.zone != nullptr) {
        r.warningDistance = host_.zone->values().fade;
    }
    if (st.mode() == FrameMode::Off && centerDirty) {
        // the points are standing coordinates of the standing pose the room carries
        const Rigid2 sNew = rigidFromYaw(-s.yaw, s.x, s.z);
        r.points = sNew.then(st.standingRigid().inv()).inv().all(r.points);
    }
    if (auto e = lib.save(r, host_.wallClock()); !e) {
        return e;
    }
    lib.setActive(r.name);
    return {};
}

void Editor::fitArea() {
    const Rigid2 g = centerPose();
    const auto s = room::fitArea(g.inv().all(pts), 0.1);
    if (!s) {
        say("Put the center inside the boundary first");
        return;
    }
    area = *s;
    areaDirty = true;
    say(std::format("Play area {} x {}", fmtLen(s->w), fmtLen(s->h)));
}

void Editor::centerInRoom() {
    const auto c = centroid(pts);
    if (!c) {
        return;
    }
    const Rigid2 g = centerPose();
    center = {g.a, g.b, g.c, g.d, c->x, c->z};
    centerDirty = true;
    say("Center moved to the middle of the room");
}

void Editor::squareToRoom() {
    const Rigid2 g = centerPose();
    const double now = std::atan2(g.c, g.a);
    const double th = domAngle(pts);
    double best = th;
    for (int k = -4; k <= 4; k++) {
        const double cand = th + k * kPi / 2;
        if (std::fabs(remainder2pi(cand - now)) < std::fabs(remainder2pi(best - now))) {
            best = cand;
        }
    }
    center = rigidFromYaw(best, g.tx, g.tz);
    centerDirty = true;
    say("Play area turned to match the walls");
}

void Editor::resetArea() {
    center = st.adjust();
    area = st.playArea();
    centerDirty = areaDirty = false;
    say("Center and area back to saved");
}

void Editor::restoreOriginal() {
    auto b = st.oldestBackup();
    if (!b) {
        say(b.error());
        return;
    }
    setPts(b->stagePts, std::format("Restored {} points from backup. Save to keep", b->stagePts.size()));
}

void Editor::selectUniverse(const std::string& id) {
    if (auto r = st.selectUniverse(id); !r) {
        say(r.error());
        return;
    }
    st.setAdj({});
    loadFromStore();
    say(std::format("Universe {}, {} points", id, pts.size()));
}

void Editor::cycleFrameMode() {
    const auto next = static_cast<FrameMode>((static_cast<int>(st.mode()) + 1) % kFrameModes);
    const FrameMode from = st.mode();
    Poly o;
    for (const Pt2 q : orig) {
        o.push_back(st.toStageMode(next, st.toFileMode(from, q)));
    }
    pts = st.setMode(next, pts);
    orig = std::move(o);
    lib.setFrameMode(next);
    theta = domAngle(pts);
    releaseAll();
    say(std::string("Align ") + frameModeName(next) + ". Check that the walls sit on SteamVR's own");
}

void Editor::buildPages() {
    pageMain = {2, 0.056,
                [this] {
                    std::string l1 = std::format("UNIVERSE {}  {} PTS  {}", st.universe(), pts.size(),
                                                 fmtArea(std::fabs(polyArea(pts))));
                    if (dirty) {
                        l1 += "  UNSAVED";
                    }
                    std::string hs;
                    for (const Hand& h : hands) {
                        if (h.valid) {
                            hs += (hs.empty() ? "" : " / ") + std::format("{:.2f}", h.m.pos().y);
                        }
                    }
                    std::string act = lib.active();
                    if (act.empty()) {
                        act = "-";
                    }
                    return std::vector<std::string>{l1, "ROOM " + upper(act) + "   CONTROLLER HEIGHT " + hs + " M"};
                },
                [this] {
                    std::vector<Button> b;
                    b.push_back({[] { return std::string("SAVE"); }, [this] { save(); }});
                    b.push_back({[] { return std::string("REVERT"); }, [this] {
                                     setPts(orig, "Back to the saved boundary");
                                     dirty = false;
                                 }});
                    b.push_back({[] { return std::string("UNDO"); }, [this] { doUndo(); }});
                    b.push_back({[] { return std::string("REDO"); }, [this] { doRedo(); }});
                    b.push_back({[] { return std::string("SQUARE UP"); }, [this] {
                                     Poly p = squareUp(simplify(pts, 0.10), domAngle(pts), 0.20);
                                     if (p.empty()) {
                                         say("Couldn't find 4 clear walls");
                                         return;
                                     }
                                     const auto n = p.size();
                                     setPts(std::move(p), std::format("Squared up into {} walls", n));
                                 }});
                    b.push_back({[] { return std::string("SIMPLIFY"); }, [this] {
                                     const auto n = pts.size();
                                     Poly p = simplify(pts, 0.08);
                                     if (p.size() == n) {
                                         say("Nothing to simplify");
                                         return;
                                     }
                                     const auto m = p.size();
                                     setPts(std::move(p), std::format("Simplified {} to {} points", n, m));
                                 }});
                    b.push_back({[this] { return std::string(drawing ? "CANCEL OUTLINE" : "NEW OUTLINE"); },
                                 [this] {
                                     drawing = !drawing;
                                     tracing = false;
                                     draft.clear();
                                     releaseAll();
                                     if (drawing) {
                                         menuOpen = false;
                                         say("Walk to each corner and pull the trigger");
                                     } else {
                                         say("Outline cancelled");
                                     }
                                 }});
                    b.push_back({[this] { return std::string(tracing ? "CANCEL TRACE" : "TRACE OUTLINE"); },
                                 [this] {
                                     tracing = !tracing;
                                     drawing = false;
                                     draft.clear();
                                     trace = {};
                                     releaseAll();
                                     if (tracing) {
                                         menuOpen = false;
                                         say("Hold the trigger and walk along the walls. Let go to finish");
                                     } else {
                                         say("Trace cancelled");
                                     }
                                 },
                                 1, [this] { return tracing; }});
                    b.push_back({[this] { return "SNAP: " + onoff(snap); }, [this] { snap = !snap; }, 1,
                                 [this] { return snap; }});
                    b.push_back({[] { return std::string("SAVED SPACES"); }, [this] { page = &pageSpaces; }});
                    b.push_back({[] { return std::string("ZONE SETTINGS"); }, [this] { page = &pageZone; }});
                    b.push_back({[this] {
                                     return floorY != 0 ? std::format("FIX FLOOR ({:+.0f} CM)", floorY * 100)
                                                        : std::string("FIX FLOOR");
                                 },
                                 [this] {
                                     floorAt = host_.now() + 3;
                                     menuOpen = false;
                                 }});
                    b.push_back({[this] { return std::string(laser ? "POINTER: LASER" : "POINTER: DOWN"); },
                                 [this] { laser = !laser; }, 1, [this] { return laser; }});
                    b.push_back({[this] { return std::string(feet ? "UNITS: FEET" : "UNITS: METERS"); },
                                 [this] { feet = !feet; }});
                    b.push_back({[this] { return "GHOST: " + onoff(ghost); }, [this] { ghost = !ghost; }, 1,
                                 [this] { return ghost; }});
                    b.push_back({[this] { return std::string(matching ? "DONE REALIGNING" : "REALIGN VIEW"); },
                                 [this] {
                                     matching = !matching;
                                     matchHand = -1;
                                     drawing = false;
                                     releaseAll();
                                     if (matching) {
                                         menuOpen = false;
                                         say("Hold trigger and drag or twist to line up. Stick click resets");
                                     } else {
                                         say("Match done");
                                     }
                                 },
                                 1, [this] { return matching; }});
                    b.push_back({[this] { return "UNIVERSE " + st.universe(); },
                                 [this] {
                                     if (st.ids().size() < 2) {
                                         say("Only one universe in the chaperone file");
                                         return;
                                     }
                                     selectUniverse(st.nextUniverse());
                                 }});
                    b.push_back({[this] { return std::string("ALIGN: ") + frameModeName(st.mode()); },
                                 [this] { cycleFrameMode(); }});
                    b.push_back({[] { return std::string("RESTORE ORIGINAL"); }, [this] { restoreOriginal(); }});
                    b.push_back({[] { return std::string("SAVE AND EXIT"); }, [this] {
                                     if (dirty || centerDirty || areaDirty) {
                                         if (!save()) {
                                             return;
                                         }
                                     }
                                     quit = true;
                                 }});
                    b.push_back({[] { return std::string("EXIT"); }, [this] {
                                     if ((dirty || centerDirty || areaDirty) && host_.now() > exitArmed) {
                                         exitArmed = host_.now() + 4;
                                         say("Unsaved changes. Press exit again to quit");
                                         return;
                                     }
                                     quit = true;
                                 },
                                 2});
                    return b;
                }};

    pageSpaces = {2, 0.056,
                  [this] {
                      std::string act = lib.active();
                      if (act.empty()) {
                          act = "NONE YET";
                      }
                      return std::vector<std::string>{"SAVED SPACES   ACTIVE: " + upper(act),
                                                      "PICK ONE TO WRITE IT TO STEAMVR. SAVE AS NEW STORES WHAT YOU SEE NOW"};
                  },
                  [this] {
                      std::vector<Button> bs;
                      const std::string act = lib.active();
                      for (const Room& r : lib.list()) {
                          bs.push_back({[r] { return std::format("{} ({})", upper(r.name), r.points.size()); },
                                        [this, r] { useSpace(r); }, 1,
                                        [r, act] { return upper(r.name) == upper(act); }});
                          if (bs.size() >= 12) {
                              break;
                          }
                      }
                      if (bs.size() % 2 == 1) {
                          bs.push_back({[] { return std::string(); }, [] {}});
                      }
                      bs.push_back({[] { return std::string("SAVE AS NEW"); }, [this] {
                                        kbText.clear();
                                        page = &pageKb;
                                    }});
                      bs.push_back({[act] { return act.empty() ? std::string("UPDATE ACTIVE") : "UPDATE " + upper(act); },
                                    [this, act] {
                                        if (act.empty()) {
                                            say("No active space yet. Use save as new");
                                            return;
                                        }
                                        if (auto e = saveAsNew(act); !e) {
                                            say(e.error());
                                            return;
                                        }
                                        say("Updated " + act);
                                    }});
                      bs.push_back({[this, act] {
                                        if (host_.now() < confirmDel) {
                                            return std::string("CONFIRM DELETE");
                                        }
                                        return act.empty() ? std::string("DELETE ACTIVE") : "DELETE " + upper(act);
                                    },
                                    [this, act] {
                                        if (act.empty()) {
                                            return;
                                        }
                                        if (host_.now() > confirmDel) {
                                            confirmDel = host_.now() + 4;
                                            return;
                                        }
                                        confirmDel = 0;
                                        if (auto e = lib.remove(act); !e) {
                                            say(e.error());
                                            return;
                                        }
                                        lib.setActive("");
                                        say("Deleted " + act);
                                    }});
                      bs.push_back({[] { return std::string("BACK"); }, [this] { page = &pageMain; }});
                      return bs;
                  }};

    const auto distLabel = [this] {
        if (host_.zone == nullptr) {
            return std::string("NOT AVAILABLE");
        }
        const ZoneValues v = host_.zone->values();
        if (v.fade) {
            return fmtLen(*v.fade);
        }
        return std::string("STEAMVR DEFAULT");
    };
    const auto nudge = [this](double d) {
        return [this, d] {
            if (host_.zone == nullptr) {
                say("Zone settings are not available");
                return;
            }
            const double v = host_.zone->values().fade.value_or(kSteamVrDefaultFade);
            auto nv = host_.zone->setWarnDistance(v + d);
            if (!nv) {
                say("Couldn't write SteamVR settings: " + nv.error());
                return;
            }
            say(std::format("Warning distance {:.2f} M saved", *nv));
        };
    };
    pageZone = {2, 0.056,
                [this, distLabel] {
                    const bool on = host_.zone == nullptr || host_.zone->values().zoneOn();
                    return std::vector<std::string>{"SAFETY ZONE SETTINGS",
                                                    "WARNING DISTANCE " + distLabel() + "   ZONE " + onoff(on)};
                },
                [this, nudge] {
                    std::vector<Button> b;
                    b.push_back({[] { return std::string("CLOSER -10 CM"); }, nudge(-0.10)});
                    b.push_back({[] { return std::string("FARTHER +10 CM"); }, nudge(0.10)});
                    b.push_back({[] { return std::string("CLOSER -2 CM"); }, nudge(-0.02)});
                    b.push_back({[] { return std::string("FARTHER +2 CM"); }, nudge(0.02)});
                    b.push_back({[] { return std::string("STEAMVR DEFAULT"); }, [this] {
                                     if (host_.zone == nullptr) {
                                         return;
                                     }
                                     if (auto r = host_.zone->setWarnDistance(std::nullopt); !r) {
                                         say("Couldn't write SteamVR settings: " + r.error());
                                         return;
                                     }
                                     say("Warning distance back to SteamVR default");
                                 }});
                    b.push_back({[this] {
                                     return "SAFETY ZONE: " + onoff(host_.zone == nullptr || host_.zone->values().zoneOn());
                                 },
                                 [this] {
                                     if (host_.zone == nullptr) {
                                         return;
                                     }
                                     const bool on = !host_.zone->values().zoneOn();
                                     if (auto r = host_.zone->setZoneOn(on); !r) {
                                         say("Couldn't write SteamVR settings: " + r.error());
                                         return;
                                     }
                                     say("Safety zone " + onoff(on));
                                 },
                                 1, [this] { return host_.zone == nullptr || host_.zone->values().zoneOn(); }});
                    b.push_back({[this] { return "PROXIMITY TEST: " + onoff(proxTest); },
                                 [this] {
                                     proxTest = !proxTest;
                                     if (proxTest) {
                                         say("Walk toward a wall. Drag the orange line to change the distance");
                                     }
                                 },
                                 2, [this] { return proxTest; }});
                    b.push_back({[this] { return "FLOOR FIX WITH SAVE: " + onoff(keepFloor); },
                                 [this] {
                                     keepFloor = !keepFloor;
                                     say(keepFloor ? "The next save also moves SteamVR's floor by the fix"
                                                   : "Floor fix is view only");
                                 },
                                 2, [this] { return keepFloor; }});
                    b.push_back({[] { return std::string("BACK"); }, [this] { page = &pageMain; }, 2});
                    return b;
                }};

    pageArea = {2, 0.056,
                [this] {
                    return std::vector<std::string>{
                        std::format("CENTER & PLAY AREA  {} X {}", fmtLen(area.w), fmtLen(area.h)),
                        "ON THE FLOOR: CENTER MOVES, CURVED ARROWS TURN, BLUE HANDLES STRETCH"};
                },
                [this] {
                    std::vector<Button> b;
                    b.push_back({[] { return std::string("FIT AREA"); }, [this] { fitArea(); }});
                    b.push_back({[] { return std::string("CENTER IN ROOM"); }, [this] { centerInRoom(); }});
                    b.push_back({[] { return std::string("SQUARE TO ROOM"); }, [this] { squareToRoom(); }});
                    b.push_back({[] { return std::string("RESET"); }, [this] { resetArea(); }});
                    b.push_back({[] { return std::string("LOCK PLAY AREA"); }, [this] { setAreaMode(false); }, 2});
                    return b;
                }};

    const auto key = [this](std::string s) {
        return Button{[s] { return s; }, [this, s] {
                          if (kbText.size() < 24) {
                              kbText += s;
                          }
                      }};
    };
    pageKb = {10, 0.05,
              [this] { return std::vector<std::string>{"NAME THIS SPACE", "> " + kbText + "_"}; },
              [this, key] {
                  std::vector<Button> bs;
                  for (const char* row : {"1234567890", "QWERTYUIOP", "ASDFGHJKL-", "ZXCVBNM"}) {
                      for (const char* c = row; *c != 0; c++) {
                          bs.push_back(key(std::string(1, *c)));
                      }
                  }
                  bs.push_back({[] { return std::string("DEL"); }, [this] {
                                    if (!kbText.empty()) {
                                        kbText.pop_back();
                                    }
                                },
                                3});
                  bs.push_back({[] { return std::string("SPACE"); }, [this] {
                                    if (kbText.size() < 24 && !kbText.empty()) {
                                        kbText += ' ';
                                    }
                                },
                                4});
                  bs.push_back({[] { return std::string("CANCEL"); }, [this] { page = &pageSpaces; }, 3});
                  bs.push_back({[] { return std::string("SAVE"); }, [this] {
                                    std::string name = kbText;
                                    while (!name.empty() && name.back() == ' ') {
                                        name.pop_back();
                                    }
                                    if (auto e = saveAsNew(name); !e) {
                                        say(e.error());
                                        return;
                                    }
                                    say("Saved space " + name);
                                    page = &pageSpaces;
                                },
                                3});
                  return bs;
              }};
    page = &pageMain;
}

std::pair<std::vector<Slot>, double> Editor::layout() const {
    const int cols = page->cols;
    const double cellW = (kPanelW - 2 * kPanelPad - (cols - 1) * kBtnGap) / cols;
    std::vector<Slot> slots;
    int col = 0;
    int row = 0;
    for (const Button& b : cur) {
        const int span = std::max(b.span, 1);
        if (col + span > cols) {
            col = 0;
            row++;
        }
        const double w = cellW * span + kBtnGap * (span - 1);
        const double x = -kPanelW / 2 + kPanelPad + col * (cellW + kBtnGap) + w / 2;
        const double y = -kHeaderH - row * (page->keyH + kBtnGap) - page->keyH / 2;
        slots.push_back({x, y, w, page->keyH});
        col += span;
        if (col >= cols) {
            col = 0;
            row++;
        }
    }
    int rows = row;
    if (col > 0) {
        rows++;
    }
    const double h = kHeaderH + rows * (page->keyH + kBtnGap) + kFooterH;
    for (Slot& s : slots) {
        s.cy += h / 2;
    }
    return {slots, h};
}

void Editor::openMenu() {
    const Vec3 hp = hmd.pos();
    Vec3 fwd = hmd.axis(2) * -1;
    fwd.y = 0;
    fwd = fwd.norm();
    if (fwd.length() < 0.5) {
        fwd = {0, 0, -1};
    }
    menuPos = hp + fwd * 0.48 + Vec3{0, -0.22, 0};
    menuN = fwd * -1;
    menuR = kUp.cross(menuN).norm();
    menuU = kUp;
    menuOpen = true;
    page = areaMode ? &pageArea : &pageMain;
}

int Editor::hitButton(Vec3 p) const {
    const auto [slots, h] = layout();
    const Vec3 d = p - menuPos;
    const double lx = d.dot(menuR);
    const double ly = d.dot(menuU);
    const double lz = d.dot(menuN);
    if (lz < -0.06 || lz > 0.10) {
        return -1;
    }
    for (std::size_t i = 0; i < slots.size(); i++) {
        const Slot& s = slots[i];
        if (std::fabs(lx - s.cx) <= s.w / 2 + kBtnGap / 2 && std::fabs(ly - s.cy) <= s.h / 2 + kBtnGap / 2) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::optional<Vec3> Editor::rayPanel(const Hand& h) const {
    if (!menuOpen || !h.aimValid) {
        return std::nullopt;
    }
    const Vec3 o = h.aim.pos();
    const Vec3 dir = h.aim.axis(2) * -1;
    const double den = dir.dot(menuN);
    if (std::fabs(den) < 1e-4) {
        return std::nullopt;
    }
    const double t = (menuPos - o).dot(menuN) / den;
    if (t <= 0 || t > 6) {
        return std::nullopt;
    }
    const Vec3 p = o + dir * t;
    const double hh = layout().second;
    const Vec3 d = p - menuPos;
    if (std::fabs(d.dot(menuR)) > kPanelW / 2 + 0.02 || std::fabs(d.dot(menuU)) > hh / 2 + 0.02) {
        return std::nullopt;
    }
    return p;
}

std::pair<int, int> Editor::nearest(Pt2 c) const {
    int bv = -1;
    double bd = grabR();
    for (std::size_t i = 0; i < pts.size(); i++) {
        if (const double d = dist(pts[i], c); d < bd) {
            bd = d;
            bv = static_cast<int>(i);
        }
    }
    if (bv >= 0) {
        return {bv, -1};
    }
    int be = -1;
    double bde = kEdgeRadius;
    for (std::size_t i = 0; i < pts.size(); i++) {
        if (const double d = segDist(c, pts[i], pts[(i + 1) % pts.size()]).d; d < bde) {
            bde = d;
            be = static_cast<int>(i);
        }
    }
    return {-1, be};
}

Pt2 Editor::snapped(int i, Pt2 c) const {
    if (!snap || pts.size() < 3) {
        return c;
    }
    const auto n = pts.size();
    const auto k = static_cast<std::size_t>(i);
    const Pt2 prev = pts[(k + n - 1) % n];
    const Pt2 next = pts[(k + 1) % n];
    return snapToNeighbors(c, &prev, &next, theta);
}

Pt2 Editor::draftSnap(Pt2 c) const {
    if (!snap || draft.size() < 2) {
        return c;
    }
    const Pt2 last = draft.back();
    const Pt2 first = draft.front();
    const double th = std::atan2(draft[1].z - draft[0].z, draft[1].x - draft[0].x);
    if (draft.size() >= 3) {
        return snapToNeighbors(c, &last, &first, th);
    }
    return snapToNeighbors(c, &last, nullptr, th);
}

void Editor::switchTool(Hand& h) {
    laser = !laser;
    laserHand = h.idx;
    haptic(h.idx, 2500);
    say(laser ? "Pointer mode" : "Floor mode");
}

std::pair<int, double> Editor::rayGen(Vec3 o, Vec3 d) const {
    int best = -1;
    double bt = std::numeric_limits<double>::infinity();
    constexpr double R = 0.13;
    const double a = d.x * d.x + d.z * d.z;
    if (a < 1e-8) {
        return {-1, 0};
    }
    for (std::size_t i = 0; i < pts.size(); i++) {
        const Pt2 p = pts[i];
        const double fx = o.x - p.x;
        const double fz = o.z - p.z;
        const double b = 2 * (fx * d.x + fz * d.z);
        const double c = fx * fx + fz * fz - R * R;
        const double disc = b * b - 4 * a * c;
        if (disc < 0) {
            continue;
        }
        const double sq = std::sqrt(disc);
        for (const double tt : {(-b - sq) / (2 * a), (-b + sq) / (2 * a)}) {
            if (tt <= 0.02 || tt >= bt) {
                continue;
            }
            const double y = o.y + d.y * tt;
            if (y < -0.05 || y > height + 0.2) {
                continue;
            }
            best = static_cast<int>(i);
            bt = tt;
            break;
        }
    }
    return {best, bt};
}

bool Editor::handleRayGen(Hand& h) {
    if (!h.aimValid || pts.size() < 3) {
        h.rayGrab = false;
        return false;
    }
    const Vec3 o = h.aim.pos();
    const Vec3 d = h.aim.axis(2) * -1;
    if (h.rayGrab) {
        if (!h.held(bit::Trigger) || h.grab < 0 || h.grab >= static_cast<int>(pts.size())) {
            h.rayGrab = false;
            h.grab = -1;
            theta = domAngle(pts);
            haptic(h.idx, 800);
            return true;
        }
        const Vec3 raw = o + d * h.rayDist;
        const Pt2 q = h.rf.step({raw.x, raw.z}, 1.0 / 90);
        const auto g = static_cast<std::size_t>(h.grab);
        pts[g] = snapped(h.grab, {q.x + h.grabOff.x, q.z + h.grabOff.z});
        const Pt2 gp = pts[g];
        h.rayHit = {gp.x, std::clamp(raw.y, 0.1, height), gp.z};
        h.rayHitOK = true;
        h.hoverV = h.grab;
        h.hoverE = -1;
        return true;
    }
    const auto [gi, tt] = rayGen(o, d);
    if (gi < 0) {
        return false;
    }
    if (const auto c = h.pointer(true); c && (fl(*c, 0) - o).length() < tt - 0.05) {
        return false;
    }
    const Vec3 hp = o + d * tt;
    if (h.hoverV != gi) {
        haptic(h.idx, 400);
    }
    h.hoverV = gi;
    h.hoverE = -1;
    h.rayHit = hp;
    h.rayHitOK = true;
    if (h.down(bit::Trigger)) {
        for (const Hand& ot : hands) {
            if (ot.idx != h.idx && ot.grab == gi) {
                return true;
            }
        }
        push();
        const Pt2 gp = pts[static_cast<std::size_t>(gi)];
        h.grab = gi;
        h.rayGrab = true;
        h.rayDist = tt;
        h.grabOff = {gp.x - hp.x, gp.z - hp.z};
        h.rf = {};
        h.rf.step({hp.x, hp.z}, 1.0 / 90);
        haptic(h.idx, 1500);
    } else if (h.down(bit::Grip)) {
        if (pts.size() <= 3) {
            say("A boundary needs at least 3 points");
            return true;
        }
        push();
        releaseAll();
        pts.erase(pts.begin() + gi);
        theta = domAngle(pts);
        haptic(h.idx, 2500);
        say("Point deleted");
    } else if (h.down(bit::Stick)) {
        doUndo();
    }
    return true;
}

std::vector<std::pair<int, Pt2>> Editor::areaHandles() {
    const Rigid2 g = centerPose();
    const double w = area.w / 2;
    const double h = area.h / 2;
    const std::vector<std::pair<int, Pt2>> loc{{1, {0, 0}},  {2, {w, 0}},   {3, {-w, 0}}, {4, {0, h}},
                                               {5, {0, -h}}, {6, {w, h}},   {7, {w, -h}}, {8, {-w, h}},
                                               {9, {-w, -h}}, {10, {0, -kArcR}}};
    std::vector<std::pair<int, Pt2>> out;
    for (const auto& [k, v] : loc) {
        out.emplace_back(k, g.apply(v));
    }
    return out;
}

bool Editor::onArc(Pt2 c) {
    const Pt2 l = centerPose().inv().apply(c);
    const double r = std::hypot(l.x, l.z);
    if (std::fabs(r - kArcR) > 0.12) {
        return false;
    }
    const double a = std::atan2(l.x, -l.z);
    return std::fabs(a) < kArcSpan / 2 + 0.1 || std::fabs(remainder2pi(a - kPi)) < kArcSpan / 2 + 0.1;
}

Pt2 Editor::lockPos() { return centerPose().apply({0.35, 0.3}); }

void Editor::setAreaMode(bool on) {
    areaMode = on;
    matching = drawing = false;
    releaseAll();
    for (Hand& h : hands) {
        h.areaGrab = h.areaHover = 0;
    }
    if (on) {
        say("Play area unlocked. Drag the handles on the floor");
        if (menuOpen) {
            page = &pageArea;
        }
        return;
    }
    if (page == &pageArea) {
        page = &pageMain;
    }
    if (centerDirty || areaDirty) {
        dirty = true;
        say("Play area locked. Press save to keep it");
    } else {
        say("Play area locked");
    }
}

int Editor::pickHandle(Pt2 c) {
    const auto hs = areaHandles();
    const auto at = [&](int k) {
        for (const auto& [kk, p] : hs) {
            if (kk == k) {
                return p;
            }
        }
        return Pt2{};
    };
    if (onArc(c)) {
        for (const int k : {6, 7, 8, 9, 2, 3, 4, 5}) {
            if (dist(at(k), c) < 0.18) {
                return k;
            }
        }
        return 10;
    }
    int best = 0;
    double bd = 0;
    for (const int k : {6, 7, 8, 9, 2, 3, 4, 5, 1}) {
        const double r = k == 1 ? 0.25 : 0.18;
        if (const double d = dist(at(k), c); d < r && (best == 0 || d < bd)) {
            best = k;
            bd = d;
        }
    }
    return best;
}

void Editor::turnArea(double rad) {
    const Rigid2 g = centerPose();
    center = rigidFromYaw(std::atan2(g.c, g.a) + rad, g.tx, g.tz);
    centerDirty = true;
}

void Editor::handleArea(Hand& h, Pt2 c) {
    if (h.areaGrab != 0) {
        if (!h.held(bit::Trigger)) {
            h.areaGrab = 0;
            haptic(h.idx, 800);
            return;
        }
        const Rigid2 g0 = center0;
        const Pt2 l = g0.inv().apply(c);
        const double w0 = area0.w;
        const double h0 = area0.h;
        const auto moveAxis = [](double val, double half, double sign) {
            const double fixed = -sign * half;
            const double moved = fixed + sign * std::max(0.5, std::min(20.0, sign * (val - fixed)));
            return std::pair{std::fabs(moved - fixed), (moved + fixed) / 2};
        };
        const int k = h.areaGrab;
        if (k == 1) {
            center = {g0.a, g0.b, g0.c, g0.d, c.x - h.grabOff.x, c.z - h.grabOff.z};
            centerDirty = true;
            return;
        }
        if (k == 10) {
            const double a0 = std::atan2(h.grabOff.z, h.grabOff.x);
            const double a1 = std::atan2(c.z - g0.tz, c.x - g0.tx);
            center = rigidFromYaw(std::atan2(g0.c, g0.a) + a1 - a0, g0.tx, g0.tz);
            centerDirty = true;
            return;
        }
        double cx = 0;
        double cz = 0;
        double w = w0;
        double hh = h0;
        if (k == 2 || k == 6 || k == 7) {
            std::tie(w, cx) = moveAxis(l.x, w0 / 2, 1);
        } else if (k == 3 || k == 8 || k == 9) {
            std::tie(w, cx) = moveAxis(l.x, w0 / 2, -1);
        }
        if (k == 4 || k == 6 || k == 8) {
            std::tie(hh, cz) = moveAxis(l.z, h0 / 2, 1);
        } else if (k == 5 || k == 7 || k == 9) {
            std::tie(hh, cz) = moveAxis(l.z, h0 / 2, -1);
        }
        const Pt2 np = g0.apply({cx, cz});
        center = {g0.a, g0.b, g0.c, g0.d, np.x, np.z};
        area = {w, hh};
        centerDirty = areaDirty = true;
        return;
    }
    h.areaHover = pickHandle(c);
    if (!h.down(bit::Trigger) || h.areaHover == 0) {
        return;
    }
    h.areaGrab = h.areaHover;
    center0 = centerPose();
    area0 = area;
    h.grabOff = {c.x - center0.tx, c.z - center0.tz};
    haptic(h.idx, 1200);
}

double Editor::warnDist(bool* known) const {
    if (host_.zone != nullptr) {
        if (const auto f = host_.zone->values().fade) {
            if (known != nullptr) {
                *known = true;
            }
            return *f;
        }
    }
    if (known != nullptr) {
        *known = false;
    }
    return kAssumedWarnDist;
}

double Editor::proximityLevel() const {
    if (!proxTest || pts.size() < 3) {
        return 0;
    }
    const double d = warnDist();
    double lvl = 0;
    const auto probe = [&](Vec3 p) {
        const double ds = distToBoundary(pts, flat(p));
        if (ds < d) {
            lvl = std::max(lvl, 1 - ds / d);
        }
    };
    probe(hmd.pos());
    for (const Hand& h : hands) {
        if (h.valid) {
            probe(h.m.pos());
        }
    }
    return lvl;
}

bool Editor::handleWarnLine(Hand& h, Pt2 c) {
    if (!proxTest) {
        return false;
    }
    if (h.lineGrab) {
        if (!h.held(bit::Trigger)) {
            h.lineGrab = false;
            const double d = std::clamp(distToBoundary(pts, c), kWarnDistMin, kWarnDistMax);
            if (host_.zone == nullptr) {
                say("Zone settings are not available");
            } else if (auto nv = host_.zone->setWarnDistance(d); !nv) {
                say("Couldn't write SteamVR settings: " + nv.error());
            } else {
                say(std::format("Warning distance {:.2f} M saved", *nv));
            }
            haptic(h.idx, 1500);
            return true;
        }
        lineDrag = std::clamp(distToBoundary(pts, c), kWarnDistMin, kWarnDistMax);
        return true;
    }
    if (h.hoverV >= 0 || h.hoverE >= 0 || !h.down(bit::Trigger)) {
        return false;
    }
    const double d = warnDist();
    if (std::fabs(distToBoundary(pts, c) - d) < 0.12) {
        h.lineGrab = true;
        lineDrag = d;
        haptic(h.idx, 1200);
        return true;
    }
    return false;
}

double Editor::currentWarnLine() const {
    for (const Hand& h : hands) {
        if (h.lineGrab) {
            return lineDrag;
        }
    }
    return warnDist();
}

bool Editor::changed() const { return orig != pts; }

void Editor::handleDrawing(Hand& h, Pt2 c) {
    h.hoverV = h.hoverE = -1;
    if (h.down(bit::Trigger)) {
        const Pt2 p = draftSnap(c);
        if (draft.size() >= 3 && dist(p, draft.front()) < kCloseRadius) {
            drawing = false;
            const auto n = draft.size();
            setPts(draft, std::format("Outline closed with {} corners", n));
            draft.clear();
            haptic(h.idx, 3000);
            return;
        }
        draft.push_back(p);
        draftHand = h.idx;
        haptic(h.idx, 1200);
        say(std::format("Corner {} placed", draft.size()));
    }
    if (h.down(bit::Grip) && !draft.empty()) {
        draft.pop_back();
        say("Corner removed");
    }
    if (h.down(bit::Stick)) {
        drawing = false;
        draft.clear();
        say("Outline cancelled");
    }
}

void Editor::handleTrace(Hand& h, Pt2 c) {
    h.hoverV = h.hoverE = -1;
    if (h.down(bit::Stick)) {
        tracing = false;
        trace = {};
        say("Trace cancelled");
        return;
    }
    if (h.down(bit::Trigger) && !trace.running()) {
        trace.start();
        traceHand = h.idx;
        haptic(h.idx, 1200);
        say("Tracing. Walk along the walls, let go at the start");
    }
    if (trace.running() && traceHand == h.idx) {
        if (h.held(bit::Trigger)) {
            trace.add(c);
            return;
        }
        Poly p = trace.finish();
        tracing = false;
        if (p.empty()) {
            say("The trace didn't close into a room. Try again");
            return;
        }
        const auto n = p.size();
        setPts(std::move(p), std::format("Traced outline with {} corners", n));
        haptic(h.idx, 3000);
    }
}

void Editor::handleHand(Hand& h) {
    auto c = h.pointer(laser);
    h.btn = -1;
    h.menuRay = false;
    if (menuOpen) {
        if (laser && h.idx == laserHand) {
            if (const auto p = rayPanel(h)) {
                h.menuRay = true;
                h.menuPt = *p;
                h.btn = hitButton(*p);
            }
        }
        if (h.btn < 0) {
            h.btn = hitButton(h.tip());
        }
    }
    if (h.down(bit::Menu)) {
        if (menuOpen) {
            menuOpen = false;
        } else {
            openMenu();
        }
        return;
    }
    if (h.down(bit::Tool)) {
        switchTool(h);
        return;
    }
    if (h.btn >= 0 && h.down(bit::Trigger)) {
        haptic(h.idx, 1500);
        const auto i = static_cast<std::size_t>(h.btn);
        if (i < cur.size() && cur[i].act) {
            cur[i].act();
        }
        return;
    }
    if (menuOpen && (h.btn >= 0 || h.menuRay)) {
        return;
    }
    if (h.idx != laserHand) {
        const Hand& lh = hands[static_cast<std::size_t>(laserHand)];
        const bool busy = lh.grab >= 0 || lh.areaGrab != 0 || (trace.running() && traceHand == laserHand);
        if (h.down(bit::Trigger) && !busy) {
            laserHand = h.idx;
            haptic(h.idx, 600);
            return;
        }
        c.reset();
    }
    if (matching) {
        handleMatch(h);
        return;
    }
    h.rayHitOK = false;
    if (laser && h.idx == laserHand && !areaMode && !drawing && !tracing && (h.rayGrab || h.grab < 0)) {
        if (handleRayGen(h)) {
            return;
        }
    }
    if (!c) {
        h.hoverV = h.hoverE = -1;
        return;
    }
    if (h.down(bit::Trigger) && h.areaGrab == 0 && !tracing && dist(*c, lockPos()) < 0.15) {
        setAreaMode(!areaMode);
        haptic(h.idx, 2000);
        return;
    }
    if (areaMode) {
        h.hoverV = h.hoverE = -1;
        handleArea(h, *c);
        return;
    }
    if (drawing) {
        handleDrawing(h, *c);
        return;
    }
    if (tracing) {
        handleTrace(h, *c);
        return;
    }
    if (h.grab >= 0) {
        if (!h.held(bit::Trigger) || h.grab >= static_cast<int>(pts.size())) {
            h.grab = -1;
            theta = domAngle(pts);
            haptic(h.idx, 800);
        } else {
            pts[static_cast<std::size_t>(h.grab)] = snapped(h.grab, {c->x + h.grabOff.x, c->z + h.grabOff.z});
        }
        return;
    }
    std::tie(h.hoverV, h.hoverE) = nearest(*c);
    if (handleWarnLine(h, *c)) {
        return;
    }
    if (h.down(bit::Trigger)) {
        bool taken = false;
        for (const Hand& o : hands) {
            if (o.idx != h.idx && o.grab == h.hoverV && h.hoverV >= 0) {
                taken = true;
            }
        }
        if (h.hoverV >= 0 && !taken) {
            push();
            h.grab = h.hoverV;
            const Pt2 gp = pts[static_cast<std::size_t>(h.grab)];
            h.grabOff = {gp.x - c->x, gp.z - c->z};
            haptic(h.idx, 1200);
        } else if (h.hoverE >= 0) {
            push();
            releaseAll();
            const auto e = static_cast<std::size_t>(h.hoverE);
            const Pt2 a = pts[e];
            const Pt2 b = pts[(e + 1) % pts.size()];
            const double t = segDist(*c, a, b).t;
            const Pt2 np{a.x + (b.x - a.x) * t, a.z + (b.z - a.z) * t};
            pts.insert(pts.begin() + static_cast<long>(e) + 1, np);
            h.grab = h.hoverE + 1;
            h.grabOff = {np.x - c->x, np.z - c->z};
            haptic(h.idx, 1800);
        }
        return;
    }
    if (h.down(bit::Grip) && h.hoverV >= 0) {
        if (pts.size() <= 3) {
            say("A boundary needs at least 3 points");
            return;
        }
        push();
        releaseAll();
        pts.erase(pts.begin() + h.hoverV);
        theta = domAngle(pts);
        haptic(h.idx, 2500);
        say("Point deleted");
        return;
    }
    if (h.down(bit::Stick)) {
        doUndo();
    }
}

void Editor::updateFloor() {
    if (!floorWarned) {
        for (const Hand& h : hands) {
            if (h.valid && h.m.pos().y < -0.15) {
                floorWarned = true;
                say("Floor looks wrong. Menu > Fix floor");
            }
        }
    }
    if (floorAt == 0 || host_.now() < floorAt) {
        return;
    }
    floorAt = 0;
    double minY = std::numeric_limits<double>::infinity();
    for (const Hand& h : hands) {
        if (h.valid) {
            minY = std::min(minY, h.m.pos().y);
        }
    }
    if (std::isinf(minY)) {
        say("No controller tracked, floor unchanged");
        return;
    }
    floorY += minY - 0.04;
    say(std::format("Floor moved {:+.0f} CM ({})", (minY - 0.04) * 100, keepFloor ? "saved with the boundary" : "view only"));
}

void Editor::applyStageMove(const Rigid2& r) {
    center = centerPose().then(r);
    st.moveStage(r);
    pts = r.all(pts);
    orig = r.all(orig);
    draft = r.all(draft);
    for (Poly& u : undo) {
        u = r.all(u);
    }
    for (Poly& u : redo) {
        u = r.all(u);
    }
    releaseAll();
    matchHand = -1;
    menuOpen = false;
}

void Editor::stageMoved(const Rigid2& oldFromNew) {
    applyStageMove(oldFromNew.inv());
    say("SteamVR recentered. Boundary kept in place");
}

void Editor::handleMatch(Hand& h) {
    if (h.down(bit::Stick)) {
        applyStageMove(st.adjust().inv());
        say("Match reset");
        return;
    }
    if (matchHand < 0 && h.down(bit::Trigger)) {
        matchHand = h.idx;
        h.matchG0 = handPose2(h.m);
        matchAdj0 = st.adjust();
        matchPts0 = pts;
        matchOrig0 = orig;
        haptic(h.idx, 1200);
        return;
    }
    if (matchHand != h.idx) {
        return;
    }
    if (!h.held(bit::Trigger)) {
        matchHand = -1;
        haptic(h.idx, 800);
        return;
    }
    const Rigid2 delta = h.matchG0.inv().then(handPose2(h.m));
    st.setAdj(matchAdj0.then(delta));
    pts = delta.all(matchPts0);
    orig = delta.all(matchOrig0);
}

void Editor::frame(const FrameInput& in) {
    if (in.headValid) {
        hmd = in.head;
        hmd.m[1][3] -= floorY;
    }
    for (std::size_t i = 0; i < 2; i++) {
        Hand& h = hands[i];
        const HandInput& hi = in.hands[i];
        h.valid = hi.valid;
        h.m = hi.grip;
        h.aim = hi.aim;
        h.aimValid = hi.aimValid;
        h.m.m[1][3] -= floorY;
        h.aim.m[1][3] -= floorY;
        h.updateLaser(in.dt);
        h.prev = h.pressed;
        h.pressed = hi.buttons;
    }
    cur = page->buttons();
    for (Hand& h : hands) {
        if (h.valid) {
            handleHand(h);
            cur = page->buttons(); // a button may have changed the page
        }
    }
    updateFloor();
    cur = page->buttons();
}

std::string tipFor(const std::string& label) {
    static const std::vector<std::pair<const char*, const char*>> tips{
        {"SAVE AND EXIT", "Saves everything and closes the room setup. SteamVR is asked to reload the boundary."},
        {"SAVE AS NEW", "Name and store the outline you see now as a saved space, like DOOR OPEN or DOOR CLOSED."},
        {"SAVED SPACES", "Store named boundaries (door open, door closed...) and switch between them with one press."},
        {"SAVE", "Writes the boundary, play center and play area to SteamVR's chaperone file and asks SteamVR to reload it."},
        {"REVERT", "Throws away unsaved edits and goes back to the last saved boundary."},
        {"UNDO", "Steps back one edit. Thumbstick click also undoes."},
        {"REDO", "Re-applies the edit you just undid."},
        {"SQUARE TO ROOM", "Turns the play area so its edges run parallel to your walls."},
        {"SQUARE UP", "Turns the traced outline into straight walls at right angles, lined up with your room."},
        {"SIMPLIFY", "Removes extra points that sit on an almost straight line (within 8 cm). The shape stays the same."},
        {"NEW OUTLINE", "Start over: walk to each corner of the room and pull the trigger. Pull it near the first corner to finish."},
        {"CANCEL OUTLINE", "Stops drawing the new outline and keeps the old one."},
        {"TRACE OUTLINE", "Start over by walking: hold the trigger and carry the controller along the walls, let go where you started."},
        {"CANCEL TRACE", "Stops tracing and keeps the old outline."},
        {"SNAP", "ON: dragged points pull square to their neighbors. OFF: free movement for fine adjustments."},
        {"ZONE SETTINGS", "How close you can get before SteamVR's boundary appears, a live proximity test, and the boundary on/off switch."},
        {"FIX FLOOR", "If the grid isn't on your real floor: rest a controller on the floor during the countdown. View only unless FLOOR FIX WITH SAVE is on."},
        {"POINTER", "FLOOR: beam straight down; grab a corner within arm's reach without touching it. LASER: aim from a distance. Press the trackpad (Index) or squeeze (Touch) to switch."},
        {"UNITS", "Show lengths in meters or feet."},
        {"GHOST", "Shows the last saved outline as a grey line while you edit, so you can compare."},
        {"REALIGN VIEW", "If the walls don't line up with your room, hold the trigger and drag or twist the outline into place. Stick click resets. Doesn't change what's saved."},
        {"DONE REALIGNING", "Finish realigning the view."},
        {"UNIVERSE", "SteamVR keeps one boundary per tracking universe. This switches which one you edit."},
        {"ALIGN", "How the file's numbers sit in the room: OFF (SteamVR's own), A or B (Pimax's raw frame). Pick the one where the walls match SteamVR's."},
        {"RESTORE ORIGINAL", "Loads the boundary from the oldest backup, from before anything here changed it. Save to keep it."},
        {"EXIT", "Close the room setup. Asks first if you have unsaved changes."},
        {"CLOSER", "Warning distance: how close you can get to a wall before SteamVR shows the boundary. Smaller = closer."},
        {"FARTHER", "Warning distance: how close you can get to a wall before SteamVR shows the boundary. Bigger = earlier warning."},
        {"STEAMVR DEFAULT", "Removes the warning distance so SteamVR goes back to its own built-in value."},
        {"SAFETY ZONE", "Shows or hides SteamVR's boundary everywhere. OFF means no warning at all near walls."},
        {"PROXIMITY TEST", "Shows the warning distance as an orange line inside the walls. Walls glow red when your head or hands cross it. Drag the line to change the distance."},
        {"FLOOR FIX WITH SAVE", "ON: the next save also moves SteamVR's floor by what FIX FLOOR measured."},
        {"UPDATE", "Overwrites the active saved space with the outline you see now."},
        {"DELETE", "Deletes the active saved space. Press twice to confirm."},
        {"CONFIRM DELETE", "Press again to delete for good."},
        {"BACK", "Back to the main menu."},
        {"FIT AREA", "Grows the play area to the biggest rectangle that fits inside your walls, with a 10 cm margin. Games use this rectangle for room-scale."},
        {"CENTER IN ROOM", "Moves the play center (where games put you) to the middle of the room."},
        {"RESET", "Puts the play center and play area back to what's saved."},
        {"LOCK PLAY AREA", "Locks the play area so its floor handles can't be grabbed by accident. Same as clicking the padlock on the floor."},
    };
    if (label.ends_with(")") && label.find(" (") != std::string::npos) {
        return "Loads this saved space and writes it to SteamVR. The number is how many corners it has.";
    }
    for (const auto& [prefix, tip] : tips) {
        if (label.starts_with(prefix)) {
            return tip;
        }
    }
    return {};
}

} // namespace pibridge::room
