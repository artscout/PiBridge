// SPDX-License-Identifier: MIT
// Ported from ZoneTweak main.go, area.go, features.go and tool.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
//
// The in-headset boundary editor without any graphics or runtime: it takes the head and controller poses and the
// buttons of one frame (stage space, metres) and changes the boundary, the play centre and area, the menu, the
// saved rooms and the zone settings. pibridge-roomsetup feeds it from OpenXR; the tests and --selftest feed it scripted
// frames. The behaviour is ZoneTweak's (see docs/ROOM-SETUP.md for the controls):
//   trigger near a corner post: grab and move it; near a wall: insert a corner there and move it;
//   A (grip button bit): delete the corner; thumbstick click: undo; B: menu; trackpad press: floor beam <-> laser;
//   the padlock on the floor unlocks the play area: centre post moves, the arcs turn, the edge/corner posts stretch;
//   menu: SAVE, REVERT, UNDO, REDO, SQUARE UP, SIMPLIFY, NEW OUTLINE, SNAP, SAVED SPACES (rooms), ZONE SETTINGS
//   (warning distance, zone on/off, proximity test), FIX FLOOR, POINTER, UNITS, GHOST, REALIGN VIEW, RESTORE ORIGINAL,
//   EXIT. PiBridge adds TRACE OUTLINE (hold the trigger and walk along the walls), UNIVERSE (switch the edited universe),
//   ALIGN (the file's frame mode, chap.h) and SAVE AND EXIT. Left out: ZoneTweak's set dressing (generators, robots,
//   fire, sound), its graphics page and the Pimax service restart.
#pragma once

#include "pibridge/room/chap.h"
#include "pibridge/room/filter.h"
#include "pibridge/room/geom.h"
#include "pibridge/room/pose.h"
#include "pibridge/room/rooms.h"
#include "pibridge/room/vmath.h"
#include "pibridge/room/zone.h"

#include <array>
#include <cstdint>
#include <ctime>
#include <expected>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::room {

/// Button bits (ZoneTweak's, from OpenVR's button ids).
namespace bit {
inline constexpr unsigned Menu = 1;    // B (Index), menu (Vive), B/Y (Touch)
inline constexpr unsigned Grip = 2;    // A: delete (Index), squeeze (Vive), A/X (Touch)
inline constexpr unsigned Tool = 3;    // trackpad press (Index), squeeze (Touch)
inline constexpr unsigned Stick = 32;  // thumbstick click: undo
inline constexpr unsigned Trigger = 33;
} // namespace bit

inline constexpr double kGrabRadius = 0.20;
inline constexpr double kEdgeRadius = 0.15;
inline constexpr double kCloseRadius = 0.25;
inline constexpr double kAssumedWarnDist = kSteamVrDefaultFade;
inline constexpr double kArcR = 0.55;
inline constexpr double kArcSpan = 0.9;

// menu panel (metres)
inline constexpr double kPanelW = 0.56;
inline constexpr double kBtnGap = 0.010;
inline constexpr double kHeaderH = 0.085;
inline constexpr double kFooterH = 0.045;
inline constexpr double kTextPx = 0.0027;
inline constexpr double kSmallPx = 0.0021;
inline constexpr double kPanelPad = 0.02;

/// Warning distance and zone switch (zone.h): the file while SteamVR is off, IVRSettings while it runs.
class ZoneControl {
public:
    virtual ~ZoneControl() = default;
    virtual ZoneValues values() = 0;
    virtual std::expected<double, std::string> setWarnDistance(std::optional<double> metres) = 0;
    virtual std::expected<void, std::string> setZoneOn(bool on) = 0;
};

/// steamvr.vrsettings on disk (tests, and the tool when SteamVR is not the runtime).
class FileZoneControl : public ZoneControl {
public:
    FileZoneControl(std::string settingsPath, std::string stateDir, std::function<std::time_t()> clock);
    ZoneValues values() override;
    std::expected<double, std::string> setWarnDistance(std::optional<double> metres) override;
    std::expected<void, std::string> setZoneOn(bool on) override;

private:
    VrSettingsFile file_;
    std::string stateDir_;
    std::function<std::time_t()> clock_;
    // the menu asks every frame: the file is read at most once a second (ZoneTweak's profile cache)
    std::optional<ZoneValues> cache_;
    double cachedAt_ = 0;
};

struct HandInput {
    bool valid = false;
    Mat34 grip;
    bool aimValid = false;
    Mat34 aim;
    std::uint64_t buttons = 0; // 1 << bit::...
    bool tracked = true;       // the runtime tracks the position now (not a held last pose)
    float stickY = 0.0F;       // thumbstick / trackpad Y, +1 up .. -1 down (floor height nudge on the floor step)
    Mat34 gripPose;            // the handle (grip) pose, kept even when `grip` gets the aim pose: the anchor for
    bool gripPoseValid = false; // drawing a controller render model (room setup)
};

struct FrameInput {
    double dt = 1.0 / 90;
    bool headValid = false;
    Mat34 head;
    std::array<HandInput, 2> hands;
};

struct Hand {
    int idx = 0;
    bool valid = false;
    Mat34 m;
    std::uint64_t pressed = 0;
    std::uint64_t prev = 0;
    int grab = -1;
    int hoverV = -1;
    int hoverE = -1;
    int btn = -1;
    int prevBtn = -1;
    Mat34 aim;
    bool aimValid = false;
    bool lineGrab = false;
    int areaGrab = 0;
    int areaHover = 0;
    Pt2 grabOff;
    bool menuRay = false;
    Vec3 menuPt;
    PointFilter lf;
    bool rayGrab = false;
    double rayDist = 0;
    PointFilter rf;
    Vec3 rayHit;
    bool rayHitOK = false;
    Rigid2 matchG0;
    Pt2 lc;
    bool lok = false;

    bool down(unsigned b) const { return (pressed & (1ULL << b)) != 0 && (prev & (1ULL << b)) == 0; }
    bool held(unsigned b) const { return (pressed & (1ULL << b)) != 0; }
    Pt2 cursor() const { return flat(m.pos()); }
    Vec3 tip() const { return m.xform({0, 0, -0.09}); }
    /// The laser's floor point (unfiltered): the aim ray down to y = 0, at most 15 m away.
    std::optional<Pt2> rawLaser() const;
    void updateLaser(double dt);
    /// FLOOR mode: the controller straight down; LASER: the filtered laser floor point.
    std::optional<Pt2> pointer(bool laser) const;
};

struct Button {
    Button(std::function<std::string()> l, std::function<void()> a, int s = 1, std::function<bool()> o = {})
        : label(std::move(l)), act(std::move(a)), span(s), on(std::move(o)) {}
    std::function<std::string()> label;
    std::function<void()> act;
    int span = 1;
    std::function<bool()> on;
};

struct Page {
    int cols = 2;
    double keyH = 0.056;
    std::function<std::vector<std::string>()> header;
    std::function<std::vector<Button>()> buttons;
};

struct Slot {
    double cx, cy, w, h;
};

/// What the editor needs from outside.
struct EditorHost {
    std::function<double()> now;              // monotonic seconds
    std::function<std::time_t()> wallClock;   // names the backups and rooms
    std::function<void(int hand, int ms, float amp)> haptic;
    /// After a successful save of the chaperone (SteamVR: reload it from disk). Its words are added to the status.
    std::function<std::string(const SaveResult&)> afterSave;
    ZoneControl* zone = nullptr;              // nullptr: zone settings not available
};

class Editor {
public:
    Editor(Store& store, RoomLib lib, EditorHost host);
    Editor(const Editor&) = delete; // the menu pages point back at it
    Editor& operator=(const Editor&) = delete;

    void loadFromStore();
    /// One frame: poses and buttons in, edits out.
    void frame(const FrameInput& in);
    /// The stage moved under us (the runtime recentred): keep the boundary where it is in the room.
    /// `oldFromNew` maps new stage coordinates to old ones.
    void stageMoved(const Rigid2& oldFromNew);

    // actions (menu buttons call these; tests too)
    bool save();
    void say(const std::string& s);
    void setPts(Poly p, const std::string& msg);
    void doUndo();
    void doRedo();
    /// Load a saved room and write it (ZoneTweak: tapping a space loads it AND writes it).
    void useSpace(const Room& r);
    /// Load a saved room into the editor only (unsaved).
    void loadSpace(const Room& r);
    /// Turn the play centre (its forward direction) by `rad`.
    void turnArea(double rad);
    /// Remember the boundary for undo (before a change made from outside, such as the app's floor plan).
    void push();
    void setAreaMode(bool on);
    void openMenu();
    void fitArea();
    void centerInRoom();
    void squareToRoom();
    void resetArea();
    void restoreOriginal();
    void selectUniverse(const std::string& id);
    void cycleFrameMode();
    std::expected<void, std::string> saveAsNew(const std::string& name);

    // queries
    Rigid2 centerPose();
    std::array<Pt2, 4> areaCorners(const Rigid2& g, double w, double h) const;
    std::vector<std::pair<int, Pt2>> areaHandles();
    Pt2 lockPos();
    bool onArc(Pt2 c);
    int pickHandle(Pt2 c);
    std::pair<std::vector<Slot>, double> layout() const;
    int hitButton(Vec3 p) const;
    std::optional<Vec3> rayPanel(const Hand& h) const;
    double warnDist(bool* known = nullptr) const;
    double currentWarnLine() const;
    double proximityLevel() const;
    bool changed() const;
    std::string fmtLen(double m) const;
    std::string fmtArea(double a) const;
    double grabR() const { return laser ? kGrabRadius : 0.45; }
    const std::string& status() const { return status_; }
    double statusAge() const { return host_.now() - statusAt_; }

    // state (read by the scene builder)
    Store& st;
    RoomLib lib;
    std::vector<Hand> hands;
    Mat34 hmd;
    Poly pts;
    Poly orig;
    double height = kDefaultHeight;
    double theta = 0;
    std::vector<Poly> undo;
    std::vector<Poly> redo;
    bool snap = false;
    bool ghost = true;
    bool feet = false;
    bool drawing = false;
    Poly draft;
    int draftHand = -1;
    bool tracing = false;   // TRACE OUTLINE armed
    TraceRecorder trace;
    int traceHand = -1;
    bool menuOpen = false;
    Vec3 menuPos, menuR, menuU, menuN;
    Page* page = nullptr;
    std::vector<Button> cur;
    std::string kbText;
    bool proxTest = false;
    bool laser = false;
    int laserHand = 1;
    double lineDrag = 0;
    bool matching = false;
    int matchHand = -1;
    Rigid2 matchAdj0;
    Poly matchPts0, matchOrig0;
    Rigid2 center, center0;
    bool centerSet = false;
    bool centerDirty = false;
    bool areaDirty = false;
    bool areaMode = false;
    Size2 area{1, 1};
    Size2 area0{1, 1};
    double floorY = 0;
    double floorAt = 0;      // countdown end (0 = none)
    bool floorWarned = false;
    bool keepFloor = false;  // write the floor fix with the next save
    bool savedAny = false;
    bool dirty = false;
    bool quit = false;
    double exitArmed = 0;
    double confirmDel = 0;
    Page pageMain, pageSpaces, pageZone, pageArea, pageKb;

private:
    void buildPages();
    void releaseAll();
    void handleHand(Hand& h);
    void handleArea(Hand& h, Pt2 c);
    bool handleWarnLine(Hand& h, Pt2 c);
    bool handleRayGen(Hand& h);
    void handleMatch(Hand& h);
    void handleDrawing(Hand& h, Pt2 c);
    void handleTrace(Hand& h, Pt2 c);
    void updateFloor();
    void applyStageMove(const Rigid2& r);
    void switchTool(Hand& h);
    std::pair<int, int> nearest(Pt2 c) const;
    Pt2 snapped(int i, Pt2 c) const;
    Pt2 draftSnap(Pt2 c) const;
    std::pair<int, double> rayGen(Vec3 o, Vec3 d) const;
    void haptic(int hand, int us);

    EditorHost host_;
    std::string status_;
    double statusAt_ = -100;
};

/// Hand pose as a floor rigid (position and the -z direction projected), for REALIGN VIEW.
Rigid2 handPose2(const Mat34& m);
/// The rigid of a stage pose (OpenXR's poseInPreviousSpace of a reference space change).
Rigid2 rigidOfPose(const Mat34& m);
/// The ZoneTweak tooltip for a button label ("" when none).
std::string tipFor(const std::string& label);

} // namespace pibridge::room
