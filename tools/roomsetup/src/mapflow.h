// SPDX-License-Identifier: MIT
// pibridge-roomsetup --map: the room mapping in the headset over the passthrough (docs/ROOM-SETUP.md "Map the room in
// the headset"), as Pimax's and Meta's room setups do it: look around until the room is mapped (coverage ring and
// the mapping numbers), confirm the floor shown as a grid on the real floor (trigger = yes, or touch the floor with
// a controller), keep the boundary found around you or draw it on the floor with the controller's ray, then look
// forward and pull the trigger. `pibridge-roommap setup fast --guide DIR` does the work and the files
// (pibridge/roommap/guide_ipc.h); this is the headset side, pure (tested with scripted frames). The OpenXR side is
// runMapXr (xrapp.h).
//
// Frames: the tool draws in OpenXR's stage. pibridge-roommap speaks the map's room frame, room = conv(raw), raw being
// Monado's raw SLAM frame (the head ring). StageLink finds stage_from_raw (a yaw and a shift: Monado's offset and,
// under SteamVR, its standing frame) from the same head pose seen in both. Nothing assumes a right world scale: the
// floor, the boundary and forward go through the same frames as the map.
#pragma once

#include "panelpaint.h"

#include "pibridge/room/draw.h"
#include "pibridge/room/editor.h"
#include "pibridge/roommap/guide_ipc.h"

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace pibridge::roomsetup {

namespace guide = roommap::guide;

/// The heading (rotation about +y turning -Z into it) of a pose's forward (-Z) direction; the x axis when looking
/// straight up or down.
double headingOf(const room::Mat34& m);

/// stage_from_raw from pairs of one head pose in both frames (yaw + shift; the window mean, restarted when the
/// offset jumps: 10 pairs in a row more than 10 cm or 4 deg away).
class StageLink {
public:
    void add(const room::Mat34& stageHead, const room::Mat34& rawHead);
    bool valid() const { return n_ >= kMinPairs; }
    double yaw() const { return yaw_; }
    room::Vec3 shift() const { return t_; }
    room::Vec3 toStage(room::Vec3 raw) const;
    room::Vec3 toRaw(room::Vec3 stage) const;
    void reset();

    static constexpr std::size_t kMinPairs = 8;
    static constexpr std::size_t kWindow = 60;

private:
    struct Pair {
        double yaw;
        room::Vec3 t;
    };
    void estimate();
    std::deque<Pair> win_;
    std::vector<Pair> jump_;
    std::size_t n_ = 0;
    double yaw_ = 0;
    room::Vec3 t_{};
};

/// Where the head has looked during the mapping: 12 sectors of 30 deg around, three bands (down, level, up).
class Coverage {
public:
    static constexpr int kSectors = 12;
    void look(const room::Mat34& head);
    bool seen(int band, int sector) const { return seen_[static_cast<std::size_t>(band)][static_cast<std::size_t>(sector)]; }
    double fraction() const;
    /// What to look at next relative to `heading`: "LOOK LEFT", "LOOK RIGHT", "LOOK BEHIND YOU", "LOOK DOWN AT THE
    /// FLOOR", "LOOK UP", or empty when everything has been seen.
    std::string hint(double heading) const;
    static double sectorHeading(int s);

private:
    std::array<std::array<bool, kSectors>, 3> seen_{};
};

class MapFlow {
public:
    /// One frame: the controllers and head in the stage, the newest state of pibridge-roommap (nullopt: none yet),
    /// the stage link, `now` in seconds.
    void frame(const room::FrameInput& in, double now, const std::optional<guide::State>& st, const StageLink& link);
    /// pibridge-roommap ended (its exit code; the last line it printed when it said nothing in the state).
    void childEnded(int code, const std::string& lastLine, double now);
    /// The answers to write, in order (each with the next id and this tool's pid).
    std::vector<guide::Answer> takeAnswers();
    void setPid(std::uint32_t pid) { pid_ = pid; }
    /// The user's eye height standing upright (pibridge.conf room_eye_height_m, or from the body height): in the floor
    /// steps the trigger then sets the floor that far below the head (standing still and upright, looking ahead, the
    /// head averaged over 1 s); A = the grid is on the floor; a controller on the floor + trigger = a touch.
    void setEyeHeight(std::optional<double> eye) { eye_ = eye; }
    /// Let the floor step follow the Pimax engine's own floor (pibridge-headtrack room-state.json). The real app turns
    /// this on; it stays off in tests so a live session never perturbs them.
    void setPimaxFloorSource(bool on) { pimaxFloor_ = on; }
    /// Draw the panel as a QPainter-painted texture (xrapp owns the painting + TexPass) instead of the vector card.
    /// When on, build() skips the vector card/text/chips; the laser, floor grid, ribbon and controller still draw.
    void setTexturedPanel(bool on) { texturedPanel_ = on; }
    /// Whether a painted panel should be drawn this frame (there is text or a chip and the flow has not ended).
    bool panelShown() const;
    /// The panel content for panelpaint: the header (first line), the body (the rest) and the chips in panel UV
    /// (0..1, y down) matching the hit-test, with hover and the primary (advancing) action marked.
    panel::Content panelContent() const;
    /// The four panel corners in the stage, matching UV (0,0) top-left, (1,0), (1,1), (0,1) bottom-left.
    std::array<room::Vec3, 4> panelCorners() const;
    /// The floor in the stage from the head of the last second: still (2 cm up and down, 4 cm sideways), looking
    /// ahead (pitch within 15 deg); nullopt otherwise.
    std::optional<double> standingFloor(double now) const;
    /// A line about the passthrough under the steps (empty: none), e.g. that there is no camera picture.
    void setNote(std::string note) { note_ = std::move(note); }
    /// The live scan cloud (room frame, from pibridge-roommap's guide `points` file) for the map-step "scanning"
    /// overlay; xrapp reads the file and hands it in each frame (empty clears it). build() draws it on the map step.
    void setScanPoints(std::vector<guide::P3f> pts) { scan_ = std::move(pts); }

    /// The scene over the passthrough: the panel, the coverage ring, the floor grid, the boundary, the ray, forward.
    void build(room::Batch& b, double now) const;
    std::vector<std::string> lines() const;
    /// The floor in the stage for the passthrough's floor plane: the map's floor once known, else 1.6 m below the head.
    double floorStage() const;
    bool floorKnown() const { return floorStage_.has_value(); }
    /// Stop the tool: 5 s after done / failed / the child's end, or right after a cancel when the child is gone.
    bool finished(double now) const;
    const std::string& step() const { return step_; }
    bool succeeded() const { return step_ == "done"; }
    const Coverage& coverage() const { return cov_; }
    /// The drawn boundary so far (stage, floor points).
    const std::vector<room::Vec3>& drawn() const { return path_; }
    /// The chosen setup mode once the mode menu is answered ("full" or "quick"); empty before.
    const std::string& mode() const { return mode_; }

    // --- controller render model (room setup): the XR side owns the OpenVR models; it tells build() to skip the box
    // proxy for a hand when it will draw the real model, and reads each hand's handle (grip) pose to place it. ---
    void setModelShown(int h, bool on) {
        if (h >= 0 && h < 2) {
            modelShown_[static_cast<std::size_t>(h)] = on;
        }
    }
    bool handModelValid(int h) const { return h >= 0 && h < 2 && handModelValid_[static_cast<std::size_t>(h)]; }
    room::Mat34 handModelPose(int h) const { return handModelPose_[static_cast<std::size_t>(h >= 0 && h < 2 ? h : 0)]; }

private:
    void answer(const std::string& kind, std::vector<double> nums);
    std::optional<room::Vec3> toRoom(room::Vec3 stage) const;
    std::optional<room::Vec3> toStage(room::Vec3 roomP) const;
    std::optional<room::Vec3> floorHit(const room::HandInput& h) const;
    void placePanel(const room::FrameInput& in, bool force);

    // --- the micro-window panel's buttons (room-setup rework): a styled card with button chips the pointer clicks ---
    struct PanelButton {
        std::string label;
        std::string action;    // mode-full | mode-quick | next | back | repeat | save
        double u = 0, v = 0;   // the chip centre on the panel plane (metres along right_ / up_ from at_)
        double hw = 0, hh = 0; // half width / height
    };
    /// Lay out the chips for the current step (empty on steps without any), a row below the text.
    void layoutButtons();
    /// Where a hand's pointing ray meets the panel plane, as (u, v, 0) in panel-local metres; nullopt if it misses.
    std::optional<room::Vec3> panelLocalHit(const room::HandInput& h) const;
    /// The chip under a panel-local (u, v) hit, or -1.
    int buttonAt(const room::Vec3& uv) const;
    /// Act on a pressed chip (emit the matching answer, or a client-side effect).
    void pressButton(const std::string& action, double now);
    /// Whether the current step has a step to go BACK to (the Back chip is hidden otherwise).
    bool backAvailable() const;

    std::uint32_t pid_ = 0;
    std::string note_;
    std::optional<double> eye_;
    struct HeadSample {
        double t;
        room::Vec3 p;
        double pitch;
    };
    std::deque<HeadSample> heads_;
    std::uint64_t id_ = 0;
    std::vector<guide::Answer> out_;
    std::optional<guide::State> st_;
    StageLink link_;
    std::string step_ = "starting";
    std::uint64_t answeredSeq_ = 0; // the state seq of the step already answered (one answer per step)
    std::string answeredStep_;
    std::array<std::uint64_t, 2> prev_{0, 0};
    std::string error_;
    std::optional<double> floorStage_;
    double headY_ = 1.6;
    bool headValid_ = false;
    room::Mat34 head_;
    Coverage cov_;
    std::vector<guide::P3f> scan_; // (unused now) a scan cloud from pibridge-roommap; kept so the plumbing compiles
    // The spatial-scan grid of the look-around step: floor tiles laid around where the user stood when the step began
    // (kScanTiles x kScanTiles cells of kScanTile metres). A tile is "scanned" once the head's gaze ray has crossed it;
    // the wall panels around the user are the Coverage sectors (level / up bands). build() draws them filling in.
    static constexpr int kScanTiles = 7;
    static constexpr double kScanTile = 0.75;
    std::optional<room::Vec3> scanOrigin_;
    std::array<bool, kScanTiles * kScanTiles> tileSeen_{};
    // wall panels lit by REAL map points (sector x band), on top of the gaze-driven Coverage
    std::array<std::array<bool, Coverage::kSectors>, 3> pointSectorSeen_{};
    double tileFraction() const;
    // the full-setup HEIGHT step ("рост"): the standing eye height measured from the head, fine-tuned with the stick
    std::optional<double> heightMeas_;
    std::optional<double> heightHeadY_; // the head's stage y the stature was measured from (frozen with the first nudge)
    bool heightNudged_ = false;
    // the boundary drawing
    int drawHand_ = -1;
    double drawStart_ = 0;
    std::vector<room::Vec3> path_;
    double pathLen_ = 0;
    std::array<std::optional<room::Vec3>, 2> hits_;
    int rayHand_ = 1;
    // forward by holding still
    std::optional<std::pair<double, room::Mat34>> still_;
    double stepSince_ = 0;
    // stop with B twice
    std::optional<double> stopAsked_;
    bool cancelled_ = false;
    std::optional<double> endedAt_;
    std::string endText_;
    // the panel
    bool placed_ = false;
    room::Vec3 at_{0, 1.45, -1.3}, right_{1, 0, 0}, up_{0, 1, 0};
    double panelPitch_ = 0;
    bool following_ = false;
    // the mode menu and the panel's button chips
    std::string mode_;             // "full" | "quick", set when the mode menu is answered
    std::vector<PanelButton> buttons_;
    int hover_ = -1;               // the chip under the pointer this frame, or -1
    // the floor-grid nudge from the thumbstick (metres, folded into floorStage()); reset at every step change
    double floorAdjust_ = 0;
    double lastNudge_ = 0;         // the time of the last stick step (rate-limit)
    // the custom/full standing floor step: the head-relative drop (metres below the face) the user fine-tunes with the
    // stick; set once on the floor step, clamped to [FLOOR_DROP_MIN_M, FLOOR_DROP_MAX_M]
    std::optional<double> floorDrop_;
    bool floorNudged_ = false; // the user moved the stick: stop following the Pimax floor, keep the manual drop
    bool pimaxFloor_ = false;   // read the Pimax engine's floor (room-state.json); off by default so tests are not
                                // affected by a live session -- the real app turns it on (setPimaxFloorSource)
    bool texturedPanel_ = false; // draw the QPainter-painted panel (xrapp) instead of the vector card
    // The Pimax engine's own floor from pibridge-headtrack's room-state.json (GetGroundState -> head-relative drop),
    // when the Pimax SLAM engine runs: the floor-below-the-head it currently estimates. nullopt when the file is
    // missing / stale / the floor is not valid (ground_ret != 1 or the head is not tracked). Read live on the floor
    // step so the drop follows the real floor as the user looks at it and stands upright.
    std::optional<double> pimaxFloorDrop() const;
    // the controller lasers and box proxies (so the user can see where they point): each hand's grip pose and the
    // panel-hit world point, kept from frame() for build() to draw every frame
    std::array<room::Mat34, 2> handGrip_;
    std::array<bool, 2> handValid_{false, false};
    std::array<std::optional<room::Vec3>, 2> panelHitWorld_;
    // the controller render model: the handle (grip) pose per hand, and whether the XR side will draw the real model
    // for that hand this frame (then build() skips the box proxy but keeps the laser)
    std::array<room::Mat34, 2> handModelPose_;
    std::array<bool, 2> handModelValid_{false, false};
    std::array<bool, 2> modelShown_{false, false};
};

} // namespace pibridge::roomsetup
