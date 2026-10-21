// SPDX-License-Identifier: MIT
#include "mapflow.h"

#include "recenter.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>

namespace pibridge::roomsetup {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;

double wrapPi(double a) {
    while (a > kPi) {
        a -= 2 * kPi;
    }
    while (a < -kPi) {
        a += 2 * kPi;
    }
    return a;
}

room::Vec3 rotY(double yaw, room::Vec3 p) {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * p.x + s * p.z, p.y, -s * p.x + c * p.z};
}

bool pressed(std::uint64_t now, std::uint64_t prev, unsigned bit) {
    return (now & (1ULL << bit)) != 0 && (prev & (1ULL << bit)) == 0;
}
bool down(std::uint64_t now, unsigned bit) { return (now & (1ULL << bit)) != 0; }

bool isFloorStep(const std::string& s) { return s == "floor-confirm" || s == "floor-touch" || s == "floor-height"; }

bool isMapStep(const std::string& s) { return s == "wear" || s == "light" || s == "map" || s == "starting"; }

std::string upper(std::string s) {
    for (auto& c : s) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return s;
}

const room::Rgba kGrid{0.35F, 0.9F, 1.0F, 0.75F};
const room::Rgba kWall{0.2F, 1.0F, 0.5F, 0.22F};
const room::Rgba kDraw{1.0F, 0.85F, 0.2F, 1.0F};
constexpr double kWallH = 2.0;        // the boundary walls (m)
constexpr double kTouchBelowHead = 0.9;
constexpr double kTouchNearGrid = 0.3;
// the standing floor-drop fine-tune range (mirror of setup::FLOOR_DROP_MIN_M / MAX_M): the floor is 1.0 .. 2.0 m below
// the face
constexpr double kFloorDropMin = 1.00;
constexpr double kFloorDropMax = 2.00;
const room::Rgba kText{0.92F, 0.96F, 1.0F, 1.0F};
// the micro-window card (room-setup rework): a framed body, a header strip, bordered button chips
const room::Rgba kCard{0.04F, 0.06F, 0.10F, 0.82F};
const room::Rgba kCardEdge{0.45F, 0.85F, 1.0F, 0.9F};
const room::Rgba kHeader{0.10F, 0.20F, 0.32F, 0.9F};
const room::Rgba kChip{0.12F, 0.26F, 0.40F, 0.92F};
const room::Rgba kChipHover{0.25F, 0.65F, 0.95F, 0.95F};
const room::Rgba kChipEdge{0.6F, 0.9F, 1.0F, 0.95F};
// the boundary ribbon and the zone fill grid drawn on the floor inside the traced area
const room::Rgba kRibbon{1.0F, 0.85F, 0.2F, 0.95F};
const room::Rgba kZoneGrid{0.35F, 0.9F, 1.0F, 0.6F};
// the controller laser and box proxy (so the user can see where the pointer goes)
const room::Rgba kBeam{0.35F, 0.95F, 1.0F, 0.95F};
const room::Rgba kBox{0.7F, 0.9F, 1.0F, 0.95F};
constexpr double kPanelHalfW = 0.5; // the card half width (metres), matches the text columns
constexpr double kLineH = 0.06;     // one text line's height on the panel

} // namespace

double headingOf(const room::Mat34& m) {
    const room::Vec3 f = m.axis(2) * -1.0;
    if (std::hypot(f.x, f.z) > 0.2) {
        return std::atan2(-f.x, -f.z);
    }
    const room::Vec3 x = m.axis(0);
    return std::atan2(-x.z, x.x);
}

// ---------------------------------------------------------------- StageLink

void StageLink::reset() {
    win_.clear();
    jump_.clear();
    n_ = 0;
    yaw_ = 0;
    t_ = {};
}

void StageLink::add(const room::Mat34& s, const room::Mat34& r) {
    // R = Rs Rr^T: the rotation stage_from_raw; its yaw (the frames share gravity)
    double R[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R[i][j] = s.m[i][0] * r.m[j][0] + s.m[i][1] * r.m[j][1] + s.m[i][2] * r.m[j][2];
        }
    }
    const double yaw = std::atan2(R[0][2] - R[2][0], R[0][0] + R[2][2]);
    const Pair p{yaw, s.pos() - rotY(yaw, r.pos())};
    const auto far = [this](const Pair& q) {
        return (q.t - t_).length() > 0.10 || std::fabs(wrapPi(q.yaw - yaw_)) > 4 * kDeg;
    };
    if (valid() && far(p)) {
        jump_.push_back(p);
        if (jump_.size() >= 10) {
            win_.assign(jump_.begin(), jump_.end());
            jump_.clear();
            estimate();
        }
        return;
    }
    jump_.clear();
    win_.push_back(p);
    while (win_.size() > kWindow) {
        win_.pop_front();
    }
    estimate();
}

void StageLink::estimate() {
    double sx = 0, cx = 0;
    room::Vec3 t{};
    for (const auto& p : win_) {
        sx += std::sin(p.yaw);
        cx += std::cos(p.yaw);
        t = t + p.t;
    }
    n_ = win_.size();
    if (n_ == 0) {
        return;
    }
    yaw_ = std::atan2(sx, cx);
    t_ = t * (1.0 / static_cast<double>(n_));
}

room::Vec3 StageLink::toStage(room::Vec3 raw) const { return rotY(yaw_, raw) + t_; }
room::Vec3 StageLink::toRaw(room::Vec3 stage) const { return rotY(-yaw_, stage - t_); }

// ---------------------------------------------------------------- Coverage

double Coverage::sectorHeading(int s) { return (s + 0.5) * 2 * kPi / kSectors; }

void Coverage::look(const room::Mat34& head) {
    const room::Vec3 f = (head.axis(2) * -1.0).norm();
    const double pitch = std::asin(std::clamp(f.y, -1.0, 1.0));
    const int band = pitch < -25 * kDeg ? 0 : pitch > 25 * kDeg ? 2 : 1;
    double h = std::fmod(headingOf(head), 2 * kPi);
    if (h < 0) {
        h += 2 * kPi;
    }
    const int s = std::clamp(static_cast<int>(h / (2 * kPi / kSectors)), 0, kSectors - 1);
    seen_[static_cast<std::size_t>(band)][static_cast<std::size_t>(s)] = true;
}

double Coverage::fraction() const {
    const auto count = [this](int b) { return static_cast<double>(std::ranges::count(seen_[static_cast<std::size_t>(b)], true)); };
    return count(1) / kSectors * 0.7 + std::min(count(0), 6.0) / 6.0 * 0.2 + std::min(count(2), 3.0) / 3.0 * 0.1;
}

std::string Coverage::hint(double heading) const {
    std::optional<double> best;
    for (int s = 0; s < kSectors; s++) {
        if (!seen(1, s)) {
            const double d = wrapPi(sectorHeading(s) - heading);
            if (!best || std::fabs(d) < std::fabs(*best)) {
                best = d;
            }
        }
    }
    if (best) {
        if (std::fabs(*best) > 120 * kDeg) {
            return "LOOK BEHIND YOU";
        }
        return *best > 0 ? "LOOK LEFT" : "LOOK RIGHT";
    }
    if (std::ranges::count(seen_[0], true) < 4) {
        return "LOOK DOWN AT THE FLOOR";
    }
    if (std::ranges::count(seen_[2], true) < 2) {
        return "LOOK UP";
    }
    return {};
}

// ---------------------------------------------------------------- MapFlow

std::optional<room::Vec3> MapFlow::toRoom(room::Vec3 stage) const {
    if (!st_ || !st_->conv || !link_.valid()) {
        return std::nullopt;
    }
    const room::Vec3 raw = link_.toRaw(stage);
    const auto r = st_->conv->apply({raw.x, raw.y, raw.z});
    return room::Vec3{r[0], r[1], r[2]};
}

std::optional<room::Vec3> MapFlow::toStage(room::Vec3 p) const {
    if (!st_ || !st_->conv || !link_.valid()) {
        return std::nullopt;
    }
    const auto raw = st_->conv->inverse_apply({p.x, p.y, p.z});
    return link_.toStage({raw[0], raw[1], raw[2]});
}

double MapFlow::floorStage() const {
    // on the standing floor step the grid follows the head-relative drop the user is tuning with the stick
    if (isFloorStep(step_) && floorDrop_ && headValid_) {
        return head_.pos().y - *floorDrop_;
    }
    return floorStage_.value_or(headY_ - 1.6) + floorAdjust_;
}

std::optional<room::Vec3> MapFlow::floorHit(const room::HandInput& h) const {
    if (!h.valid) {
        return std::nullopt;
    }
    const room::Vec3 o = h.grip.pos();
    const room::Vec3 d = (h.grip.axis(2) * -1.0).norm();
    if (d.y > -0.05) {
        return std::nullopt;
    }
    const double t = (floorStage() - o.y) / d.y;
    if (t <= 0 || t > 8) {
        return std::nullopt;
    }
    return o + d * t;
}

void MapFlow::answer(const std::string& kind, std::vector<double> nums) {
    if (kind != "cancel" && st_ && answeredSeq_ == st_->seq && answeredStep_ == step_) {
        return; // one answer until pibridge-roommap has reacted
    }
    guide::Answer a;
    a.pid = pid_;
    a.id = ++id_;
    a.step = step_;
    a.kind = kind;
    a.nums = std::move(nums);
    out_.push_back(std::move(a));
    answeredSeq_ = st_ ? st_->seq : 0;
    answeredStep_ = step_;
}

std::vector<guide::Answer> MapFlow::takeAnswers() {
    std::vector<guide::Answer> o;
    o.swap(out_);
    return o;
}

void MapFlow::placePanel(const room::FrameInput& in, bool force) {
    if (!in.headValid) {
        return;
    }
    const room::Vec3 g = (in.head.axis(2) * -1.0).norm();
    room::Vec3 f{g.x, 0, g.z};
    const double n = std::hypot(f.x, f.z);
    if (n < 1e-6) {
        f = (in.head.axis(1) * 1.0);
        f.y = 0;
        if (std::hypot(f.x, f.z) < 1e-6) {
            return;
        }
    }
    f = f.norm();
    // along the gaze, pitched between 30 deg down and 5 deg up, facing the head
    const double pitch = std::clamp(std::asin(std::clamp(g.y, -1.0, 1.0)), -30 * kDeg, 5 * kDeg);
    const auto put = [&](room::Vec3 dir, double p) {
        const room::Vec3 f3 = dir * std::cos(p) + room::Vec3{0, 1, 0} * std::sin(p);
        right_ = {-dir.z, 0, dir.x};
        up_ = right_.cross(f3);
        panelPitch_ = p;
        return in.head.pos() + f3 * 1.3;
    };
    if (!placed_ || force) {
        at_ = put(f, pitch);
        placed_ = true;
        return;
    }
    // the panel follows when you turn away from it (more than 40 deg, or 20 deg up or down), gliding
    room::Vec3 d = at_ - in.head.pos();
    d.y = 0;
    const double dn = std::hypot(d.x, d.z);
    const double off = dn < 1e-6 ? kPi : std::acos(std::clamp((d.x * f.x + d.z * f.z) / dn, -1.0, 1.0));
    if (off > 40 * kDeg || std::fabs(pitch - panelPitch_) > 20 * kDeg || (at_ - in.head.pos()).length() > 2.0) {
        following_ = true;
    }
    if (following_) {
        const double k = std::clamp(in.dt * 3.0, 0.0, 1.0);
        const room::Vec3 cur = dn < 1e-6 ? f : d * (1.0 / dn);
        const room::Vec3 dir = (cur + (f - cur) * k).norm();
        const double p = panelPitch_ + (pitch - panelPitch_) * k;
        at_ = put(dir, p);
        if (off < 3 * kDeg && std::fabs(pitch - p) < 2 * kDeg) {
            following_ = false;
        }
    }
}

bool MapFlow::backAvailable() const {
    // only the custom boundary step has a previous step to return to (the floor step); everything else is either the
    // first interactive step or a terminal confirm.
    return step_ == "boundary";
}

void MapFlow::layoutButtons() {
    buttons_.clear();
    std::vector<std::pair<std::string, std::string>> defs; // label, action
    if (step_ == "mode") {
        defs = {{"FULL MAPPING", "mode-full"}, {"QUICK SETUP", "mode-quick"}};
    } else if (step_ == "posture") {
        defs = {{"STAND", "posture-stand"}, {"SIT", "posture-sit"}};
    } else if (step_ == "size") {
        defs = {{"SMALL", "size-s"}, {"MEDIUM", "size-m"}, {"BIG", "size-b"}};
    } else if (step_ == "map") {
        defs = {{"DONE LOOKING", "next"}}; // only ends the look-around; it does not finish the setup
    } else if (step_ == "confirm") {
        defs = {{"FINISH", "next"}}; // quick: the one primary button that completes the setup
    } else if (isFloorStep(step_)) {
        defs.push_back({"REPEAT", "repeat"}); // reset the floor to the default drop
        defs.push_back({"NEXT", "next"});     // confirm the floor, go to the boundary
    } else if (step_ == "boundary") {
        if (backAvailable()) {
            defs.push_back({"BACK", "back"}); // back to the floor step
        }
        defs.push_back({"REPEAT", "repeat"}); // clear the stroke and draw again
        defs.push_back({"SAVE", "save"});     // accept the drawn boundary and finish
    }
    if (defs.empty()) {
        return;
    }
    // a centred row of chips just below the last text line (lines() is the same here and in build() this frame)
    const double nlines = static_cast<double>(lines().size());
    const double rowV = 0.06 - kLineH * nlines - 0.03;
    const bool wide = step_ == "mode" || step_ == "posture";
    const double hw = wide ? 0.21 : 0.13, hh = wide ? 0.05 : 0.032, gap = 0.035;
    const double total = static_cast<double>(defs.size()) * 2 * hw + static_cast<double>(defs.size() - 1) * gap;
    double u = -total / 2 + hw;
    for (const auto& d : defs) {
        buttons_.push_back({d.first, d.second, u, rowV - hh, hw, hh});
        u += 2 * hw + gap;
    }
}

std::optional<room::Vec3> MapFlow::panelLocalHit(const room::HandInput& h) const {
    if (!h.valid) {
        return std::nullopt;
    }
    const room::Vec3 n = right_.cross(up_).norm();
    const room::Vec3 o = h.grip.pos();
    const room::Vec3 d = (h.grip.axis(2) * -1.0).norm();
    const double denom = d.dot(n);
    if (std::fabs(denom) < 1e-4) {
        return std::nullopt;
    }
    const double t = (at_ - o).dot(n) / denom;
    if (t <= 0 || t > 6) {
        return std::nullopt;
    }
    const room::Vec3 rel = (o + d * t) - at_;
    return room::Vec3{rel.dot(right_), rel.dot(up_), 0};
}

int MapFlow::buttonAt(const room::Vec3& uv) const {
    for (std::size_t i = 0; i < buttons_.size(); i++) {
        const PanelButton& b = buttons_[i];
        if (std::fabs(uv.x - b.u) <= b.hw && std::fabs(uv.y - b.v) <= b.hh) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void MapFlow::pressButton(const std::string& action, double now) {
    (void)now; // no button handler needs the time any more (the floor uses the stick-tuned drop)
    if (action == "mode-full") {
        mode_ = "full";
        answer("mode", {1.0});
        return;
    }
    if (action == "mode-quick") {
        mode_ = "quick";
        answer("mode", {0.0});
        return;
    }
    if (action == "posture-stand") {
        answer("posture", {0.0});
        return;
    }
    if (action == "posture-sit") {
        answer("posture", {1.0});
        return;
    }
    if (action == "size-s") {
        answer("size", {0.0});
        return;
    }
    if (action == "size-m") {
        answer("size", {1.0});
        return;
    }
    if (action == "size-b") {
        answer("size", {2.0});
        return;
    }
    if (action == "back") {
        answer("back", {});
        return;
    }
    if (action == "repeat") {
        if (step_ == "boundary") {
            path_.clear(); // start the stroke over
            pathLen_ = 0;
        }
        if (isFloorStep(step_)) {
            floorDrop_.reset(); // reset the floor to the posture / eye default on the next frame
        }
        answer("repeat", {});
        return;
    }
    if (step_ == "map" && action == "next") {
        answer("next", {}); // pibridge-roommap ends the walk early
        return;
    }
    if (step_ == "confirm" && action == "next") {
        answer("next", {}); // quick: the final FINISH; pibridge-roommap then writes everything from the live head
        return;
    }
    if (isFloorStep(step_) && action == "next") {
        // save the floor as the HEAD-RELATIVE drop the user tuned with the stick: answer height {room y, drop}. The
        // drop (not a raw room y) is what the chaperone uses, so the floor is always "N below the face".
        if (headValid_ && floorDrop_) {
            const double fy = head_.pos().y - *floorDrop_; // the floor plane in the stage
            if (const auto r = toRoom({head_.pos().x, fy, head_.pos().z})) {
                error_.clear();
                answer("height", {r->y, *floorDrop_});
            } else {
                error_ = "WAITING FOR THE HEAD TRACKING. LOOK AROUND A LITTLE.";
            }
        }
        return;
    }
    if (step_ == "boundary" && action == "save") {
        if (!path_.empty()) {
            std::vector<double> xz;
            for (const auto& p : path_) {
                if (const auto r = toRoom(p)) {
                    xz.push_back(r->x);
                    xz.push_back(r->z);
                }
            }
            if (xz.size() >= 6) {
                error_.clear();
                answer("path", std::move(xz));
                path_.clear();
            } else {
                error_ = "DRAW THE EDGE ALL THE WAY AROUND ON THE FLOOR, THEN SAVE.";
            }
        } else if (st_ && st_->boundary.size() >= 3) {
            error_.clear();
            answer("yes", {}); // keep the proposed free area
        } else {
            error_ = "DRAW THE EDGE OF YOUR SPACE ON THE FLOOR, THEN SAVE.";
        }
        return;
    }
}

std::optional<double> MapFlow::standingFloor(double now) const {
    if (!eye_ || heads_.empty() || now - heads_.front().t < 0.9) {
        return std::nullopt;
    }
    double lo = 1e9, hi = -1e9, sum = 0;
    std::size_t n = 0;
    for (const auto& h : heads_) {
        if (now - h.t > 1.0) {
            continue;
        }
        if (std::fabs(h.pitch) > 15 * kDeg || std::hypot(h.p.x - heads_.back().p.x, h.p.z - heads_.back().p.z) > 0.04) {
            return std::nullopt;
        }
        lo = std::min(lo, h.p.y);
        hi = std::max(hi, h.p.y);
        sum += h.p.y;
        n++;
    }
    if (n < 10 || hi - lo > 0.02) {
        return std::nullopt;
    }
    return sum / static_cast<double>(n) - *eye_;
}

void MapFlow::frame(const room::FrameInput& in, double now, const std::optional<guide::State>& st, const StageLink& link) {
    headValid_ = in.headValid;
    for (std::size_t i = 0; i < 2; i++) {
        handValid_[i] = in.hands[i].valid;
        handGrip_[i] = in.hands[i].grip;          // the aim pose (pointing ray) for the laser
        handModelValid_[i] = in.hands[i].gripPoseValid;
        handModelPose_[i] = in.hands[i].gripPose;  // the handle pose for the render model
    }
    if (in.headValid) {
        head_ = in.head;
        headY_ = in.head.pos().y;
        const room::Vec3 g = (in.head.axis(2) * -1.0).norm();
        heads_.push_back({now, in.head.pos(), std::asin(std::clamp(g.y, -1.0, 1.0))});
    }
    while (!heads_.empty() && now - heads_.front().t > 1.5) {
        heads_.pop_front();
    }
    link_ = link;
    bool stepChanged = false;
    if (st && (!st_ || st->seq != st_->seq || st->pid != st_->pid)) {
        if (!endedAt_ && st->step != step_) {
            step_ = st->step;
            stepChanged = true;
        }
        st_ = st;
    }
    if (stepChanged) {
        error_.clear();
        drawHand_ = -1;
        path_.clear();
        pathLen_ = 0;
        still_.reset();
        stepSince_ = now;
        floorAdjust_ = 0; // the floor nudge is per floor step
        lastNudge_ = 0;
        floorDrop_.reset();
    }
    // The backend reached the end (done) or failed: end the frontend here so the XR loop exits and the process returns,
    // which lets pibridge-roomsetup write the SteamVR chaperone. Previously we only ended when the backend *process*
    // exited, so a lingering backend left the frontend (and the chaperone write) hanging on the final step.
    if (!endedAt_ && (step_ == "done" || step_ == "failed")) {
        endedAt_ = now;
        if (step_ == "failed" && endText_.empty()) {
            endText_ = st_ ? st_->text : std::string();
        }
    }
    if (st_ && st_->floor) {
        if (const auto p = toStage({0, *st_->floor, 0})) {
            floorStage_ = p->y;
        } // else keep the last one: the grid never vanishes while the link catches up
    } else {
        floorStage_.reset();
    }
    placePanel(in, stepChanged && !isMapStep(step_));
    if (isMapStep(step_) && in.headValid) {
        cov_.look(in.head);
    }
    std::array<std::uint64_t, 2> btn{in.hands[0].buttons, in.hands[1].buttons};
    for (std::size_t i = 0; i < 2; i++) {
        hits_[i] = floorHit(in.hands[i]);
        if (hits_[i] && pressed(btn[i], prev_[i], room::bit::Trigger)) {
            rayHand_ = static_cast<int>(i);
        }
    }
    const auto done = [&] { prev_ = btn; };
    if (endedAt_) {
        done();
        return;
    }
    for (std::size_t i = 0; i < 2; i++) {
        if (pressed(btn[i], prev_[i], room::bit::Menu)) {
            if (stopAsked_ && now - *stopAsked_ < 3.0) {
                answer("cancel", {});
                cancelled_ = true;
                stopAsked_.reset();
            } else {
                stopAsked_ = now;
            }
        }
    }
    // The custom/full standing FLOOR step: the floor is a head-relative drop ("the floor is N m below your face"),
    // started at the default the backend published (the entered eye height, else 1.60 m) and fine-tuned with the
    // thumbstick +/- 1 cm, clamped to [kFloorDropMin, kFloorDropMax]. Stick up raises the floor (smaller drop), down
    // lowers it. No controller-lowering, no "touch the floor" action. Quick has no floor step.
    if (isFloorStep(step_)) {
        if (!floorDrop_) {
            double d = eye_.value_or(1.60);
            if (headValid_ && st_ && st_->floor) {
                if (const auto fs = toStage({0, *st_->floor, 0})) {
                    d = head_.pos().y - fs->y; // the published default floor, as a drop below the head
                }
            }
            floorDrop_ = std::clamp(d, kFloorDropMin, kFloorDropMax);
        }
        double sy = 0;
        for (const auto& h : in.hands) {
            if (std::fabs(static_cast<double>(h.stickY)) > std::fabs(sy)) {
                sy = h.stickY;
            }
        }
        if (std::fabs(sy) > 0.5 && now - lastNudge_ > 0.12) {
            floorDrop_ = std::clamp(*floorDrop_ + (sy > 0 ? -0.01 : 0.01), kFloorDropMin, kFloorDropMax);
            lastNudge_ = now;
        }
    }
    // the panel's button chips: the pointer hovers one and the trigger presses it (that trigger is then consumed)
    layoutButtons();
    hover_ = -1;
    int hoverHand = -1;
    panelHitWorld_[0].reset();
    panelHitWorld_[1].reset();
    for (std::size_t i = 0; i < 2; i++) {
        if (const auto uv = panelLocalHit(in.hands[i])) {
            if (const int b = buttonAt(*uv); b >= 0) {
                hover_ = b;
                hoverHand = static_cast<int>(i);
                // the hit point back in the stage (on the panel plane) for the laser's end
                panelHitWorld_[i] = at_ + right_ * uv->x + up_ * uv->y;
            }
        }
    }
    std::array<bool, 2> btnPressed{false, false};
    if (hover_ >= 0 && hoverHand >= 0) {
        const auto hh = static_cast<std::size_t>(hoverHand);
        if (pressed(btn[hh], prev_[hh], room::bit::Trigger)) {
            pressButton(buttons_[static_cast<std::size_t>(hover_)].action, now);
            btnPressed[hh] = true;
        }
    }
    for (std::size_t i = 0; i < 2; i++) {
        if (btnPressed[i]) {
            continue; // the trigger went to a panel button, not the floor / boundary / forward
        }
        const bool trig = pressed(btn[i], prev_[i], room::bit::Trigger);
        // the floor step is driven by the stick + the NEXT chip (above / in pressButton), not by the controllers
        if (step_ == "boundary") {
            if (trig && drawHand_ < 0) {
                drawHand_ = static_cast<int>(i);
                drawStart_ = now;
                path_.clear();
                pathLen_ = 0;
                error_.clear();
            }
            if (drawHand_ == static_cast<int>(i) && down(btn[i], room::bit::Trigger) && hits_[i]) {
                if (path_.empty() || (*hits_[i] - path_.back()).length() >= 0.05) {
                    if (!path_.empty()) {
                        pathLen_ += (*hits_[i] - path_.back()).length();
                    }
                    path_.push_back(*hits_[i]);
                }
            }
            if (drawHand_ == static_cast<int>(i) && !down(btn[i], room::bit::Trigger)) {
                // Stroke finished. The trigger used for drawing (ray on the floor, not a menu chip) NEVER submits or
                // advances: the drawn stroke stays as the pending boundary and the user must press the SAVE chip to
                // accept it. Just give a hint here.
                drawHand_ = -1;
                if (pathLen_ >= 2.0) {
                    error_ = "PRESS SAVE TO KEEP THIS BOUNDARY, OR DRAW AGAIN.";
                } else if (!path_.empty()) {
                    error_ = "THAT WAS TOO SHORT. DRAW ALL THE WAY AROUND, THEN PRESS SAVE.";
                }
            }
        }
    }
    // (the old "forward" step is gone: in the custom flow the centre and forward are taken from the live head at SAVE)
    done();
}

void MapFlow::childEnded(int code, const std::string& lastLine, double now) {
    if (endedAt_) {
        return;
    }
    endedAt_ = now;
    if (step_ != "done" && step_ != "failed") {
        step_ = code == 0 ? "done" : "failed";
        endText_ = code == 0 ? std::string() : (lastLine.empty() ? std::format("pibridge-roommap ended with code {}", code) : lastLine);
    } else if (code != 0 && step_ == "done") {
        step_ = "failed";
        endText_ = lastLine;
    }
}

bool MapFlow::finished(double now) const {
    return endedAt_ && now - *endedAt_ > (cancelled_ ? 1.5 : 2.0);
}

std::vector<std::string> MapFlow::lines() const {
    std::vector<std::string> l;
    const auto add = [&l](const std::string& s, std::size_t w = 34) {
        for (auto& x : room::wrapText(s, w)) {
            l.push_back(x);
        }
    };
    if (!st_) {
        l.push_back("ROOM MAPPING");
        add(endedAt_ ? (endText_.empty() ? std::string("THE ROOM MAPPING DID NOT START.") : endText_)
                     : std::string("STARTING THE ROOM MAPPING..."));
    } else {
        l.push_back(upper(st_->title));
        add(st_->text);
        if (endedAt_ && !endText_.empty() && step_ == "failed" && st_->step != "failed") {
            add(endText_);
        }
    }
    if (isMapStep(step_) && st_ && step_ == "map") {
        l.push_back(std::format("VIEWS MAPPED {}   POINTS IN VIEW {}", st_->keyframes, st_->points));
        l.push_back(std::format("ROOM SEEN {:.0f} %", cov_.fraction() * 100.0));
        if (const auto h = cov_.hint(headValid_ ? headingOf(head_) : 0.0); !h.empty()) {
            l.push_back(h);
        }
    }
    if (isFloorStep(step_) && floorDrop_ && !endedAt_) {
        l.push_back(std::format("FLOOR IS {:.2f} M BELOW YOUR FACE", *floorDrop_));
    }
    if (!error_.empty()) {
        add(error_);
    }
    if (stopAsked_ && !endedAt_) {
        l.push_back("PRESS B AGAIN TO STOP");
    }
    if (!note_.empty()) {
        add(note_);
    }
    return l;
}

void MapFlow::build(room::Batch& b, double now) const {
    b.clear();
    (void)now;
    const room::Vec3 up{0, 1, 0};
    const double fy = floorStage();
    const room::Vec3 hp = headValid_ ? head_.pos() : room::Vec3{0, headY_, 0};

    // (room-setup rework) the coverage ring around you is gone; the look-around feedback is the ROOM SEEN % and the
    // hint in the panel. A floor grid is shown only on the floor steps, so the floor can be confirmed; everywhere else
    // the grid appears only inside the drawn zone (below).
    if (isFloorStep(step_) && floorStage_) {
        const room::Rgba g = kGrid;
        const double cx = std::round(hp.x * 2) / 2, cz = std::round(hp.z * 2) / 2;
        for (int i = -6; i <= 6; i++) {
            for (int j = -6; j < 6; j++) {
                const double a = i * 0.5, c0 = j * 0.5, c1 = (j + 1) * 0.5;
                b.line({cx + a, fy, cz + c0}, {cx + a, fy, cz + c1}, g);
                b.line({cx + c0, fy, cz + a}, {cx + c1, fy, cz + a}, g);
            }
        }
    }

    // ray casting point-in-polygon (x, z) and helpers for the thick boundary ribbon and the zone fill grid
    const auto inside = [](const std::vector<room::Vec3>& poly, double x, double z) {
        bool in = false;
        for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
            if (((poly[i].z > z) != (poly[j].z > z)) &&
                (x < (poly[j].x - poly[i].x) * (z - poly[i].z) / (poly[j].z - poly[i].z) + poly[i].x)) {
                in = !in;
            }
        }
        return in;
    };
    const auto ribbon = [&](const std::vector<room::Vec3>& pts, bool closed, double halfw, room::Rgba c) {
        const std::size_t n = pts.size();
        if (n < 2) {
            return;
        }
        const std::size_t segs = closed ? n : n - 1;
        for (std::size_t i = 0; i < segs; i++) {
            room::Vec3 a = pts[i], d = pts[(i + 1) % n];
            a.y = d.y = fy + 0.006;
            room::Vec3 dir = d - a;
            dir.y = 0;
            const double len = std::hypot(dir.x, dir.z);
            if (len < 1e-6) {
                continue;
            }
            dir = dir * (1.0 / len);
            const room::Vec3 side{-dir.z * halfw, 0, dir.x * halfw};
            b.quad(a - side, d - side, d + side, a + side, c); // a floor ribbon of 2*halfw width
        }
    };
    const auto fillGrid = [&](const std::vector<room::Vec3>& poly) {
        if (poly.size() < 3) {
            return;
        }
        double minx = 1e9, maxx = -1e9, minz = 1e9, maxz = -1e9;
        for (const auto& p : poly) {
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            minz = std::min(minz, p.z);
            maxz = std::max(maxz, p.z);
        }
        const double cell = 0.5, seg = 0.1, gy = fy + 0.004;
        for (double x = std::ceil(minx / cell) * cell; x <= maxx; x += cell) {
            for (double z = minz; z < maxz; z += seg) {
                if (inside(poly, x, z + seg * 0.5)) {
                    b.line({x, gy, z}, {x, gy, std::min(z + seg, maxz)}, kZoneGrid);
                }
            }
        }
        for (double z = std::ceil(minz / cell) * cell; z <= maxz; z += cell) {
            for (double x = minx; x < maxx; x += seg) {
                if (inside(poly, x + seg * 0.5, z)) {
                    b.line({x, gy, z}, {std::min(x + seg, maxx), gy, z}, kZoneGrid);
                }
            }
        }
    };

    // the kept / proposed boundary (room frame) in the stage: the enclosed zone filled with a grid, a thick ribbon on
    // the floor, and faint low walls for a height cue. Not on the map / floor steps (the zone only after it is drawn).
    if (st_ && !isMapStep(step_) && !isFloorStep(step_) && st_->boundary.size() >= 3) {
        std::vector<room::Vec3> poly;
        for (const auto& p : st_->boundary) {
            if (const auto s = toStage({p[0], st_->floor.value_or(0), p[1]})) {
                poly.push_back({s->x, fy, s->z});
            }
        }
        if (poly.size() >= 3) {
            fillGrid(poly);
            ribbon(poly, true, 0.025, kRibbon);
            for (std::size_t i = 0; i < poly.size(); i++) {
                const room::Vec3 a = poly[i], c = poly[(i + 1) % poly.size()];
                b.quad(a, c, c + up * kWallH, a + up * kWallH, kWall.alpha(0.12F));
            }
        }
    }
    // the stroke being drawn: the same thick ribbon (open), bright
    if (path_.size() >= 2) {
        ribbon(path_, false, 0.025, kRibbon);
    }
    // the ray's floor point while drawing the boundary
    if (step_ == "boundary") {
        for (std::size_t i = 0; i < 2; i++) {
            if (hits_[i] && (static_cast<int>(i) == rayHand_ || !hits_[static_cast<std::size_t>(rayHand_)])) {
                const room::Vec3 h = *hits_[i];
                const room::Rgba c = drawHand_ == static_cast<int>(i) ? kRibbon : kText.alpha(0.8F);
                for (int s = 0; s < 16; s++) {
                    const double a0 = s * kPi / 8, a1 = (s + 1) * kPi / 8;
                    b.line({h.x + 0.06 * std::cos(a0), fy + 0.005, h.z + 0.06 * std::sin(a0)},
                           {h.x + 0.06 * std::cos(a1), fy + 0.005, h.z + 0.06 * std::sin(a1)}, c);
                }
            }
        }
    }
    // forward: an arrow on the floor where you look
    if (step_ == "forward" && headValid_) {
        const double h = headingOf(head_);
        const room::Vec3 f{-std::sin(h), 0, -std::cos(h)}, r{-f.z, 0, f.x};
        const room::Vec3 o{hp.x, fy + 0.01, hp.z};
        const room::Vec3 tip = o + f * 1.2;
        b.quad(o + f * 0.3 - r * 0.04, o + f * 0.95 - r * 0.04, o + f * 0.95 + r * 0.04, o + f * 0.3 + r * 0.04, kDraw);
        b.tri(tip, o + f * 0.95 + r * 0.14, o + f * 0.95 - r * 0.14, kDraw);
        b.line(o - r * 0.15, o + r * 0.15, kDraw);
        b.line(o - f * 0.15, o + f * 0.15, kDraw);
    }

    // the controller laser (always) plus a shape: the real SteamVR render model is drawn by the XR side (rendermodel.h)
    // when loaded, which sets modelShown_ so build() skips the box proxy for that hand; otherwise the box proxy (ported
    // from the editor's drawTools, libpibridge-room scene.cpp) stands in. The beam ends at the hovered chip, else the
    // floor point, else ~3 m out.
    for (std::size_t i = 0; i < 2; i++) {
        if (!handValid_[i]) {
            continue;
        }
        const room::Mat34& m = handGrip_[i];
        const room::Vec3 o = m.pos();
        const room::Vec3 dir = (m.axis(2) * -1.0).norm();
        // the box along the controller's axes (only when the real model is not being drawn for this hand)
        if (!modelShown_[i]) {
            const room::Vec3 ax = m.axis(0) * 0.02, ay = m.axis(1) * 0.015, az = m.axis(2) * 0.06;
            for (const room::Vec3 sgn : {ax, ax * -1.0}) {
                b.line(o + sgn + ay - az, o + sgn + ay + az, kBox);
                b.line(o + sgn - ay - az, o + sgn - ay + az, kBox);
            }
            b.line(o - ax + ay - az, o + ax + ay - az, kBox);
            b.line(o - ax - ay - az, o + ax - ay - az, kBox);
        }
        // the laser to where it points
        room::Vec3 end = o + dir * 3.0;
        if (panelHitWorld_[i]) {
            end = *panelHitWorld_[i];
        } else if (hits_[i]) {
            end = *hits_[i];
        }
        b.line(o, end, kBeam);
        // a small reticle facing along the beam at the hit
        room::Vec3 ra = dir.cross({0, 1, 0});
        if (ra.length() < 1e-4) {
            ra = dir.cross({1, 0, 0});
        }
        ra = ra.norm();
        const room::Vec3 rb = dir.cross(ra).norm();
        const double rr = 0.03;
        for (int sgi = 0; sgi < 16; sgi++) {
            const double a0 = sgi * kPi / 8, a1 = (sgi + 1) * kPi / 8;
            b.line(end + ra * (rr * std::cos(a0)) + rb * (rr * std::sin(a0)),
                   end + ra * (rr * std::cos(a1)) + rb * (rr * std::sin(a1)), kBeam);
        }
    }

    // the micro-window card: framed body, a header strip over the title, the text, then the button chips
    const auto ls = lines();
    const room::Vec3 pu = up_;
    const room::Vec3 away = right_.cross(pu) * -1.0;
    const double topV = 0.06 + kLineH * 0.9;
    double botV = 0.06 - kLineH * static_cast<double>(ls.size()) - 0.03;
    for (const auto& bt : buttons_) {
        botV = std::min(botV, bt.v - bt.hh - 0.025);
    }
    const double halfW = kPanelHalfW + 0.03;
    const room::Vec3 cbg = at_ + away * 0.012;
    const auto quadAt = [&](const room::Vec3& base, double u0, double u1, double v0, double v1, room::Rgba c) {
        b.quad(base + right_ * u0 + pu * v1, base + right_ * u1 + pu * v1, base + right_ * u1 + pu * v0,
               base + right_ * u0 + pu * v0, c);
    };
    const auto outline = [&](const room::Vec3& base, double u0, double u1, double v0, double v1, room::Rgba c) {
        const room::Vec3 a = base + right_ * u0 + pu * v0, d = base + right_ * u1 + pu * v0,
                         e = base + right_ * u1 + pu * v1, g = base + right_ * u0 + pu * v1;
        b.line(a, d, c);
        b.line(d, e, c);
        b.line(e, g, c);
        b.line(g, a, c);
    };
    quadAt(cbg, -halfW, halfW, botV, topV, kCard);
    outline(cbg, -halfW, halfW, botV, topV, kCardEdge);
    // header strip over the first line
    const room::Vec3 chdr = at_ + away * 0.011;
    const double hdrBot = 0.06 - kLineH * 0.55;
    quadAt(chdr, -halfW, halfW, hdrBot, topV, kHeader);
    b.line(chdr - right_ * halfW + pu * hdrBot, chdr + right_ * halfW + pu * hdrBot, kCardEdge.alpha(0.6F));
    double y = 0.06;
    for (std::size_t i = 0; i < ls.size(); i++) {
        room::drawTextCentered(b, ls[i], at_ + pu * y, right_, pu, i == 0 ? 0.006 : 0.0045,
                               i == 0 ? room::Rgba{0.55F, 1.0F, 0.75F, 1.0F} : kText);
        y -= kLineH;
    }
    // the button chips in front of the card
    const room::Vec3 cchip = at_ + away * 0.010;
    for (std::size_t i = 0; i < buttons_.size(); i++) {
        const PanelButton& bt = buttons_[i];
        const bool hov = static_cast<int>(i) == hover_;
        quadAt(cchip, bt.u - bt.hw, bt.u + bt.hw, bt.v - bt.hh, bt.v + bt.hh, hov ? kChipHover : kChip);
        outline(cchip, bt.u - bt.hw, bt.u + bt.hw, bt.v - bt.hh, bt.v + bt.hh, kChipEdge);
        room::drawTextCentered(b, bt.label, cchip + right_ * bt.u + pu * bt.v, right_, pu, 0.0035,
                               hov ? room::Rgba{0.02F, 0.05F, 0.08F, 1.0F} : kText);
    }
}

} // namespace pibridge::roomsetup
