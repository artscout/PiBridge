// SPDX-License-Identifier: MIT
// Live poses for tracing a room, and the tracer.
//
// A PoseSource gives the head and the two controllers in stage space (the standing universe), with the trigger
// state, at a time. Implementations:
//   FakePoseSource   a scripted walk around a room (fake mode, tests, pibridge-roomsetup --selftest)
//   OpenXR           in pibridge-roomsetup (the editor's own input, tools/roomsetup/src/xrapp.cpp)
//   Monado           not written yet. Plan: libmonado (monado/src/xrt/targets/libmonado, the same IPC socket the
//                    monado-ctl tool uses) gives the device poses in Monado's tracking origin without an OpenXR
//                    session; with PiBridge's SLAM headset that origin is the SLAM map's, so it only makes sense once
//                    the map is anchored across sessions (see docs/ROOM-SETUP.md, "Inside-out").
// The desktop app has no live source of its own: the in-headset tool is where a room is traced.
//
// The tracer: hold the trigger (or press "Start trace" in a scripted run) and walk along the walls with the
// controller; the floor-projected controller positions are collected (one every 5 cm) and, when the trigger is let
// go, simplified into the boundary polygon (polygonFromTrace).
#pragma once

#include "pibridge/room/geom.h"
#include "pibridge/room/vmath.h"

#include <array>
#include <optional>
#include <vector>

namespace pibridge::room {

struct TrackedPose {
    bool valid = false;
    Vec3 pos;
    Vec3 forward{0, 0, -1}; // -z of the device
};

struct PoseSample {
    double t = 0; // seconds
    TrackedPose head;
    std::array<TrackedPose, 2> hands; // left, right
    std::array<bool, 2> trigger{};
};

class PoseSource {
public:
    virtual ~PoseSource() = default;
    virtual std::optional<PoseSample> sample(double t) = 0;
};

/// Walks the corners of `room` (counter-clockwise, `speed` m/s), the right controller held 0.3 m outside the head
/// path at 1.0 m height and its trigger held for the whole lap after `startDelay` seconds; the head at 1.7 m looks
/// along the walk. One lap, then the trigger is released and the walker stands still.
class FakePoseSource : public PoseSource {
public:
    FakePoseSource(Poly room, double speed = 0.8, double startDelay = 0.5);
    std::optional<PoseSample> sample(double t) override;
    double lapSeconds() const { return lap_ / speed_; }

private:
    Pt2 at(double s) const;
    Poly room_;
    double speed_;
    double delay_;
    double lap_ = 0;
};

class TraceRecorder {
public:
    /// Feed one floor point while the trace runs.
    void add(Pt2 p);
    void start() {
        path_.clear();
        running_ = true;
    }
    /// Stop and simplify; empty when the walk did not make a simple polygon.
    Poly finish(double tol = 0.08);
    bool running() const { return running_; }
    const Poly& path() const { return path_; }

private:
    Poly path_;
    bool running_ = false;
};

/// Follow a pose source with the right controller's trigger: start on press, finish on release.
struct TraceFollow {
    TraceRecorder rec;
    bool wasDown = false;
    /// The finished polygon on release (empty when it failed), nullopt while nothing finished this step.
    std::optional<Poly> step(const PoseSample& s, int hand = 1);
};

} // namespace pibridge::room
