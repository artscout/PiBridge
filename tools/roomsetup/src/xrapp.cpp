// SPDX-License-Identifier: MIT
// The Vulkan / OpenXR plumbing is tools/xrkit (shared with pibridge-home; it follows tools/eyecal/src/xrapp.cpp: session,
// swapchains, frame loop); the input bindings and the stage handling are ZoneTweak's xr.go (MIT, Copyright (c) 2026
// killdano; see NOTICE).
#include "xrapp.h"

#include "rendermodel.h"

#include "pibridge/room/draw.h"
#include "pibridge/room/scene.h"
#include "pibridge/room/vmath.h"
#include "pibridge/cam/slam.h"
#include "pibridge/xrkit/passthrough.h"
#include "pibridge/xrkit/xrkit.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <format>
#include <memory>
#include <optional>
#include <time.h>
#include <span>
#include <thread>
#include <vector>

namespace pibridge::roomsetup {

namespace {

using namespace std::chrono_literals;
using room::Batch;
using room::Mat34;
using xrkit::kIdentity;
using xrkit::kViewType;
using xrkit::locate;
using xrkit::toMat;
using xrkit::xr_make;

/// The editor's actions (they go with the instance).
struct Actions {
    XrAction grab = XR_NULL_HANDLE, del = XR_NULL_HANDLE, menu = XR_NULL_HANDLE, undo = XR_NULL_HANDLE,
             tool = XR_NULL_HANDLE, pose = XR_NULL_HANDLE, aim = XR_NULL_HANDLE, buzz = XR_NULL_HANDLE;
    // the thumbstick (trackpad on Vive) Y axis: nudges the floor up/down on the floor step (room setup rework)
    XrAction stick = XR_NULL_HANDLE;
};

/// ZoneTweak's bindings: per profile, for each action the left and (if different) right component.
std::expected<void, std::string> makeControls(xrkit::Session& x, Actions& c) {
    auto si = xr_make<XrActionSetCreateInfo>(XR_TYPE_ACTION_SET_CREATE_INFO);
    std::snprintf(si.actionSetName, sizeof si.actionSetName, "editor");
    std::snprintf(si.localizedActionSetName, sizeof si.localizedActionSetName, "Boundary editing");
    PIBRIDGE_XR(xrCreateActionSet(x.instance, &si, &x.set), "action set");
    x.hands[0].path = x.path("/user/hand/left");
    x.hands[1].path = x.path("/user/hand/right");
    const std::array<XrPath, 2> subs{x.hands[0].path, x.hands[1].path};
    struct A {
        const char* name;
        const char* label;
        XrActionType type;
        XrAction* out;
    };
    for (const A& a : {A{"grab", "Grab / place point", XR_ACTION_TYPE_BOOLEAN_INPUT, &c.grab},
                       A{"delete", "Delete point", XR_ACTION_TYPE_BOOLEAN_INPUT, &c.del},
                       A{"menu", "Menu", XR_ACTION_TYPE_BOOLEAN_INPUT, &c.menu},
                       A{"undo", "Undo", XR_ACTION_TYPE_BOOLEAN_INPUT, &c.undo},
                       A{"tool", "Switch tool mode", XR_ACTION_TYPE_BOOLEAN_INPUT, &c.tool},
                       A{"hand", "Hand pose", XR_ACTION_TYPE_POSE_INPUT, &c.pose},
                       A{"aim", "Pointer", XR_ACTION_TYPE_POSE_INPUT, &c.aim},
                       A{"stickaxis", "Thumbstick axis", XR_ACTION_TYPE_VECTOR2F_INPUT, &c.stick},
                       A{"buzz", "Haptics", XR_ACTION_TYPE_VIBRATION_OUTPUT, &c.buzz}}) {
        auto r = xrkit::makeAction(x.set, a.name, a.label, a.type, subs);
        if (!r) {
            return std::unexpected(r.error());
        }
        *a.out = *r;
    }
    struct B {
        XrAction a;
        const char* l;
        const char* r;
    };
    const std::vector<std::pair<const char*, std::vector<B>>> profiles{
        {"/interaction_profiles/valve/index_controller",
         {{c.grab, "trigger/click", ""}, {c.del, "a/click", ""}, {c.menu, "b/click", ""}, {c.undo, "thumbstick/click", ""},
          {c.tool, "trackpad/force", ""}, {c.pose, "grip/pose", ""}, {c.aim, "aim/pose", ""}, {c.stick, "thumbstick", ""},
          {c.buzz, "haptic", ""}}},
        {"/interaction_profiles/htc/vive_controller",
         {{c.grab, "trigger/click", ""}, {c.del, "squeeze/click", ""}, {c.menu, "menu/click", ""}, {c.undo, "trackpad/click", ""},
          {c.pose, "grip/pose", ""}, {c.aim, "aim/pose", ""}, {c.stick, "trackpad", ""}, {c.buzz, "haptic", ""}}},
        {"/interaction_profiles/oculus/touch_controller",
         {{c.grab, "trigger/value", ""}, {c.del, "x/click", "a/click"}, {c.menu, "y/click", "b/click"},
          {c.undo, "thumbstick/click", ""}, {c.tool, "squeeze/value", ""}, {c.pose, "grip/pose", ""}, {c.aim, "aim/pose", ""},
          {c.stick, "thumbstick", ""}, {c.buzz, "haptic", ""}}},
        {"/interaction_profiles/khr/simple_controller",
         {{c.grab, "select/click", ""}, {c.menu, "menu/click", ""}, {c.pose, "grip/pose", ""}, {c.aim, "aim/pose", ""},
          {c.buzz, "haptic", ""}}},
    };
    for (const auto& [prof, bs] : profiles) {
        for (const bool withTool : {true, false}) {
            std::vector<XrActionSuggestedBinding> sb;
            for (const B& b : bs) {
                if (!withTool && b.a == c.tool) {
                    continue;
                }
                for (int side = 0; side < 2; side++) {
                    const std::string comp = side == 1 && b.r[0] != 0 ? b.r : b.l;
                    const std::string kind = comp == "haptic" ? "output" : "input";
                    sb.push_back({b.a, x.path(std::string("/user/hand/") + (side == 0 ? "left" : "right") + "/" + kind + "/" + comp)});
                }
            }
            auto s = xr_make<XrInteractionProfileSuggestedBinding>(XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING);
            s.interactionProfile = x.path(prof);
            s.countSuggestedBindings = static_cast<std::uint32_t>(sb.size());
            s.suggestedBindings = sb.data();
            const XrResult r = xrSuggestInteractionProfileBindings(x.instance, &s);
            std::printf("bindings %s%s: %d\n", prof, withTool ? "" : " (without the tool button)", static_cast<int>(r));
            if (XR_SUCCEEDED(r)) {
                break;
            }
        }
    }
    return {};
}

std::expected<void, std::string> setup(xrkit::Session& x, Actions& c, const XrOpts& opts) {
    xrkit::OpenOpts o;
    o.appName = "pibridge-roomsetup";
    o.runtimeHint = "is SteamVR running and the active OpenXR runtime?";
    o.resDiv = opts.resDiv;
    if (auto r = xrkit::openSession(x, o); !r) {
        return r;
    }
    if (auto r = makeControls(x, c); !r) {
        return r;
    }
    auto att = xr_make<XrSessionActionSetsAttachInfo>(XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO);
    att.countActionSets = 1;
    att.actionSets = &x.set;
    PIBRIDGE_XR(xrAttachSessionActionSets(x.session, &att), "attach action sets");
    for (auto& h : x.hands) {
        XrActionSpaceCreateInfo as{XR_TYPE_ACTION_SPACE_CREATE_INFO, nullptr, c.pose, h.path, kIdentity};
        PIBRIDGE_XR(xrCreateActionSpace(x.session, &as, &h.grip), "hand space");
        as.action = c.aim;
        PIBRIDGE_XR(xrCreateActionSpace(x.session, &as, &h.aim), "aim space");
    }
    return xrkit::createSwapchains(x, o);
}

bool boolean(const xrkit::Session& x, XrAction a, XrPath sub) {
    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, a, sub};
    auto st = xr_make<XrActionStateBoolean>(XR_TYPE_ACTION_STATE_BOOLEAN);
    return XR_SUCCEEDED(xrGetActionStateBoolean(x.session, &gi, &st)) && st.isActive == XR_TRUE && st.currentState == XR_TRUE;
}

/// The thumbstick / trackpad Y of one hand (0 when the action is unbound or inactive).
float stickY(const xrkit::Session& x, XrAction a, XrPath sub) {
    if (a == XR_NULL_HANDLE) {
        return 0.0F;
    }
    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, a, sub};
    auto st = xr_make<XrActionStateVector2f>(XR_TYPE_ACTION_STATE_VECTOR2F);
    if (XR_SUCCEEDED(xrGetActionStateVector2f(x.session, &gi, &st)) && st.isActive == XR_TRUE) {
        return st.currentState.y;
    }
    return 0.0F;
}

} // namespace

namespace {

/// What the frame loop drives: the editor, or the recenter flow.
struct Loop {
    std::function<void(const room::FrameInput&)> frame;
    std::function<void(Batch&, double t)> build;
    std::function<bool()> quit;
    std::function<std::string()> status;
    std::function<void(const room::Rigid2&)> stageMoved;
    std::function<void(xrkit::Session&, const XrFrameState&)> pre; // after xrBeginFrame, before the input
    xrkit::PassthroughHook pt;                                       // the passthrough background per eye
};

std::expected<void, std::string> runLoop(xrkit::Session& x, Actions& c, const Loop& loop, const std::atomic<bool>& stop);

} // namespace

std::expected<void, std::string> runXr(const EditorFactory& make, const XrOpts& opts, const std::atomic<bool>& stop) {
    xrkit::Session x;
    Actions c;
    if (auto r = setup(x, c, opts); !r) {
        return std::unexpected(r.error());
    }
    const Haptic haptic = [&x, &c](int hand, int ms, float amp) {
        if (hand < 0 || hand > 1 || x.session == XR_NULL_HANDLE) {
            return;
        }
        XrHapticActionInfo hi{XR_TYPE_HAPTIC_ACTION_INFO, nullptr, c.buzz, x.hands[static_cast<std::size_t>(hand)].path};
        auto v = xr_make<XrHapticVibration>(XR_TYPE_HAPTIC_VIBRATION);
        v.duration = static_cast<XrDuration>(ms) * 1000000;
        v.frequency = XR_FREQUENCY_UNSPECIFIED;
        v.amplitude = amp;
        (void)xrApplyHapticFeedback(x.session, &hi, reinterpret_cast<const XrHapticBaseHeader*>(&v));
    };
    room::Editor* ed = make(x.runtimeName, haptic);
    if (ed == nullptr) {
        return std::unexpected(std::string("no room to edit"));
    }
    Loop loop;
    loop.frame = [ed](const room::FrameInput& in) { ed->frame(in); };
    loop.build = [ed](Batch& b, double t) { room::buildScene(*ed, b, t); };
    loop.quit = [ed] { return ed->quit; };
    loop.status = [ed] { return ed->status(); };
    loop.stageMoved = [ed](const room::Rigid2& r) { ed->stageMoved(r); };
    return runLoop(x, c, loop, stop);
}

std::expected<void, std::string> runRecenterXr(RecenterFlow& flow, const XrOpts& opts, const std::atomic<bool>& stop,
                                               const std::function<std::optional<RingSample>()>& readRing) {
    xrkit::Session x;
    Actions c;
    if (auto r = setup(x, c, opts); !r) {
        return std::unexpected(r.error());
    }
    Loop loop;
    loop.frame = [&flow, &readRing](const room::FrameInput& in) { flow.frame(in, readRing); };
    loop.build = [&flow](Batch& b, double) { flow.build(b); };
    loop.quit = [&flow] { return flow.finished(); };
    loop.status = [&flow] { return flow.lines().empty() ? std::string() : flow.lines().front(); };
    return runLoop(x, c, loop, stop);
}

namespace {

std::uint64_t monoNow() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

room::Mat34 shift(room::Vec3 t) {
    room::Mat34 m;
    m.m[0][3] = t.x;
    m.m[1][3] = t.y;
    m.m[2][3] = t.z;
    return m;
}

} // namespace

std::expected<void, std::string> runMapXr(MapFlow& flow, const MapLink& link, const XrOpts& opts, const std::atomic<bool>& stop) {
    xrkit::Session x;
    Actions c;
    if (auto r = setup(x, c, opts); !r) {
        return std::unexpected(r.error());
    }
    // the passthrough: the four cameras of the headset's own calibration (else the reference), from the broker
    std::optional<xrkit::pt::Rig> cams;
    room::Vec3 headInBody{};
    std::string note;
    if (link.camera != "off") {
        if (auto e = xrkit::enablePassthrough(x); !e) {
            note = "NO PASSTHROUGH: " + e.error();
        } else if (auto cal = cam::DeviceCalib::load_default(std::nullopt); !cal) {
            note = "NO PASSTHROUGH: NO CAMERA CALIBRATION";
        } else {
            const auto h = cam::slam::default_head_in_imu(cal->first);
            headInBody = {h[0], h[1], h[2]};
            if (auto p = xrkit::pt::makeRig(cal->first, headInBody)) {
                cams = *p;
                std::string names;
                for (std::size_t s = 0; s < p->n; s++) {
                    names += (s ? ", " : "") + std::to_string(p->cam[s].tile) + " " + p->cam[s].name;
                }
                std::printf("passthrough: cameras %s of %s; time %s\n", names.c_str(), cal->second.c_str(),
                            x.timespec ? "XR_KHR_convert_timespec_time" : "estimated from the frame times");
            } else {
                note = "NO PASSTHROUGH: " + p.error();
            }
        }
    } else {
        const auto cal = cam::DeviceCalib::load_default(std::nullopt);
        if (cal) {
            const auto h = cam::slam::default_head_in_imu(cal->first);
            headInBody = {h[0], h[1], h[2]};
        }
    }
    std::unique_ptr<xrkit::pt::Feed> feed;
    if (cams) {
        feed = std::make_unique<xrkit::pt::Feed>(link.camera, cams->tiles());
        feed->start();
    }
    flow.setNote(note);
    if (!note.empty()) {
        std::printf("%s\n", note.c_str());
    }

    StageLink stageLink;
    std::optional<guide::State> state;
    xrkit::pt::Frame frame;
    std::optional<room::Mat34> captureHead; // the head at the newest frame's capture time
    double gain = 1.0;
    std::array<double, xrkit::pt::kCams> camGain{1, 1, 1, 1};
    std::uint64_t lastRingTs = 0;
    std::optional<room::Mat34> prevRaw;
    std::int64_t xrMinusMono = 0; // the clock estimate without XR_KHR_convert_timespec_time
    bool haveOffset = false;
    unsigned frameNo = 0;
    std::optional<double> endedAt;
    const auto steady = [] { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); };
    const auto xrTimeOf = [&](const xrkit::Session& s, std::uint64_t mono) -> XrTime {
        if (const auto t = xrkit::xrTimeOfMonotonic(s, mono)) {
            return *t;
        }
        return static_cast<XrTime>(static_cast<std::int64_t>(mono) + xrMinusMono);
    };

    Loop loop;
    loop.pre = [&](xrkit::Session& s, const XrFrameState& fs) {
        frameNo++;
        if (!s.timespec) {
            // the display is about one frame period after xrWaitFrame returned
            const std::int64_t est = fs.predictedDisplayTime - static_cast<std::int64_t>(monoNow()) - fs.predictedDisplayPeriod;
            xrMinusMono = haveOffset ? xrMinusMono + (est - xrMinusMono) / 16 : est;
            haveOffset = true;
        }
        if (feed && feed->take(frame, frame.seq)) {
            s.gpu.pt.setFrame(frame);
            room::Mat34 m;
            if (locate(s.view, s.stage, xrTimeOf(s, frame.monoNs), m)) {
                captureHead = m;
            } else if (locate(s.view, s.stage, fs.predictedDisplayTime, m)) {
                captureHead = m; // no pose of that time: no time warp for this frame
            }
            gain = gain + (xrkit::pt::autoGain(frame.mean) - gain) * 0.2;
            const auto bal = xrkit::pt::balanceGains(frame);
            for (std::size_t k = 0; k < xrkit::pt::kCams; k++) {
                camGain[k] += (bal[k] - camGain[k]) * 0.2;
            }
        }
        if (feed && frameNo % 90 == 0) {
            const std::string e = feed->error();
            flow.setNote(e.empty() ? note : "NO CAMERA PICTURE YET: " + e.substr(0, 60));
        }
        // the stage link: the newest raw head of the ring and the stage head at its time
        if (frameNo % 3 == 0 && link.readRing) {
            if (const auto r = link.readRing(); r && r->tsNs != lastRingTs) {
                lastRingTs = r->tsNs;
                const auto& p = r->pose;
                const room::Mat34 raw =
                    room::mul34(room::poseMatrix(p[3], p[4], p[5], p[6], {p[0], p[1], p[2]}), shift(headInBody));
                room::Mat34 st;
                if (locate(s.view, s.stage, xrTimeOf(s, r->tsNs), st)) {
                    stageLink.add(st, raw);
                } else if (prevRaw && (raw.pos() - prevRaw->pos()).length() < 0.01 &&
                           std::fabs(headingOf(raw) - headingOf(*prevRaw)) < 0.02 &&
                           locate(s.view, s.stage, fs.predictedDisplayTime, st)) {
                    stageLink.add(st, raw); // no pose of the ring's time: only while the head is still
                }
                prevRaw = raw;
            }
        }
        if (link.readState) {
            if (auto st = link.readState()) {
                state = std::move(st);
            }
        }
        if (!endedAt && link.childExit) {
            if (const auto code = link.childExit()) {
                endedAt = steady();
                flow.childEnded(*code, link.childLastLine ? link.childLastLine() : std::string(), *endedAt);
            }
        }
    };
    loop.frame = [&](const room::FrameInput& in) {
        flow.frame(in, steady(), state, stageLink);
        for (const auto& a : flow.takeAnswers()) {
            if (link.writeAnswer) {
                link.writeAnswer(a);
            }
        }
    };
    // the real SteamVR controller render models, drawn over the box proxy when they load (room setup). The OpenVR
    // background client and the models degrade to no-op if SteamVR / the models are unavailable: then build() keeps
    // its box proxy. The grey the models are flat-shaded with.
    ControllerModels models;
    const room::Rgba kCtrlGrey{0.78F, 0.80F, 0.84F, 1.0F};
    loop.build = [&](Batch& b, double t) {
        models.poll();
        for (int h = 0; h < 2; h++) {
            flow.setModelShown(h, flow.handModelValid(h) && models.ready(h));
        }
        flow.build(b, t); // draws the lasers + the box proxy for any hand whose model is not shown
        for (int h = 0; h < 2; h++) {
            if (flow.handModelValid(h) && models.ready(h)) {
                models.draw(b, h, flow.handModelPose(h), kCtrlGrey);
            }
        }
    };
    loop.quit = [&] { return flow.finished(steady()); };
    loop.status = [&] {
        const auto l = flow.lines();
        return flow.step() + (l.empty() ? std::string() : ": " + l.front());
    };
    loop.pt = [&](std::size_t eye, const xrkit::pt::EyeView& v) -> std::optional<xrkit::pt::Uniform> {
        if (!cams || !captureHead) {
            return std::nullopt;
        }
        xrkit::pt::Look look;
        look.floorY = flow.floorStage();
        look.gain = gain;
        look.camGain = camGain;
        look.srgb = x.srgb();
        return xrkit::pt::makeUniform(static_cast<int>(eye), v, *cams, *captureHead, look);
    };
    auto r = runLoop(x, c, loop, stop);
    if (feed) {
        feed->stop();
    }
    return r;
}

namespace {

std::expected<void, std::string> runLoop(xrkit::Session& x, Actions& c, const Loop& loop, const std::atomic<bool>& stop) {
    bool running = false;
    std::optional<std::chrono::steady_clock::time_point> exitAt;
    std::string lastStatus;
    Batch batch;
    auto last = std::chrono::steady_clock::now();
    const auto requestExit = [&] {
        if (!exitAt) {
            (void)xrRequestExitSession(x.session);
            exitAt = std::chrono::steady_clock::now();
        }
    };
    while (true) {
        if (stop.load() || loop.quit()) {
            requestExit();
        }
        auto ev = xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        bool leave = false;
        while (xrPollEvent(x.instance, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                const auto& e = *reinterpret_cast<const XrEventDataSessionStateChanged*>(&ev);
                if (e.state == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO, nullptr, kViewType};
                    PIBRIDGE_XR(xrBeginSession(x.session, &bi), "xrBeginSession");
                    running = true;
                } else if (e.state == XR_SESSION_STATE_STOPPING) {
                    PIBRIDGE_XR(xrEndSession(x.session), "xrEndSession");
                    running = false;
                } else if (e.state == XR_SESSION_STATE_EXITING || e.state == XR_SESSION_STATE_LOSS_PENDING) {
                    leave = true;
                }
            } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
                leave = true;
            } else if (ev.type == XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING) {
                // a recentre (or SteamVR reloading a moved play centre): keep the boundary where it is in the room
                const auto& e = *reinterpret_cast<const XrEventDataReferenceSpaceChangePending*>(&ev);
                std::printf("reference space %d changed, pose valid %d\n", static_cast<int>(e.referenceSpaceType),
                            static_cast<int>(e.poseValid));
                if (e.poseValid == XR_TRUE && e.referenceSpaceType == x.stageType && loop.stageMoved) {
                    loop.stageMoved(room::rigidOfPose(toMat(e.poseInPreviousSpace)));
                }
            }
            ev = xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        }
        if (leave) {
            break;
        }
        if (!running) {
            if (exitAt && std::chrono::steady_clock::now() - *exitAt > 3s) {
                break;
            }
            std::this_thread::sleep_for(20ms);
            continue;
        }
        auto fwi = xr_make<XrFrameWaitInfo>(XR_TYPE_FRAME_WAIT_INFO);
        auto fs = xr_make<XrFrameState>(XR_TYPE_FRAME_STATE);
        PIBRIDGE_XR(xrWaitFrame(x.session, &fwi, &fs), "xrWaitFrame");
        auto fbi = xr_make<XrFrameBeginInfo>(XR_TYPE_FRAME_BEGIN_INFO);
        PIBRIDGE_XR(xrBeginFrame(x.session, &fbi), "xrBeginFrame");
        const auto nowTp = std::chrono::steady_clock::now();
        const double dt = std::clamp(std::chrono::duration<double>(nowTp - last).count(), 0.001, 0.1);
        last = nowTp;
        const XrTime t = fs.predictedDisplayTime;
        if (loop.pre) {
            loop.pre(x, fs);
        }

        // input
        std::array<XrActiveActionSet, 1> active{{{x.set, XR_NULL_PATH}}};
        auto sync = xr_make<XrActionsSyncInfo>(XR_TYPE_ACTIONS_SYNC_INFO);
        sync.countActiveActionSets = 1;
        sync.activeActionSets = active.data();
        (void)xrSyncActions(x.session, &sync);
        room::FrameInput in;
        in.dt = dt;
        in.headValid = locate(x.view, x.stage, t, in.head);
        for (std::size_t i = 0; i < 2; i++) {
            room::HandInput& h = in.hands[i];
            const XrPath sub = x.hands[i].path;
            h.valid = locate(x.hands[i].grip, x.stage, t, h.grip);
            h.gripPose = h.grip;       // keep the handle pose before the aim pose overwrites h.grip (controller model)
            h.gripPoseValid = h.valid;
            bool aimTracked = false;
            h.aimValid = locate(x.hands[i].aim, x.stage, t, h.aim, aimTracked);
            // The controller frame the editor draws and points with is the aim pose: its -Z is the pointing ray on every
            // runtime. OpenXR's grip -Z runs along the handle: on the Touch profile driver_monado gives the Pimax
            // controllers it is 60 deg above the ray, plus PIMAX_CTRL_ANGLE (30): the tip pointed straight up and the
            // box stood upright (live 2026-10-19). The grip is used only when there is no aim pose.
            if (h.aimValid) {
                h.grip = h.aim;
                h.valid = true;
                h.tracked = aimTracked;
            }
            const auto set = [&](XrAction a, unsigned b) {
                if (a != XR_NULL_HANDLE && boolean(x, a, sub)) {
                    h.buttons |= 1ULL << b;
                }
            };
            set(c.grab, room::bit::Trigger);
            set(c.del, room::bit::Grip);
            set(c.menu, room::bit::Menu);
            set(c.undo, room::bit::Stick);
            set(c.tool, room::bit::Tool);
            h.stickY = stickY(x, c.stick, sub);
        }
        loop.frame(in);
        if (const std::string st = loop.status(); st != lastStatus) {
            lastStatus = st;
            std::printf("%s\n", lastStatus.c_str());
            std::fflush(stdout);
        }
        batch.clear();
        loop.build(batch, std::chrono::duration<double>(nowTp.time_since_epoch()).count());

        // draw
        std::vector<XrCompositionLayerBaseHeader*> layers;
        xrkit::ProjectionLayer proj;
        if (fs.shouldRender == XR_TRUE) {
            auto drawn = xrkit::renderStereo(x, batch, t, 0.05, 60, proj, loop.pt);
            if (!drawn) {
                return std::unexpected(drawn.error());
            }
            if (*drawn) {
                layers.push_back(proj.header());
            }
        }
        auto fei = xr_make<XrFrameEndInfo>(XR_TYPE_FRAME_END_INFO);
        fei.displayTime = t;
        fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        fei.layerCount = static_cast<std::uint32_t>(layers.size());
        fei.layers = layers.data();
        PIBRIDGE_XR(xrEndFrame(x.session, &fei), "xrEndFrame");
    }
    return {};
}

} // namespace

} // namespace pibridge::roomsetup
