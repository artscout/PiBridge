// SPDX-License-Identifier: MIT
#include "xrhome.h"

#include "backgrounds.h"
#include "loop.h"
#include "panel.h"
#include "plan.h"
#include "sky.h"

#include "pibridge/cam/slam.h"
#include "pibridge/xrkit/passthrough.h"
#include "pibridge/xrkit/xrkit.h"
#include "pibridge_vrpanel.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <future>
#include <memory>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace pibridge::home {

namespace {

using namespace std::chrono_literals;
using xrkit::xr_make;

struct Actions {
    XrAction grip = XR_NULL_HANDLE;
    XrAction aim = XR_NULL_HANDLE;
    XrAction trigger = XR_NULL_HANDLE; // float: the left mouse button on the PiBridge window
    XrAction stick = XR_NULL_HANDLE;   // vector2: y = the wheel
    XrAction squeeze = XR_NULL_HANDLE; // float: held 0.5 s = the window comes in front of you
};

XrState fromXr(XrSessionState s) {
    switch (s) {
    case XR_SESSION_STATE_IDLE: return XrState::Idle;
    case XR_SESSION_STATE_READY: return XrState::Ready;
    case XR_SESSION_STATE_SYNCHRONIZED: return XrState::Synchronized;
    case XR_SESSION_STATE_VISIBLE: return XrState::Visible;
    case XR_SESSION_STATE_FOCUSED: return XrState::Focused;
    case XR_SESSION_STATE_STOPPING: return XrState::Stopping;
    case XR_SESSION_STATE_LOSS_PENDING: return XrState::LossPending;
    case XR_SESSION_STATE_EXITING: return XrState::Exiting;
    default: return XrState::Unknown;
    }
}

/// Grip and aim poses, the trigger, the stick and the grip button: the Pimax controllers in Monado offer the Touch,
/// Index and simple controller profiles; Vive for other headsets (its trackpad as the stick).
std::expected<void, std::string> makeControls(xrkit::Session& x, Actions& c) {
    auto si = xr_make<XrActionSetCreateInfo>(XR_TYPE_ACTION_SET_CREATE_INFO);
    std::snprintf(si.actionSetName, sizeof si.actionSetName, "home");
    std::snprintf(si.localizedActionSetName, sizeof si.localizedActionSetName, "Home scene");
    PIBRIDGE_XR(xrCreateActionSet(x.instance, &si, &x.set), "action set");
    x.hands[0].path = x.path("/user/hand/left");
    x.hands[1].path = x.path("/user/hand/right");
    const std::array<XrPath, 2> subs{x.hands[0].path, x.hands[1].path};
    auto g = xrkit::makeAction(x.set, "hand", "Hand pose", XR_ACTION_TYPE_POSE_INPUT, subs);
    auto a = xrkit::makeAction(x.set, "aim", "Pointer", XR_ACTION_TYPE_POSE_INPUT, subs);
    auto tr = xrkit::makeAction(x.set, "click", "Click", XR_ACTION_TYPE_FLOAT_INPUT, subs);
    auto st = xrkit::makeAction(x.set, "scroll", "Scroll", XR_ACTION_TYPE_VECTOR2F_INPUT, subs);
    auto sq = xrkit::makeAction(x.set, "bring", "Bring the window in front", XR_ACTION_TYPE_FLOAT_INPUT, subs);
    if (!g || !a || !tr || !st || !sq) {
        return std::unexpected(!g ? g.error() : !a ? a.error() : !tr ? tr.error() : !st ? st.error() : sq.error());
    }
    c.grip = *g;
    c.aim = *a;
    c.trigger = *tr;
    c.stick = *st;
    c.squeeze = *sq;
    struct Prof {
        const char* path;
        const char* trigger;
        const char* stick; // nullptr: none
        const char* squeeze;
    };
    for (const Prof& prof : {Prof{"/interaction_profiles/oculus/touch_controller", "trigger/value", "thumbstick", "squeeze/value"},
                             Prof{"/interaction_profiles/valve/index_controller", "trigger/value", "thumbstick", "squeeze/value"},
                             Prof{"/interaction_profiles/khr/simple_controller", "select/click", nullptr, nullptr},
                             Prof{"/interaction_profiles/htc/vive_controller", "trigger/value", "trackpad", "squeeze/click"}}) {
        std::vector<XrActionSuggestedBinding> sb;
        for (const char* side : {"left", "right"}) {
            const std::string h = std::string("/user/hand/") + side + "/input/";
            sb.push_back({c.grip, x.path(h + "grip/pose")});
            sb.push_back({c.aim, x.path(h + "aim/pose")});
            sb.push_back({c.trigger, x.path(h + prof.trigger)});
            if (prof.stick != nullptr) {
                sb.push_back({c.stick, x.path(h + prof.stick)});
            }
            if (prof.squeeze != nullptr) {
                sb.push_back({c.squeeze, x.path(h + prof.squeeze)});
            }
        }
        const char* profPath = prof.path;
        auto s = xr_make<XrInteractionProfileSuggestedBinding>(XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING);
        s.interactionProfile = x.path(profPath);
        s.countSuggestedBindings = static_cast<std::uint32_t>(sb.size());
        s.suggestedBindings = sb.data();
        std::printf("bindings %s: %d\n", profPath, static_cast<int>(xrSuggestInteractionProfileBindings(x.instance, &s)));
    }
    auto att = xr_make<XrSessionActionSetsAttachInfo>(XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO);
    att.countActionSets = 1;
    att.actionSets = &x.set;
    PIBRIDGE_XR(xrAttachSessionActionSets(x.session, &att), "attach action sets");
    for (auto& h : x.hands) {
        XrActionSpaceCreateInfo as{XR_TYPE_ACTION_SPACE_CREATE_INFO, nullptr, c.grip, h.path, xrkit::kIdentity};
        PIBRIDGE_XR(xrCreateActionSpace(x.session, &as, &h.grip), "hand space");
        as.action = c.aim;
        PIBRIDGE_XR(xrCreateActionSpace(x.session, &as, &h.aim), "aim space");
    }
    return {};
}

double nowS() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

float actionFloat(XrSession s, XrAction a, XrPath sub) {
    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, a, sub};
    auto st = xr_make<XrActionStateFloat>(XR_TYPE_ACTION_STATE_FLOAT);
    if (XR_FAILED(xrGetActionStateFloat(s, &gi, &st)) || st.isActive != XR_TRUE) {
        return 0.0F;
    }
    return st.currentState;
}

XrVector2f actionVec2(XrSession s, XrAction a, XrPath sub) {
    XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO, nullptr, a, sub};
    auto st = xr_make<XrActionStateVector2f>(XR_TYPE_ACTION_STATE_VECTOR2F);
    if (XR_FAILED(xrGetActionStateVector2f(s, &gi, &st)) || st.isActive != XR_TRUE) {
        return {0.0F, 0.0F};
    }
    return st.currentState;
}

std::int64_t mtimeNs(const std::string& path) {
    struct stat st{};
    if (::stat(path.c_str(), &st) != 0) {
        return -1;
    }
    return static_cast<std::int64_t>(st.st_mtim.tv_sec) * 1000000000LL + st.st_mtim.tv_nsec;
}

std::uint64_t monoNow() { return vrpanel::monoNs(); }

} // namespace

std::expected<void, std::string> runHome(const RunOpts& opts, const std::atomic<bool>& stop) {
    xrkit::Session x;
    xrkit::OpenOpts o;
    o.appName = "pibridge-home";
    o.runtimeHint = "does the Tracking session run, and does XR_RUNTIME_JSON point at its Monado (or does SteamVR run)?";
    o.resDiv = opts.resDiv;
    o.maxEye = 2048; // the window's text reads well at this size and it keeps the GPU free for games
    Loop loop;
    World world;
    std::string error;
    Plan shown;
    std::string panelWords = "off";
    const auto writeStatus = [&] {
        auto w = statusFields(loop, ::getpid(), x.runtimeName, world.stage, world.roomName, error);
        w.str("background", backName(shown.back)).str("background_note", shown.note).str("window", panelWords);
        (void)w.write("home");
    };
    writeStatus();
    if (auto r = xrkit::openSession(x, o); !r) {
        error = r.error();
        writeStatus();
        return r;
    }
    Actions c;
    if (auto r = makeControls(x, c); !r) {
        error = r.error();
        writeStatus();
        return r;
    }
    if (auto r = xrkit::createSwapchains(x, o); !r) {
        error = r.error();
        writeStatus();
        return r;
    }
    x.gpu.background = {0.010F, 0.012F, 0.020F, 1.0F};
    world = worldFor(x.stageType == XR_REFERENCE_SPACE_TYPE_STAGE, opts.roomsDir);
    std::printf("home scene: %s space, room %s\n", world.stage ? "STAGE" : "LOCAL",
                world.room ? world.roomName.c_str() : "(none)");
    // the optional parts: each one that fails only falls back (plan.h), the home runs on
    std::string skyErr, panelErr, ptErr;
    if (auto r = xrkit::enableSky(x); !r) {
        skyErr = r.error();
        std::printf("no background pictures: %s\n", r.error().c_str());
    }
    if (auto r = xrkit::enablePanel(x); !r) {
        panelErr = r.error();
        std::printf("no PiBridge window: %s\n", r.error().c_str());
    }
    std::optional<xrkit::pt::Rig> rig;
    room::Vec3 headInBody{};
    if (opts.camera != "off") {
        if (auto e = xrkit::enablePassthrough(x); !e) {
            ptErr = e.error();
        } else if (auto cal = cam::DeviceCalib::load_default(std::nullopt); !cal) {
            ptErr = "no camera calibration";
        } else {
            const auto h = cam::slam::default_head_in_imu(cal->first);
            headInBody = {h[0], h[1], h[2]};
            if (auto p = xrkit::pt::makeRig(cal->first, headInBody)) {
                rig = *p;
                std::printf("passthrough: %zu cameras of %s\n", p->n, cal->second.c_str());
            } else {
                ptErr = p.error();
            }
        }
        if (!ptErr.empty()) {
            std::printf("no passthrough: %s\n", ptErr.c_str());
        }
    }
    std::fflush(stdout);
    writeStatus();

    // settings (pibridge.conf, re-read when it changes)
    const std::string confFile = bg::confPath().string();
    std::int64_t confStamp = -2;
    bg::HomeSettings settings;
    double lastConfCheck = 0;
    // the picture: loaded on a thread, uploaded once
    std::string pictureId;    // the one loaded (or loading)
    std::future<std::expected<sky::Pano, std::string>> loading;
    bool pictureLoaded = false, pictureFailed = false;
    enum class SkyUp { None, Dusk, Picture } skyUp = SkyUp::None;
    sky::Pano picture;
    const sky::Pano duskPano = sky::dusk(2048, 1024);
    // the passthrough feed
    std::unique_ptr<xrkit::pt::Feed> feed;
    xrkit::pt::Frame frame;
    std::optional<room::Mat34> captureHead;
    double gain = 1.0;
    std::array<double, xrkit::pt::kCams> camGain{1, 1, 1, 1};
    double lastCamFrame = -10;
    // the PiBridge window
    vrpanel::Link link;
    double linkTry = -10;
    std::uint64_t panelSeq = 0;
    std::vector<std::uint8_t> panelPix;
    std::uint32_t panelW = 0, panelH = 0;
    std::optional<win::Geometry> geom;
    win::Pointer pointer;
    std::size_t activeHand = 1;
    std::array<float, 2> lastTrig{0, 0};
    double squeezeSince = -1;
    double lastFrameS = nowS();

    Frame frameIn;
    std::optional<Panel> textPanel;
    room::Batch batch, overlay;
    std::string lastStatus = loop.status();
    double lastWrite = nowS();
    while (true) {
        if (stop.load() && loop.requestStop(nowS())) {
            (void)xrRequestExitSession(x.session); // STOPPING follows
        }
        auto ev = xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        bool leave = false;
        while (xrPollEvent(x.instance, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                const auto& e = *reinterpret_cast<const XrEventDataSessionStateChanged*>(&ev);
                const XrState st = fromXr(e.state);
                std::printf("session %s\n", stateName(st));
                std::fflush(stdout);
                switch (loop.onState(st)) {
                case Step::Begin: {
                    XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO, nullptr, xrkit::kViewType};
                    PIBRIDGE_XR(xrBeginSession(x.session, &bi), "xrBeginSession");
                    break;
                }
                case Step::End: PIBRIDGE_XR(xrEndSession(x.session), "xrEndSession"); break;
                case Step::Leave: leave = true; break;
                case Step::None: break;
                }
            } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
                loop.onInstanceLoss();
                leave = true;
            }
            ev = xr_make<XrEventDataBuffer>(XR_TYPE_EVENT_DATA_BUFFER);
        }
        const double t0 = nowS();
        if (loop.status() != lastStatus || t0 - lastWrite >= 2.0) {
            lastStatus = loop.status();
            lastWrite = t0;
            writeStatus();
        }
        if (leave || loop.shouldLeave(t0)) {
            break;
        }
        if (!loop.running()) {
            std::this_thread::sleep_for(20ms);
            continue;
        }
        // ---- settings, once a second
        if (t0 - lastConfCheck >= 1.0) {
            lastConfCheck = t0;
            if (const std::int64_t m = mtimeNs(confFile); m != confStamp) {
                confStamp = m;
                const bg::HomeSettings ns = bg::loadSettings(confFile);
                if (!(ns == settings)) {
                    std::printf("home: background %s, PiBridge window %s\n", bg::choiceValue(ns.background).c_str(),
                                ns.panel ? "on" : "off");
                    std::fflush(stdout);
                }
                settings = ns;
            }
        }
        // ---- the PiBridge window link (retried every 2 s while it cannot be opened)
        const std::uint64_t mono = monoNow();
        if (settings.panel && panelErr.empty() && !link.isOpen() && t0 - linkTry > 2.0) {
            linkTry = t0;
            if (!link.open()) {
                panelWords = "no link: " + link.error();
            }
        }
        const bool appLinked = link.isOpen() && vrpanel::fresh(link.header()->appBeatNs.load(), mono);
        // ---- the plan
        Have have;
        const bool wantPicture = settings.background.kind == bg::Kind::Picture;
        if (wantPicture) {
            have.pictureFile = bg::downloaded(bg::libraryDir(), settings.background.id);
            have.pictureLoaded = pictureLoaded && pictureId == settings.background.id;
            have.pictureFailed = pictureFailed && pictureId == settings.background.id;
        }
        have.cameras = rig.has_value();
        have.cameraFrames = t0 - lastCamFrame < 2.0;
        if (!skyErr.empty()) {
            // no sky pass: the dark grid instead of a picture
            have.pictureFile = false;
        }
        Plan p = plan(settings, have);
        if (!skyErr.empty() && (p.back == Back::Dusk || p.back == Back::Picture)) {
            p.back = Back::Grid;
            p.grid = true;
        }
        if (!(p.back == shown.back && p.note == shown.note && p.window == shown.window)) {
            std::printf("home shows: %s%s%s\n", backName(p.back), p.window ? ", the PiBridge window" : "",
                        p.note.empty() ? "" : (" (" + p.note + ")").c_str());
            std::fflush(stdout);
        }
        shown = p;
        world.grid = p.grid;
        panelWords = !settings.panel ? "off" : !panelErr.empty() ? "no GPU pass" : appLinked ? "linked" : "waiting for the app";
        // the picture loads on a thread; the dusk sky stays meanwhile
        if (p.loadPicture && pictureId != settings.background.id) {
            pictureId = settings.background.id;
            pictureLoaded = pictureFailed = false;
            const std::string path = bg::picturePath(bg::libraryDir(), pictureId).string();
            loading = std::async(std::launch::async, [path] { return sky::loadPpm(path, 4096); });
        }
        if (loading.valid() && loading.wait_for(0s) == std::future_status::ready) {
            auto r = loading.get();
            if (r) {
                picture = std::move(*r);
                pictureLoaded = true;
                skyUp = SkyUp::None; // upload it
            } else {
                pictureFailed = true;
                std::printf("background: %s\n", r.error().c_str());
            }
        }
        // the camera feed runs only while the passthrough is the background (nothing reads the camera otherwise)
        if (p.runFeed && !feed && rig) {
            feed = std::make_unique<xrkit::pt::Feed>(opts.camera, rig->tiles());
            feed->start();
        } else if (!p.runFeed && feed) {
            feed->stop();
            feed.reset();
            lastCamFrame = -10;
        }

        // every frame the runtime asks for, also while another app is shown (then nothing is drawn or submitted)
        auto fwi = xr_make<XrFrameWaitInfo>(XR_TYPE_FRAME_WAIT_INFO);
        auto fs = xr_make<XrFrameState>(XR_TYPE_FRAME_STATE);
        PIBRIDGE_XR(xrWaitFrame(x.session, &fwi, &fs), "xrWaitFrame");
        auto fbi = xr_make<XrFrameBeginInfo>(XR_TYPE_FRAME_BEGIN_INFO);
        PIBRIDGE_XR(xrBeginFrame(x.session, &fbi), "xrBeginFrame");
        const XrTime t = fs.predictedDisplayTime;
        std::vector<XrCompositionLayerBaseHeader*> layers;
        xrkit::ProjectionLayer proj;
        const bool drawNow = loop.submit(fs.shouldRender == XR_TRUE);
        // the app renders its window only while the home shows it
        if (link.isOpen()) {
            link.header()->homePid.store(static_cast<std::uint32_t>(::getpid()));
            if (drawNow && settings.panel && panelErr.empty()) {
                link.header()->homeBeatNs.store(mono);
            }
        }
        if (drawNow) {
            const double dt = std::clamp(t0 - lastFrameS, 0.0, 0.1);
            lastFrameS = t0;
            if (loop.focused()) {
                std::array<XrActiveActionSet, 1> active{{{x.set, XR_NULL_PATH}}};
                auto sync = xr_make<XrActionsSyncInfo>(XR_TYPE_ACTIONS_SYNC_INFO);
                sync.countActiveActionSets = 1;
                sync.activeActionSets = active.data();
                (void)xrSyncActions(x.session, &sync);
            }
            room::Mat34 m;
            frameIn.head = xrkit::locate(x.view, x.stage, t, m) ? std::optional(m) : std::nullopt;
            for (std::size_t i = 0; i < 2; i++) {
                frameIn.grip[i] = loop.focused() && xrkit::locate(x.hands[i].grip, x.stage, t, m) ? std::optional(m) : std::nullopt;
                frameIn.aim[i] = loop.focused() && xrkit::locate(x.hands[i].aim, x.stage, t, m) ? std::optional(m) : std::nullopt;
            }
            if (!textPanel && frameIn.head) {
                textPanel = placePanel(*frameIn.head, world.floorY);
            }
            frameIn.panel = p.textPanel ? textPanel : std::nullopt;
            // ---- the background
            if (p.back == Back::Passthrough || p.runFeed) {
                if (feed && feed->take(frame, frame.seq)) {
                    x.gpu.pt.setFrame(frame);
                    lastCamFrame = t0;
                    std::optional<XrTime> ct = xrkit::xrTimeOfMonotonic(x, frame.monoNs);
                    if (ct && xrkit::locate(x.view, x.stage, *ct, m)) {
                        captureHead = m;
                    } else if (xrkit::locate(x.view, x.stage, t, m)) {
                        captureHead = m; // no pose of that time: no time warp for this frame
                    }
                    gain += (xrkit::pt::autoGain(frame.mean) - gain) * 0.2;
                    const auto bal = xrkit::pt::balanceGains(frame);
                    for (std::size_t k = 0; k < xrkit::pt::kCams; k++) {
                        camGain[k] += (bal[k] - camGain[k]) * 0.2;
                    }
                }
            }
            if (skyErr.empty()) {
                if (p.back == Back::Picture && skyUp != SkyUp::Picture && pictureLoaded) {
                    const auto rgba = sky::toRgba(picture);
                    if (auto r = x.gpu.sky.setPixels(rgba.data(), static_cast<std::uint32_t>(picture.w),
                                                     static_cast<std::uint32_t>(picture.h), static_cast<std::size_t>(picture.w) * 4);
                        r) {
                        skyUp = SkyUp::Picture;
                    } else {
                        pictureFailed = true;
                        std::printf("background upload: %s\n", r.error().c_str());
                    }
                } else if (p.back == Back::Dusk && skyUp != SkyUp::Dusk) {
                    const auto rgba = sky::toRgba(duskPano);
                    if (x.gpu.sky.setPixels(rgba.data(), static_cast<std::uint32_t>(duskPano.w), static_cast<std::uint32_t>(duskPano.h),
                                            static_cast<std::size_t>(duskPano.w) * 4)) {
                        skyUp = SkyUp::Dusk;
                    }
                }
            }
            // ---- the PiBridge window: the newest picture, the pointer
            overlay.clear();
            if (p.window) {
                std::uint32_t w = 0, h = 0;
                if (const auto sq = link.readFrame(panelSeq, panelPix, w, h)) {
                    panelSeq = *sq;
                    if (auto r = x.gpu.panel.setPixels(panelPix.data(), w, h, static_cast<std::size_t>(w) * 4); r) {
                        if (w != panelW || h != panelH || !geom) {
                            panelW = w;
                            panelH = h;
                            if (!geom && frameIn.head) {
                                geom = win::place(*frameIn.head, world.floorY, w, h);
                            } else if (geom) {
                                win::fitAspect(*geom, w, h);
                            }
                            if (geom) {
                                const auto mv = win::mesh(*geom);
                                std::vector<xrkit::TexVertex> tv(mv.size());
                                std::ranges::transform(mv, tv.begin(), [](const win::MeshVertex& v) {
                                    return xrkit::TexVertex{v.x, v.y, v.z, v.u, v.v};
                                });
                                x.gpu.panel.setMesh(tv);
                            }
                        }
                    }
                }
                // the hand whose trigger moved last points; the grip held 0.5 s brings the window in front of you
                std::array<float, 2> trig{0, 0};
                float sqz = 0;
                if (loop.focused()) {
                    for (std::size_t i = 0; i < 2; i++) {
                        trig[i] = actionFloat(x.session, c.trigger, x.hands[i].path);
                        sqz = std::max(sqz, actionFloat(x.session, c.squeeze, x.hands[i].path));
                        if (trig[i] > 0.3F && lastTrig[i] <= 0.3F && !pointer.pressed()) {
                            activeHand = i;
                        }
                    }
                }
                lastTrig = trig;
                if (sqz > 0.8F) {
                    if (squeezeSince < 0) {
                        squeezeSince = t0;
                    } else if (t0 - squeezeSince > 0.5 && frameIn.head && panelW > 0) {
                        geom = win::place(*frameIn.head, world.floorY, panelW, panelH);
                        const auto mv = win::mesh(*geom);
                        std::vector<xrkit::TexVertex> tv(mv.size());
                        std::ranges::transform(mv, tv.begin(), [](const win::MeshVertex& v) {
                            return xrkit::TexVertex{v.x, v.y, v.z, v.u, v.v};
                        });
                        x.gpu.panel.setMesh(tv);
                        squeezeSince = 1e18; // once per hold
                    }
                } else {
                    squeezeSince = -1;
                }
                std::optional<win::Hit> hh;
                const auto& aim = frameIn.aim[activeHand];
                if (geom && aim) {
                    const room::Vec3 o2 = aim->pos();
                    const room::Vec3 d2 = (aim->axis(2) * -1.0).norm();
                    hh = win::hit(*geom, o2, d2);
                    win::addRay(overlay, o2, d2, hh, *geom, pointer.pressed());
                }
                const XrVector2f stick = loop.focused() ? actionVec2(x.session, c.stick, x.hands[activeHand].path) : XrVector2f{0, 0};
                pointer.update(hh, trig[activeHand], stick.y, dt, panelW, panelH);
                if (link.isOpen()) {
                    link.header()->pointer.store(vrpanel::packPointer(pointer.state()));
                    link.header()->wheel.store(pointer.wheel());
                }
            } else if (link.isOpen()) {
                link.header()->pointer.store(0);
            }
            buildScene(batch, world, frameIn);
            // ---- draw
            xrkit::Extras extras;
            const bool srgb = x.srgb();
            if (p.back == Back::Dusk || p.back == Back::Picture) {
                extras.sky = [&](std::size_t, const xrkit::pt::EyeView& v) -> std::optional<xrkit::SkyPush> {
                    xrkit::SkyPush sp;
                    sp.stageFromEye = room::mat4From34(v.stageFromEye);
                    sp.tan = {static_cast<float>(v.tanL), static_cast<float>(v.tanR), static_cast<float>(v.tanU),
                              static_cast<float>(v.tanD)};
                    sp.misc = {srgb ? 1.0F : 0.0F, 0.0F, 1.0F, 0.0F};
                    return sp;
                };
            }
            extras.panel = p.window && geom.has_value() && panelSeq != 0;
            extras.overlay = &overlay;
            xrkit::PassthroughHook ptHook;
            if (p.back == Back::Passthrough && rig) {
                ptHook = [&](std::size_t eye, const xrkit::pt::EyeView& v) -> std::optional<xrkit::pt::Uniform> {
                    if (!captureHead) {
                        return std::nullopt;
                    }
                    xrkit::pt::Look look;
                    look.floorY = world.floorY;
                    look.gain = gain;
                    look.camGain = camGain;
                    look.srgb = srgb;
                    if (world.room && world.stage) {
                        look.room = xrkit::pt::RoomShape{*world.room, world.floorY + world.roomHeight};
                    }
                    return xrkit::pt::makeUniform(static_cast<int>(eye), v, *rig, *captureHead, look);
                };
            }
            auto drawn = xrkit::renderStereo(x, batch, t, 0.05, kFarZ, proj, ptHook, &extras);
            if (!drawn) {
                error = drawn.error();
                writeStatus();
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
    if (feed) {
        feed->stop();
    }
    if (link.isOpen()) {
        link.header()->homeBeatNs.store(0);
        link.header()->pointer.store(0);
    }
    writeStatus();
    return {};
}

} // namespace pibridge::home
