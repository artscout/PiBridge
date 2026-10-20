// SPDX-License-Identifier: MIT
#include "selftest.h"

#include "backgrounds.h"
#include "loop.h"
#include "panel.h"
#include "plan.h"
#include "scene.h"
#include "sky.h"

#include "pibridge/xrkit/ptsynth.h"
#include "pibridge_vrpanel.h"

#include "pibridge/room/raster.h"
#include "pibridge/room/rooms.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <numbers>
#include <string>
#include <unistd.h>

namespace pibridge::home {

namespace {

namespace fs = std::filesystem;
using room::Vec3;

int g_failed = 0;

void step(const char* what, bool ok, const std::string& detail = {}) {
    std::printf("%s %s%s%s\n", ok ? "ok  " : "FAIL", what, detail.empty() ? "" : ": ", detail.c_str());
    if (!ok) {
        g_failed++;
    }
}

bool near(double a, double b, double eps = 1e-9) { return std::abs(a - b) < eps; }

// every line vertex at height y?
bool allLinesAt(const room::Batch& b, std::size_t from, std::size_t to, double y) {
    for (std::size_t i = from; i < to; i++) {
        if (!near(b.lines[i].y, y, 1e-5)) {
            return false;
        }
    }
    return true;
}

room::Mat34 headAt(Vec3 p, double yaw) {
    // a head turned by `yaw` about +y (OpenXR: looking along -z at yaw 0)
    return room::poseMatrix(0, std::sin(yaw / 2), 0, std::cos(yaw / 2), p);
}

} // namespace

int runSelfTest(const std::string& snapshotDir, const std::string& panelPpm) {
    g_failed = 0;
    std::error_code ec;
    const fs::path tmp = fs::temp_directory_path(ec) / ("pibridge-home-selftest-" + std::to_string(::getpid()));
    fs::remove_all(tmp, ec);
    fs::create_directories(tmp / "rooms", ec);

    // ---- the scene without a room: grid on the floor, the horizon at eye height
    World w = worldFor(true, (tmp / "rooms").string());
    step("no saved room: no boundary", !w.room.has_value());
    room::Batch b;
    const std::size_t grid = addGrid(b, w);
    step("floor grid on the floor (STAGE, y = 0)", grid > 1000 && allLinesAt(b, 0, b.lines.size(), 0.0),
         std::to_string(grid) + " lines");
    const std::size_t h0 = b.lines.size();
    const std::size_t hz = addHorizon(b, w);
    step("horizon ring at eye height", hz == 128 && allLinesAt(b, h0, b.lines.size(), kEyeHeight));
    const double r = std::hypot(b.lines[h0].x, b.lines[h0].z);
    step("horizon inside the far plane", near(r, kHorizonRadius, 1e-6) && kHorizonRadius < kFarZ);
    // the grid fades out: the outermost pieces are (nearly) invisible
    float edgeAlpha = 1.0F;
    for (std::size_t i = 0; i < h0; i++) {
        if (std::hypot(b.lines[i].x, b.lines[i].z) > kGridRadius - 0.01) {
            edgeAlpha = std::min(edgeAlpha, b.lines[i].a);
        }
    }
    step("grid fades out at its edge", edgeAlpha < 0.01F);

    // ---- LOCAL fallback: the floor about 1.6 m below the start
    World local = worldFor(false, (tmp / "rooms").string());
    room::Batch bl;
    addGrid(bl, local);
    step("LOCAL space: grid 1.6 m below the start", near(local.floorY, kLocalFloorY) &&
                                                       allLinesAt(bl, 0, bl.lines.size(), kLocalFloorY));

    // ---- the active saved room, raw numbers through its standing pose (frame mode A)
    room::RoomLib lib((tmp / "rooms").string());
    room::Room rm;
    rm.name = "Office";
    rm.universe = "1111111111";
    rm.height = 2.5;
    rm.points = {{1, 2}, {5, 2}, {5, 5}, {1, 5}};
    rm.standing = room::Standing{3, 0, 3.5, std::numbers::pi / 2};
    const auto saved = lib.save(rm, std::time(nullptr));
    lib.setActive("Office");
    lib.setFrameMode(room::FrameMode::A);
    w = worldFor(true, (tmp / "rooms").string());
    step("saved room found", saved.has_value() && w.room.has_value() && w.roomName == "Office");
    if (w.room) {
        // (1, 2) - (3, 3.5) = (-2, -1.5), turned by 90 deg: (1.5, -2)
        const room::Pt2 p = w.room->front();
        step("room in stage numbers (standing pose applied)", near(p.x, 1.5, 1e-9) && near(p.z, -2, 1e-9),
             std::to_string(p.x) + ", " + std::to_string(p.z));
    }
    const std::size_t rl = addRoom(b, w);
    step("room boundary: floor, top and corner lines", rl == 12);
    room::Room flat = rm;
    flat.frame = "stage";
    step("a room saved in stage numbers is used as is", roomOutline(flat, room::FrameMode::A)->front() == room::Pt2{1, 2});
    step("frame mode Off: the file numbers as they are", roomOutline(rm, room::FrameMode::Off)->front() == room::Pt2{1, 2});
    room::Room tiny = rm;
    tiny.points.resize(2);
    step("fewer than three points: no boundary", !roomOutline(tiny, room::FrameMode::A).has_value());

    // ---- the panel in front of the head, facing it; the controllers
    const room::Mat34 head = headAt({0.2, 1.7, 0.4}, std::numbers::pi / 2); // turned left: looks along -x
    const Panel pn = placePanel(head, 0.0);
    step("panel 1.8 m in front of the head", near(pn.center.x, 0.2 - 1.8, 1e-6) && near(pn.center.z, 0.4, 1e-6) &&
                                                 near(pn.center.y, 1.6, 1e-9));
    const Vec3 normal = pn.right.cross(pn.up);
    step("panel faces the head", normal.dot(head.pos() - pn.center) > 0);
    const Panel down = placePanel(headAt({0, 0.3, 0}, 0), 0.0);
    step("panel never lower than 0.9 m above the floor", near(down.center.y, 0.9, 1e-9));
    Frame f;
    f.head = head;
    f.panel = pn;
    f.grip[0] = room::poseMatrix(0, 0, 0, 1, {-0.2, 1.1, -0.3});
    f.aim[0] = f.grip[0];
    const std::size_t cl = addControllers(b, f);
    step("controller marker and pointer (one hand)", cl == 4);
    room::Batch pb;
    const std::size_t pl = addPanel(pb, pn);
    step("panel with text", pl == 4 && pb.tris.size() > 300, std::to_string(pb.tris.size() / 3) + " triangles");
    room::Batch full;
    buildScene(full, w, f);
    step("whole scene is small", full.lines.size() / 2 < 3000 && full.tris.size() / 3 < 5000,
         std::to_string(full.lines.size() / 2) + " lines, " + std::to_string(full.tris.size() / 3) + " triangles");

    // ---- the session rules: begin once, yield to a game, never take the headset back
    Loop l;
    step("READY: begin", l.onState(XrState::Ready) == Step::Begin && l.status() == "running");
    step("SYNCHRONIZED (another app is shown): nothing submitted",
         l.onState(XrState::Synchronized) == Step::None && !l.submit(true));
    l.onState(XrState::Visible);
    step("VISIBLE: drawn", l.submit(true) && !l.submit(false) && l.status() == "visible");
    l.onState(XrState::Focused);
    step("FOCUSED", l.focused() && l.status() == "focused");
    l.onState(XrState::Synchronized); // a game started: Monado shows it
    step("a game took over: hidden, still running", l.running() && !l.visible() && !l.submit(true));
    step("a second READY never begins again", l.onState(XrState::Ready) == Step::None);
    step("STOPPING: end, then leave", l.onState(XrState::Stopping) == Step::End && l.shouldLeave(0));
    Loop s;
    s.onState(XrState::Ready);
    step("stop asked while running: request the exit once", s.requestStop(10.0) && !s.requestStop(10.5));
    step("waits for the runtime ...", !s.shouldLeave(11.0));
    step("... at most 3 s", s.shouldLeave(10.0 + kExitWaitS));
    Loop e;
    e.onState(XrState::Ready);
    step("EXITING / LOSS_PENDING: leave", e.onState(XrState::Exiting) == Step::Leave && e.shouldLeave(0));
    Loop n;
    step("stop before a session: leave at once", !n.requestStop(1.0) && n.shouldLeave(1.0));
    step("instance loss: leave", Loop{}.onInstanceLoss() == Step::Leave);
    const std::string js = statusFields(l, 4242, "Monado", true, "Office", "").text();
    step("status file", js.find("\"state\": \"ending\"") != std::string::npos &&
                            js.find("\"pid\": 4242") != std::string::npos);

    // ---- backgrounds: the choice, the list, the settings
    step("background choice: passthrough by default", bg::parseChoice("").kind == bg::Kind::Passthrough &&
                                                          bg::parseChoice("bogus").kind == bg::Kind::Passthrough);
    step("background choice: a listed picture", bg::parseChoice("sunflowers").kind == bg::Kind::Picture &&
                                                     bg::choiceValue(bg::parseChoice("dusk")) == "dusk");
    step("the list is CC0 only", std::ranges::all_of(bg::catalogue(), [](const bg::Picture& p) { return p.licence == "CC0"; }));
    const auto hs = bg::parseSettings("# x\nhome_background = venice_sunset\nhome_panel=off\n");
    step("settings from pibridge.conf", hs.background.id == "venice_sunset" && !hs.panel);
    // ---- what is shown (plan.h)
    {
        bg::HomeSettings st;
        Have hv;
        step("passthrough without cameras: the dusk sky, said why", plan(st, hv).back == Back::Dusk && !plan(st, hv).note.empty());
        hv.cameras = true;
        hv.cameraFrames = true;
        step("passthrough with frames", plan(st, hv).back == Back::Passthrough && plan(st, hv).runFeed);
        st.background = bg::parseChoice("sunflowers");
        step("a picture not downloaded: the dusk sky", plan(st, hv).back == Back::Dusk && !plan(st, hv).loadPicture);
        hv.pictureFile = true;
        step("a picture loading: dusk meanwhile", plan(st, hv).loadPicture && plan(st, hv).back == Back::Dusk);
        hv.pictureLoaded = true;
        step("a picture loaded", plan(st, hv).back == Back::Picture && !plan(st, hv).runFeed && !plan(st, hv).grid);
        step("no app: the text panel", plan(st, hv).textPanel && !plan(st, hv).window);
        hv.appLinked = true;
        step("the app linked: its window", plan(st, hv).window && !plan(st, hv).textPanel);
        st.panel = false;
        step("window off in the settings", !plan(st, hv).window);
    }
    // ---- the sky
    const sky::Pano dusk = sky::dusk(512, 256);
    {
        const auto up = sky::sample(dusk, {0, 1, 0});
        const auto ahead = sky::sample(dusk, {0, 0.05, -1});
        const auto below = sky::sample(dusk, {0, -1, 0});
        step("dusk: brighter at the horizon than at the zenith, dark below",
             ahead[0] + ahead[1] + ahead[2] > up[0] + up[1] + up[2] && below[0] + below[1] + below[2] < ahead[0] + ahead[1] + ahead[2]);
        double s1 = 0, t1 = 0;
        sky::dirToSt({0, 0, -1}, 0, s1, t1);
        double s2 = 0, t2 = 0;
        sky::dirToSt({1, 0, 0}, 0, s2, t2);
        step("equirect: ahead is the middle, right is a quarter on", near(s1, 0.5) && near(t1, 0.5) && near(s2, 0.75));
    }
    // ---- the PiBridge window: placement, hit, pointer, link
    const room::Mat34 wh = headAt({0.2, 1.65, 0.5}, 0.3);
    const win::Geometry g = win::place(wh, 0.0, 1500, 920);
    {
        const Vec3 fwd = (wh.axis(2) * -1.0).norm();
        const auto c = win::hit(g, wh.pos(), fwd);
        step("the window is in front of the head", c && std::abs(c->u - 0.5) < 0.02 && c->t > 1.1 && c->t < 1.4,
             c ? std::to_string(c->u) + " " + std::to_string(c->v) : "no hit");
        step("its height follows the window's aspect", near(g.height, 1.5 * 920.0 / 1500.0, 1e-9));
        const auto corner = win::hit(g, wh.pos(), (g.at(0.02, 0.03) - wh.pos()).norm());
        step("a corner maps back to its coordinates", corner && std::abs(corner->u - 0.02) < 1e-6 && std::abs(corner->v - 0.03) < 1e-6);
        step("looking away misses it", !win::hit(g, wh.pos(), fwd * -1.0));
        step("mesh: 2 triangles per segment", win::mesh(g).size() == static_cast<std::size_t>(win::kSegments) * 6);
        win::Pointer ptr;
        ptr.update(c, 0.0, 0.0, 0.011, 1500, 920);
        const bool hover = ptr.state().on && ptr.state().buttons == 0 && std::abs(static_cast<int>(ptr.state().x) - 750) < 30;
        ptr.update(c, 0.9, 0.0, 0.011, 1500, 920);
        const bool pressedNow = ptr.pressed() && ptr.state().buttons == 1;
        ptr.update(std::nullopt, 0.5, 0.0, 0.011, 1500, 920);
        const bool drag = ptr.pressed() && ptr.state().on; // hysteresis, and the drag keeps its place
        ptr.update(c, 0.1, 0.0, 0.011, 1500, 920);
        const bool up = !ptr.pressed() && ptr.state().buttons == 0;
        for (int i = 0; i < 90; i++) {
            ptr.update(c, 0.0, 1.0, 1.0 / 90, 1500, 920);
        }
        step("pointer: hover, press, drag, release", hover && pressedNow && drag && up);
        step("stick: 8 notches a second", std::abs(ptr.wheel() - 960) <= 2, std::to_string(ptr.wheel()));
        const vrpanel::Pointer pp{true, 1499, 919, 1};
        step("pointer packing", vrpanel::unpackPointer(vrpanel::packPointer(pp)) == pp &&
                                    !vrpanel::unpackPointer(vrpanel::packPointer(vrpanel::Pointer{})).on);
        vrpanel::Link app, home;
        const std::string lp = (tmp / "vrpanel.shm").string();
        const bool opened = app.open(lp) && home.open(lp);
        const auto card = win::testCard(64, 40);
        std::vector<std::uint8_t> got;
        std::uint32_t gw = 0, gh = 0;
        const bool none = opened && !home.readFrame(0, got, gw, gh);
        const bool wrote = opened && app.writeFrame(card.data(), 64, 40, 64 * 4);
        const auto seq = opened ? home.readFrame(0, got, gw, gh) : std::nullopt;
        step("link: a picture goes across whole", none && wrote && seq && gw == 64 && gh == 40 && got == card &&
                                                     !home.readFrame(*seq, got, gw, gh));
        step("link: too big is refused", opened && !app.writeFrame(card.data(), vrpanel::kMaxW + 1, 1, 4));
        home.header()->homeBeatNs.store(vrpanel::monoNs());
        step("link: the beat is fresh, then stale", vrpanel::fresh(app.header()->homeBeatNs.load(), vrpanel::monoNs()) &&
                                                       !vrpanel::fresh(app.header()->homeBeatNs.load(),
                                                                       vrpanel::monoNs() + vrpanel::kBeatTimeoutNs + 1));
    }

    // ---- the pictures (software renderer)
    if (!snapshotDir.empty()) {
        fs::create_directories(snapshotDir, ec);
        Frame sf;
        const room::Mat34 sh = headAt({0.0, 1.7, 1.0}, 0.15);
        sf.head = sh;
        sf.panel = placePanel(sh, 0.0);
        sf.grip[0] = room::poseMatrix(0, 0, 0, 1, {-0.25, 1.15, 0.45});
        sf.aim[0] = sf.grip[0];
        sf.grip[1] = room::poseMatrix(0, 0.2, 0, 0.98, {0.3, 1.1, 0.5});
        sf.aim[1] = sf.grip[1];
        room::Batch sb;
        buildScene(sb, w, sf);
        const room::Mat4 view = room::lookAt({0.0, 1.7, 1.6}, {0.0, 1.3, -1.0});
        const room::Mat4 proj = room::projectionVulkan(-0.9, 0.9, 0.7, -0.7, 0.05, kFarZ);
        const room::Image img = room::rasterize(sb, room::mul4(proj, view), 1280, 900, {0.010F, 0.012F, 0.020F, 1});
        const std::string path = snapshotDir + "/home-1-view.ppm";
        step("snapshot", room::writePpm(img, path), path);

        // the app's window: a screenshot when given (--panel-ppm), else the test card
        std::uint32_t pw = 1500, ph = 920;
        std::vector<std::uint8_t> pix = win::testCard(pw, ph);
        if (!panelPpm.empty()) {
            if (auto pp = sky::loadPpm(panelPpm, 8192)) {
                pw = static_cast<std::uint32_t>(pp->w);
                ph = static_cast<std::uint32_t>(pp->h);
                pix.assign(static_cast<std::size_t>(pw) * ph * 4, 255);
                for (std::size_t i = 0; i < static_cast<std::size_t>(pw) * ph; i++) {
                    pix[i * 4] = pp->rgb[i * 3 + 2];
                    pix[i * 4 + 1] = pp->rgb[i * 3 + 1];
                    pix[i * 4 + 2] = pp->rgb[i * 3];
                }
            } else {
                step("--panel-ppm", false, pp.error());
            }
        }
        // one eye of the home, as the headset would show it: the background, the window, the controllers and the ray
        const room::Mat34 eh0 = headAt({0.0, 1.62, 1.2}, 0.0);
        room::Mat34 eyeOff;
        eyeOff.m[0][3] = -0.032;
        const room::Mat34 eye = room::mul34(eh0, eyeOff);
        const double tl = -1.0, tr = 0.95, tu = 0.85, td = -0.9;
        const int ew = 1100, eh = 950;
        const win::Geometry pg = win::place(eh0, 0.0, pw, ph);
        Frame hf;
        hf.head = eh0;
        hf.grip[1] = room::poseMatrix(-0.30, 0.10, 0, 0.95, {0.22, 1.20, 0.80});
        hf.aim[1] = hf.grip[1];
        const auto shot = [&](room::Image bgImg, const World& wd, const std::string& name) {
            // the GPU's order: the background, the scene lists, the window, the pointer (the overlay lists)
            const room::Mat4 vp = room::mul4(room::projectionVulkan(tl, tr, tu, td, 0.05, kFarZ), room::mat4From34(room::inv34(eye)));
            room::Batch b2;
            buildScene(b2, wd, hf);
            room::rasterizeOnto(bgImg, b2, vp);
            win::drawOnto(bgImg, eye, std::tan(tl), std::tan(tr), std::tan(tu), std::tan(td), pg, pix, pw, ph);
            room::Batch ov;
            const room::Vec3 o2 = hf.aim[1]->pos();
            const room::Vec3 d2 = (pg.at(0.62, 0.35) - o2).norm(); // pointing at the window
            win::addRay(ov, o2, d2, win::hit(pg, o2, d2), pg, false);
            room::rasterizeOnto(bgImg, ov, vp);
            const std::string p2 = snapshotDir + "/" + name + ".ppm";
            step(("snapshot " + name).c_str(), room::writePpm(bgImg, p2), p2);
        };
        World wd;
        wd.grid = false;
        shot(sky::render(sky::dusk(2048, 1024), eye, std::tan(tl), std::tan(tr), std::tan(tu), std::tan(td), ew, eh), wd, "home-2-dusk-window");
        // the passthrough of the synthetic room through all four cameras, its walls as the reprojection surface
        const auto rig = xrkit::ptsynth::referenceRig(true);
        step("passthrough: the reference rig has four cameras", rig && rig->n == 4);
        if (rig) {
            namespace pt = xrkit::pt;
            const roommap::synth::Room synthRoom;
            const pt::Frame f0 = xrkit::ptsynth::synthFrame(synthRoom, *rig, eh0, 1);
            pt::Look look;
            look.gain = pt::autoGain(f0.mean);
            look.camGain = pt::balanceGains(f0);
            look.room = xrkit::ptsynth::roomShape(synthRoom);
            const pt::Uniform un = pt::makeUniform(0, pt::eyeView(eye, tl, tr, tu, td), *rig, eh0, look);
            World wr;
            wr.grid = false;
            wr.room = look.room->outline;
            wr.roomHeight = look.room->ceilingY;
            shot(pt::render(un, f0, ew, eh), wr, "home-3-passthrough-window");
            // looking up at the ceiling: what the upper cameras add
            const room::Mat34 upHead = room::mul34(eh0, room::poseMatrix(std::sin(0.35), 0, 0, std::cos(0.35), {0, 0, 0}));
            const room::Mat34 upEye = room::mul34(upHead, eyeOff);
            const pt::Frame fu = xrkit::ptsynth::synthFrame(synthRoom, *rig, upHead, 2);
            const pt::Uniform uu = pt::makeUniform(0, pt::eyeView(upEye, tl, tr, tu, td), *rig, upHead, look);
            const std::string p4 = snapshotDir + "/home-4-passthrough-up-coverage.ppm";
            step("snapshot home-4-passthrough-up-coverage", room::writePpm(pt::coverage(uu, fu, ew, eh), p4), p4);
            const auto lower = xrkit::ptsynth::referenceRig(false);
            if (lower) {
                const pt::Frame fl = xrkit::ptsynth::synthFrame(synthRoom, *lower, upHead, 2);
                const pt::Uniform ul = pt::makeUniform(0, pt::eyeView(upEye, tl, tr, tu, td), *lower, upHead, look);
                const std::string p5 = snapshotDir + "/home-5-passthrough-up-two-cameras.ppm";
                step("snapshot home-5-passthrough-up-two-cameras", room::writePpm(pt::render(ul, fl, ew, eh), p5), p5);
                const std::string p6 = snapshotDir + "/home-6-passthrough-up-four-cameras.ppm";
                step("snapshot home-6-passthrough-up-four-cameras", room::writePpm(pt::render(uu, fu, ew, eh), p6), p6);
            }
        }
    }

    fs::remove_all(tmp, ec);
    std::printf("%s\n", g_failed == 0 ? "self test passed" : "self test FAILED");
    return g_failed == 0 ? 0 : 1;
}

} // namespace pibridge::home
