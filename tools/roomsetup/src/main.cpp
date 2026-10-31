// SPDX-License-Identifier: MIT
// pibridge-roomsetup: the room setup in the headset (a C++ port of ZoneTweak's editor, MIT, Copyright (c) 2026
// killdano; see NOTICE and docs/ROOM-SETUP.md), for SteamVR's chaperone on Linux. Also: --list, --use NAME (write a
// saved room without VR), --selftest (no headset), and the OpenVR helpers it runs as its own child processes.
#include "host.h"
#include "openvr_helper.h"
#include "lhstage.h"
#include "options.h"
#include "selftest.h"

#ifdef PIBRIDGE_ROOMSETUP_XR
#include "xrapp.h"
#endif

#include "pibridge/room/chap.h"
#include "pibridge/room/editor.h"
#include "pibridge/room/files.h"
#include "pibridge/room/rooms.h"

#include "pibridge/anchor/backend.h"
#include "pibridge/anchor/world.h"
#include "pibridge/roommap/floor.h"
#include "pibridge/roommap/guide_ipc.h"
#include "pibridge/roommap/setup.h"
#include "pibridge/roommap/store.h"

#include <atomic>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <mutex>
#include <thread>

extern "C" char** environ;

using namespace pibridge;
using namespace pibridge::roomsetup;

namespace {

std::atomic<bool> g_stop{false};

extern "C" void onSignal(int) { g_stop.store(true); }

int fail(const std::string& e, int code = 1) {
    std::fprintf(stderr, "pibridge-roomsetup: %s\n", e.c_str());
    return code;
}

/// Whether SteamVR's server runs (a process named vrserver).
bool vrserverRunning() {
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator("/proc", ec)) {
        const std::string n = e.path().filename().string();
        if (n.empty() || n.find_first_not_of("0123456789") != std::string::npos) {
            continue;
        }
        std::ifstream f(e.path() / "comm");
        std::string comm;
        if (std::getline(f, comm) && comm == "vrserver") {
            return true;
        }
    }
    return false;
}

double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Paths {
    std::vector<std::string> configDirs;
    std::string rooms;
    std::string settings;
};

Paths paths(const Options& o) {
    const auto env = room::Env::fromProcess();
    Paths p;
    p.configDirs = o.configDirs.empty() ? room::steamConfigDirs(env) : o.configDirs;
    p.rooms = o.roomsDir.empty() ? room::roomsDir(env) : o.roomsDir;
    if (!o.settingsFile.empty()) {
        p.settings = o.settingsFile;
    } else if (!p.configDirs.empty()) {
        p.settings = p.configDirs.front() + "/steamvr.vrsettings";
    } else {
        p.settings = env.dataHome() + "/Steam/config/steamvr.vrsettings";
    }
    return p;
}

int list(const Options& o) {
    const Paths p = paths(o);
    auto st = room::Store::load(p.configDirs, o.universe);
    if (!st) {
        std::printf("%s\n", st.error().c_str());
    } else {
        std::printf("universes (editing %s):\n", st->universe().c_str());
        for (const auto& id : st->ids()) {
            const auto& u = st->universes().at(id);
            std::printf("  %s  %d points%s  time \"%s\"  in %zu file(s)\n", id.c_str(), u.points, u.pimax ? "  pimax" : "",
                        u.time.c_str(), u.files.size());
        }
    }
    room::RoomLib lib(p.rooms);
    const auto rooms = lib.list();
    std::printf("saved rooms in %s (active: %s):\n", p.rooms.c_str(), lib.active().empty() ? "-" : lib.active().c_str());
    for (const auto& r : rooms) {
        std::printf("  %s  universe %s  %zu points  saved %s\n", r.name.c_str(), r.universe.c_str(), r.points.size(), r.saved.c_str());
    }
    return 0;
}

int use(const Options& o) {
    const Paths p = paths(o);
    room::RoomLib lib(p.rooms);
    const auto r = lib.find(o.useName);
    if (!r) {
        std::string names;
        for (const auto& x : lib.list()) {
            names += (names.empty() ? "" : ", ") + x.name;
        }
        return fail("no saved room called \"" + o.useName + "\". Saved rooms: " + (names.empty() ? "none" : names));
    }
    if (vrserverRunning()) {
        return fail("SteamVR is running: it would overwrite the chaperone file when it exits. Close SteamVR first, or "
                    "pick the room in the headset (Saved spaces).");
    }
    auto st = room::Store::load(p.configDirs, r->universe);
    if (!st) {
        return fail(st.error());
    }
    const std::string id = st->universes().contains(r->universe) ? r->universe : st->universe();
    auto w = st->writeUniverse(id, r->toUniverse(), r->standing.has_value(), r->playArea.has_value(), std::time(nullptr));
    if (!w) {
        return fail(w.error());
    }
    lib.setActive(r->name);
    std::printf("%s is now the boundary of universe %s (%zu file(s)); SteamVR shows it at its next start.\n", r->name.c_str(),
                id.c_str(), w->written.size());
    return 0;
}

#ifdef PIBRIDGE_ROOMSETUP_XR
int headset(const Options& o) {
    const Paths p = paths(o);
    const HelperRunner run = [](const std::vector<std::string>& a) { return runSelf(a); };
    std::string universe = o.universe;
    if (universe.empty() && !o.noOpenVr) {
        universe = currentUniverse(run);
        if (!universe.empty()) {
            std::printf("SteamVR tracks in universe %s\n", universe.c_str());
        }
    }
    auto st = room::Store::load(p.configDirs, universe);
    if (!st) {
        return fail(st.error());
    }
    room::RoomLib lib(p.rooms);
    const auto pimax = st->universes().at(st->universe()).pimax;
    if (const auto m = lib.frameMode(room::defaultFrameMode(pimax)); m != st->mode()) {
        (void)st->setMode(m, st->stagePts());
    }
    std::printf("editing universe %s: %zu points, align %s, files:", st->universe().c_str(), st->stagePts().size(),
                room::frameModeName(st->mode()));
    for (const auto& f : st->universeFiles()) {
        std::printf(" %s", f.c_str());
    }
    std::printf("\n");
    std::unique_ptr<ToolZoneControl> zone;
    std::unique_ptr<room::Editor> ed;
    const EditorFactory make = [&](const std::string& runtimeName, Haptic haptic) {
        const bool steamvr = !o.noOpenVr && runtimeName.find("SteamVR") != std::string::npos;
        zone = std::make_unique<ToolZoneControl>(p.settings, p.rooms, steamvr, run, [] { return std::time(nullptr); });
        room::EditorHost host;
        host.now = steadySeconds;
        host.wallClock = [] { return std::time(nullptr); };
        host.haptic = std::move(haptic);
        host.afterSave = [run, steamvr](const room::SaveResult&) { return reloadAfterSave(run, steamvr); };
        host.zone = zone.get();
        ed = std::make_unique<room::Editor>(*st, lib, host);
        // inside-out (CustomHeadsetOpenVR's SLAM headset, universe 30): no persistent world anchor yet, so a recenter
        // comes first
        ed->say(st->universe() == "30" ? std::string("Inside-out: recenter first (Room page, Recenter in headset). B opens the menu")
                                      : "Universe " + st->universe() + ". B opens the menu");
        return ed.get();
    };
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    auto r = runXr(make, XrOpts{o.resDiv}, g_stop);
    if (!r) {
        return fail(r.error());
    }
    if (ed && ed->savedAny) {
        std::printf("saved; the boundary is in %s\n", st->source().c_str());
    }
    return 0;
}

int recenter(const Options& o) {
    // the backend's head ring (anchor/backend.h), unless --ring names one
    const std::string ring = o.ringFile.empty() ? anchor::ring_path() : o.ringFile;
    std::error_code ec;
    std::filesystem::remove(o.recenterOut, ec); // an old result never counts
    RecenterFlow flow;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    auto r = runRecenterXr(flow, XrOpts{o.resDiv}, g_stop, [ring] { return readRing(ring); });
    if (!r) {
        return fail(r.error());
    }
    if (!flow.result()) {
        return fail("recenter cancelled", 3);
    }
    if (auto w = room::writeFileAtomic(o.recenterOut, recenterResultText(*flow.result())); !w) {
        return fail(w.error());
    }
    std::printf("recenter written to %s\n", o.recenterOut.c_str());
    return 0;
}

/// --lh-stage (the Monado backend in the Lighthouse mode): the recenter steps in the headset (centre, forward, the floor
/// touch, else --eye-height below the head, else the old floor), then the room's stage written to the room
/// (<room>/lh-stage) and to the live file steamvr_lh re-reads (lhstage.h). Takes effect at once.
int lhStage(const Options& o) {
    const std::string ring = o.ringFile.empty() ? anchor::ring_path() : o.ringFile;
    RecenterFlow flow;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    // the base stations where they stand (steamvr_lh's file, re-read once a second: the stage moves with the setup)
    std::vector<room::Mat34> stations;
    auto stationsRead = std::chrono::steady_clock::time_point{};
    const auto drawStations = [&stations, &stationsRead](room::Batch& b) {
        const auto now = std::chrono::steady_clock::now();
        if (now - stationsRead > std::chrono::seconds(1)) {
            stationsRead = now;
            // only a file Monado is writing now (once a second), never one an earlier session left
            std::error_code ec;
            const auto age = std::filesystem::file_time_type::clock::now() -
                             std::filesystem::last_write_time(baseStationsPath(), ec);
            std::ifstream f(baseStationsPath());
            stations = ec || age > std::chrono::seconds(5)
                           ? std::vector<room::Mat34>{}
                           : parseBaseStations(std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>()));
        }
        drawBaseStations(b, stations);
    };
    auto r = runRecenterXr(flow, XrOpts{o.resDiv}, g_stop, [ring] { return readRing(ring); }, drawStations);
    if (!r) {
        return fail(r.error());
    }
    if (!flow.result()) {
        return fail("room setup cancelled", 3);
    }
    const std::string roomDir = roommap::store::map_dir(o.room);
    const std::string live = lhStageRuntimePath();
    // the head was read in the stage that is live now: the live file, else the saved one, else the raw space
    StagePose old = kStageIdentity;
    for (const std::string& p : {live, lhStageRoomPath(roomDir)}) {
        std::ifstream f(p);
        std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        if (const auto st = parseStage(text)) {
            old = *st;
            break;
        }
    }
    const StagePose st = lhStageFromRecenter(old, *flow.result(), o.eyeHeight);
    std::error_code ec;
    std::filesystem::create_directories(roomDir, ec);
    std::filesystem::create_directories(std::filesystem::path(live).parent_path(), ec);
    for (const std::string& p : {lhStageRoomPath(roomDir), live}) {
        if (auto w = room::writeFileAtomic(p, stageText(st)); !w) {
            return fail(w.error());
        }
    }
    std::printf("Lighthouse stage of room \"%s\" (Monado backend): %s written to %s and %s\n", o.room.c_str(),
                stageText(st).c_str(), lhStageRoomPath(roomDir).c_str(), live.c_str());
    return 0;
}

/// pibridge-roommap: --roommap, else next to this binary, else on PATH.
std::string roommapTool(const Options& o) {
    if (!o.roommapTool.empty()) {
        return o.roommapTool;
    }
    std::error_code ec;
    const auto self = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        const auto next = self.parent_path() / "pibridge-roommap";
        if (std::filesystem::exists(next, ec)) {
            return next.string();
        }
        // the build tree: tools/roomsetup/pibridge-roomsetup next to tools/roommap/pibridge-roommap
        const auto build = self.parent_path().parent_path() / "roommap" / "pibridge-roommap";
        if (std::filesystem::exists(build, ec)) {
            return build.string();
        }
    }
    return "pibridge-roommap";
}

std::string lastLine(const std::string& file) {
    std::ifstream f(file);
    std::string line, last;
    while (std::getline(f, line)) {
        if (!line.empty()) {
            last = line;
        }
    }
    return last;
}

/// Where the mapping run's logs go: --log-dir, else the logs folder of the newest inside-out session
/// ($XDG_STATE_HOME/pibridge/inside-out/<yyyyMMdd-HHmmss>/logs, the session that runs), else "".
std::string mapLogDir(const Options& o) {
    if (!o.logDir.empty()) {
        return o.logDir;
    }
    const char* sh = std::getenv("XDG_STATE_HOME");
    const char* home = std::getenv("HOME");
    const std::filesystem::path base = sh && *sh ? std::filesystem::path(sh) / "pibridge/inside-out"
                                                 : std::filesystem::path(home ? home : "/tmp") / ".local/state/pibridge/inside-out";
    std::error_code ec;
    std::string best;
    for (const auto& e : std::filesystem::directory_iterator(base, ec)) {
        const std::string n = e.path().filename().string();
        if (e.is_directory(ec) && std::filesystem::is_directory(e.path() / "logs", ec) && n > best) {
            best = n;
        }
    }
    return best.empty() ? std::string() : (base / best / "logs").string();
}

/// Run a program to its end, its output appended to `log`; the exit code (127: not started).
int runTo(const std::vector<std::string>& args, const std::string& log) {
    std::vector<std::string> a = args;
    std::vector<char*> argv;
    for (auto& x : a) {
        argv.push_back(x.data());
    }
    argv.push_back(nullptr);
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 1, log.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    posix_spawn_file_actions_adddup2(&fa, 1, 2);
    pid_t pid = 0;
    const int sp = posix_spawnp(&pid, a[0].c_str(), &fa, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    if (sp != 0) {
        return 127;
    }
    int st = 0;
    ::waitpid(pid, &st, 0);
    return WIFEXITED(st) ? WEXITSTATUS(st) : 128;
}

/// --floor-height: SET FLOOR FROM MY HEIGHT (Room page) in the running session, without mapping: the room must be
/// recognised in this session (its room fix, last-reloc.toml, names the session). In the headset, over the
/// passthrough: "stand up straight, look ahead, pull the trigger"; the floor goes the eye height below the head
/// (OpenXR's head pose, the eye midpoint, averaged over 1 s), into the map's floor.toml (`pibridge-roommap floor set --y`)
/// and Monado's offset (`pibridge-roommap apply`, through the writer lease).
int floorHeight(const Options& o) {
    if (!o.eyeHeight) {
        return fail("--floor-height needs --eye-height M");
    }
    const std::string dir = roommap::store::map_dir(o.room);
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    const std::string ringPath = o.ringFile.empty() ? anchor::ring_path() : o.ringFile;
    const auto ring = readRing(ringPath);
    auto res = roommap::store::RelocResult::load(dir + "/last-reloc.toml");
    if (!ring || ring->pid == 0) {
        return fail("no head poses from the tracker: start VR in inside-out first with the room anchor on");
    }
    if (!res || res->pid != ring->pid) {
        return fail("the room \"" + o.room + "\" is not recognised in this session: wait for \"Room recognised\" or map the room");
    }
    const std::string tool = roommapTool(o);
    const std::string logs = mapLogDir(o);
    const std::string log = (logs.empty() ? std::string(rt && *rt ? rt : "/tmp") : logs) + "/roomsetup-floor.log";
    guide::State st;
    st.pid = static_cast<std::uint32_t>(::getpid());
    st.seq = 1;
    st.step = "floor-height";
    st.title = "Floor from your height";
    st.text = std::format("Stand up straight, look ahead and pull the trigger: the floor goes {:.2f} m below your eyes.",
                          *o.eyeHeight);
    st.conv = guide::Conv{res->align.yaw, {res->align.t[0], res->align.t[1], res->align.t[2]}};
    if (auto f = roommap::floor::StoredFloor::load(dir)) {
        st.floor = f->y;
    }
    std::mutex m;
    std::optional<int> code;
    std::thread worker;
    MapFlow flow;
    flow.setPid(st.pid);
    flow.setPimaxFloorSource(true); // use the Pimax engine's floor (room-state.json) on the floor step, when it runs
    flow.setEyeHeight(o.eyeHeight);
    MapLink link;
    link.camera = o.camera;
    link.readState = [&]() -> std::optional<guide::State> {
        std::lock_guard l(m);
        return st;
    };
    link.writeAnswer = [&](const guide::Answer& a) {
        std::lock_guard l(m);
        if (a.kind == "cancel") {
            st.step = "failed";
            st.title = "Floor not changed";
            st.text = "Stopped in the headset.";
            st.seq++;
            code = 3;
            return;
        }
        if (a.kind != "height" || worker.joinable()) {
            return;
        }
        const double y = a.nums[0];
        st.text = "Setting the floor...";
        st.seq++;
        worker = std::thread([&, y] {
            const std::string ys = std::format("{:.4f}", y);
            int c = runTo({tool, "floor", "set", "--name", o.room, "--y", ys}, log);
            if (c == 0) {
                c = runTo({tool, "apply", "--name", o.room}, log);
            }
            std::lock_guard l2(m);
            st.step = c == 0 ? "done" : "failed";
            st.title = c == 0 ? "Floor set" : "Floor not changed";
            st.text = c == 0 ? std::format("The floor is now {:.2f} m below your eyes.", *o.eyeHeight)
                             : "pibridge-roommap could not set it: " + lastLine(log);
            st.floor = y;
            st.seq++;
            code = c;
        });
    };
    link.childExit = [&]() -> std::optional<int> {
        std::lock_guard l(m);
        return code;
    };
    link.childLastLine = [&log] { return lastLine(log); };
    link.readRing = [ringPath] { return readRing(ringPath); };
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    auto r = runMapXr(flow, link, XrOpts{o.resDiv}, g_stop);
    if (worker.joinable()) {
        worker.join();
    }
    if (!r) {
        return fail(r.error());
    }
    if (!code || *code != 0) {
        return fail(code ? lastLine(log) : std::string("the floor was not set"), code && *code == 3 ? 3 : 1);
    }
    std::printf("floor set %.2f m below the eyes (log %s)\n", *o.eyeHeight, log.c_str());
    return 0;
}

int mapRoom(const Options& o) {
    const char* rt = std::getenv("XDG_RUNTIME_DIR");
    const std::string dir = !o.guideDir.empty() ? o.guideDir
                                                : std::string(rt && *rt ? rt : "/tmp") + "/pibridge/roommap-guide-" +
                                                      std::to_string(::getpid());
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::filesystem::remove(dir + "/" + roommap::guide::STATE_FILE, ec);
    std::filesystem::remove(dir + "/" + roommap::guide::ANSWER_FILE, ec);
    const std::string ring = o.ringFile.empty() ? anchor::ring_path() : o.ringFile;
    // pibridge-roommap does the work. The logs: its output and this tool's own lines, in the session's logs folder
    const std::string tool = roommapTool(o);
    const std::string logs = mapLogDir(o);
    const std::string log = (logs.empty() ? dir : logs) + "/roommap-setup.log";
    const std::string ownLog = (logs.empty() ? dir : logs) + "/roomsetup-map.log";
    const int out = ::dup(1);
    if (const int fd = ::open(ownLog.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644); fd >= 0) {
        std::fflush(stdout);
        std::fflush(stderr);
        ::dup2(fd, 1);
        ::dup2(fd, 2);
        ::close(fd);
        setvbuf(stdout, nullptr, _IOLBF, 0);
    }
    const auto said = [out](const std::string& line) { // to the log and to the app (the original stdout)
        std::printf("%s\n", line.c_str());
        std::fflush(stdout);
        if (out >= 0) {
            (void)!::write(out, (line + "\n").data(), line.size() + 1);
        }
    };
    said("pibridge-roomsetup --map: room " + o.room + ", log " + ownLog + ", pibridge-roommap's output " + log);
    std::vector<std::string> args{tool, "setup", "fast", "--name", o.room, "--guide", dir, "--no-hud"};
    if (o.chaperone) {
        // SteamVR path: pibridge-roommap saves the setup but writes no Monado offset; this tool then writes SteamVR's
        // chaperone standing from the live head pose (below, after the wizard ends).
        args.push_back("--no-offset");
    }
    if (o.eyeHeight) {
        args.push_back("--eye-height");
        args.push_back(std::format("{:.3f}", *o.eyeHeight));
    }
    if (!o.mapSeconds.empty()) {
        args.push_back("--map-seconds");
        args.push_back(o.mapSeconds);
    }
    if (o.quickDirect) {
        args.push_back("--quick-direct"); // skip the in-VR Full/Quick mode menu, go straight to quick
    }
    std::vector<char*> argv;
    for (auto& a : args) {
        argv.push_back(a.data());
    }
    argv.push_back(nullptr);
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 1, log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&fa, 1, 2);
    pid_t child = 0;
    const int sp = posix_spawnp(&child, tool.c_str(), &fa, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&fa);
    if (sp != 0) {
        return fail("cannot start " + tool + ": " + std::strerror(sp));
    }
    std::printf("room mapping in the headset: %s (pid %d), guide %s\n", tool.c_str(), static_cast<int>(child), dir.c_str());
    std::fflush(stdout);
    std::optional<int> exitCode;
    const auto poll = [&]() -> std::optional<int> {
        if (exitCode) {
            return exitCode;
        }
        int st = 0;
        if (::waitpid(child, &st, WNOHANG) == child) {
            exitCode = WIFEXITED(st) ? WEXITSTATUS(st) : 128 + (WIFSIGNALED(st) ? WTERMSIG(st) : 0);
        }
        return exitCode;
    };
    MapFlow flow;
    flow.setPid(static_cast<std::uint32_t>(::getpid()));
    flow.setPimaxFloorSource(true); // use the Pimax engine's floor (room-state.json) on the floor step, when it runs
    flow.setEyeHeight(o.eyeHeight);
    MapLink link;
    link.camera = o.camera;
    link.readState = [&dir]() -> std::optional<roommap::guide::State> {
        const auto t = roommap::guide::read_file(dir, roommap::guide::STATE_FILE);
        if (!t) {
            return std::nullopt;
        }
        auto s = roommap::guide::parse_state(*t);
        return s ? std::optional(*s) : std::nullopt;
    };
    link.readScanPoints = [&dir]() -> std::vector<roommap::guide::P3f> {
        const auto t = roommap::guide::read_file(dir, roommap::guide::POINTS_FILE);
        return t ? roommap::guide::parse_points(*t) : std::vector<roommap::guide::P3f>{};
    };
    link.writeAnswer = [&dir](const roommap::guide::Answer& a) {
        std::string nums;
        for (const double d : a.nums) {
            nums += std::format(" {:.3f}", d);
        }
        const std::string what = a.kind == "path" ? std::format(" ({} points)", a.nums.size() / 2) : nums;
        std::printf("answer %llu to %s: %s%s\n", static_cast<unsigned long long>(a.id), a.step.c_str(), a.kind.c_str(),
                    what.c_str());
        if (auto w = roommap::guide::write_atomic(dir, roommap::guide::ANSWER_FILE, roommap::guide::format_answer(a)); !w) {
            std::fprintf(stderr, "pibridge-roomsetup: %s\n", w.error().c_str());
        }
    };
    link.childExit = poll;
    link.childLastLine = [&log] { return lastLine(log); };
    link.readRing = [ring] { return readRing(ring); };
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    auto r = runMapXr(flow, link, XrOpts{o.resDiv}, g_stop);
    // pibridge-roommap still running (the headset view ended first): ask it to stop, then end it by its pid
    if (!poll()) {
        roommap::guide::Answer a;
        a.pid = static_cast<std::uint32_t>(::getpid());
        a.id = 1000000;
        a.kind = "cancel";
        a.step = "cancel";
        link.writeAnswer(a);
        for (int i = 0; i < 30 && !poll(); i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!poll()) {
            ::kill(child, SIGTERM);
            for (int i = 0; i < 30 && !poll(); i++) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        if (!poll()) {
            ::kill(child, SIGKILL);
            int st = 0;
            ::waitpid(child, &st, 0);
            exitCode = 137;
        }
    }
    std::printf("pibridge-roommap ended with code %d\n", *exitCode);
    if (logs.empty()) {
        std::ifstream f(log);
        std::string line;
        while (std::getline(f, line)) {
            said(line);
        }
    }
    const std::string why = !r ? r.error() : *exitCode != 0 ? lastLine(log) : std::string();
    std::fflush(stdout);
    if (out >= 0) { // the end goes to the app again
        ::dup2(out, 1);
        ::dup2(out, 2);
        ::close(out);
    }
    if (!logs.empty()) {
        std::filesystem::remove_all(dir, ec);
    }
    if (!r) {
        return fail(why);
    }
    if (*exitCode != 0) {
        return fail(why.empty() ? "the room mapping did not finish" : why, 1);
    }
    std::printf("room mapped (logs: %s)\n", ownLog.c_str());
    // SteamVR path: write the floor/centre/forward into SteamVR's chaperone from the live head pose (the user has just
    // confirmed forward, so they are standing in the centre looking forward), then ask SteamVR to reload it.
    if (o.chaperone) {
        std::printf("writing the SteamVR chaperone from the room setup...\n");
        std::fflush(stdout);
        std::error_code ec2;
        const auto self = std::filesystem::read_symlink("/proc/self/exe", ec2);
        const std::string selfPath = ec2 ? std::string("pibridge-roomsetup") : self.string();
        const auto st = roommap::setup::RoomSetup::load(roommap::store::map_dir(o.room));
        std::string perimFile;
        // Seated is the same as standing, only with a seated eye height (~1.30 m) instead of ~1.60: the floor still goes
        // `drop` below the LIVE head, and the chosen zone is still the play-area boundary. head_m holds the drop the
        // setup saved (Sit ~1.30, Stand ~1.60; full = the measured floor), falling back to the app's eye height.
        {
            std::optional<double> drop;
            if (st && st->head_m > 0.5 && st->head_m < 3.0) {
                drop = st->head_m;
            } else if (o.eyeHeight) {
                drop = o.eyeHeight;
            }
            if (drop) {
                std::vector<std::string> va{selfPath, "--vr-standing", "--eye-height", std::format("{:.3f}", *drop)};
                // the saved boundary (traced hull in full, the auto circle in quick) as the chaperone perimeter: its
                // setup-frame points equal the standing frame, committed with the floor/centre/forward in one Live write.
                if (st) {
                    const auto poly = st->boundary_setup();
                    if (poly.size() >= 3) {
                        std::string txt = "# pibridge chaperone perimeter (setup frame, x z metres)\n";
                        for (const auto& p : poly) {
                            txt += std::format("{:.4f} {:.4f}\n", p[0], p[1]);
                        }
                        perimFile =
                            std::string(rt && *rt ? rt : "/tmp") + "/pibridge-perimeter-" + std::to_string(::getpid());
                        if (room::writeFileAtomic(perimFile, txt)) {
                            va.push_back("--perimeter");
                            va.push_back(perimFile);
                        } else {
                            perimFile.clear();
                        }
                    }
                }
                const bool havePerim = !perimFile.empty();
                const int cs = runTo(va, ownLog);
                std::printf("%s\n",
                            cs == 0 ? (havePerim ? "SteamVR chaperone: floor, centre, forward and the boundary set"
                                                 : "SteamVR chaperone: floor, centre and forward set")
                                    : ("SteamVR chaperone: could not set the standing pose (code " + std::to_string(cs) +
                                       ", see " + ownLog + ")")
                                          .c_str());
            } else {
                std::printf("SteamVR chaperone: no floor height known, the standing pose was left as it was\n");
            }
        }
        if (!perimFile.empty()) {
            std::error_code ec3;
            std::filesystem::remove(perimFile, ec3);
        }
        (void)runTo({selfPath, "--vr-reload"}, ownLog);
    }
    return 0;
}
#endif

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    auto o = parseArgs(args);
    if (!o) {
        std::fprintf(stderr, "pibridge-roomsetup: %s\n\n%s", o.error().c_str(), usage().c_str());
        return 2;
    }
    if (o->help) {
        std::fputs(usage().c_str(), stdout);
        return 0;
    }
    if (!o->backend.empty()) {
        // the library (rooms folder, ring, offset) and every child (pibridge-roommap) follow the same backend
        ::setenv(anchor::BACKEND_ENV, o->backend.c_str(), 1);
    }
    switch (o->run) {
    case Run::VrUniverse: return vrUniverseMain();
    case Run::VrReload: return vrReloadMain();
    case Run::VrStanding: {
        // The floor to write: the room setup's SAVED floor plane, put into SteamVR's raw space through the vertical of
        // the current relocalize (room = Ry(yaw) raw + t, y untouched by yaw: raw_y = room_y - t[1]). It is the floor
        // every session restores to, independent of where the head is right now. Without a stored floor / room fix
        // (the very first setup) the writer falls back to head y - eye height.
        std::optional<double> floorY;
        if (!o->seated && !o->keepFloor) {
            const std::string dir = roommap::store::map_dir(o->room);
            const auto f = roommap::floor::StoredFloor::load(dir);
            const auto res = roommap::store::RelocResult::load(dir + "/last-reloc.toml");
            if (f && res) {
                floorY = f->y - res->align.t[1];
                std::printf("floor: the saved plane y %.3f (room) -> %.3f (raw) through the room fix\n", f->y, *floorY);
            }
        }
        return vrStandingMain(*o, floorY);
    }
    case Run::VrZone: return vrZoneMain(*parseZoneArg(o->zoneArg));
    case Run::SelfTest: return runSelfTest(o->snapshotDir);
    case Run::List: return list(*o);
    case Run::Use: return use(*o);
    case Run::Recenter:
#ifdef PIBRIDGE_ROOMSETUP_XR
        return recenter(*o);
#else
        return fail("built without OpenXR / Vulkan: no --recenter", 2);
#endif
    case Run::LhStage:
#ifdef PIBRIDGE_ROOMSETUP_XR
        return lhStage(*o);
#else
        return fail("built without OpenXR / Vulkan: no --lh-stage", 2);
#endif
    case Run::FloorHeight:
#ifdef PIBRIDGE_ROOMSETUP_XR
        return floorHeight(*o);
#else
        return fail("built without OpenXR / Vulkan: no --floor-height", 2);
#endif
    case Run::Map:
#ifdef PIBRIDGE_ROOMSETUP_XR
        return mapRoom(*o);
#else
        return fail("built without OpenXR / Vulkan: no --map", 2);
#endif
    case Run::Headset:
#ifdef PIBRIDGE_ROOMSETUP_XR
        return headset(*o);
#else
        return fail("built without OpenXR / Vulkan: only --selftest, --list and --use work", 2);
#endif
    }
    return 2;
}
