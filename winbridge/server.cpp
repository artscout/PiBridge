// pibridge winbridge server: hosts the real Pimax Play tracking engines under Wine -- head SLAM
// (pimax_slam.pi.dll) and controller LED tracking (LedObjectPoseEstimator.dll) -- and serves one
// Linux client (libpibridge_vit_winbridge.so in Monado) over TCP localhost (proto.h).
// Build: x86_64-w64-mingw32-g++ -O2 -static server.cpp -lws2_32 -o winbridge.exe
// Run: WINEDEBUG=-all wine winbridge.exe [port]
// Threads: the socket loop makes the head SLAM calls; the controller LED engine's calls run on a worker thread of their
// own (ctrl_worker; PIBRIDGE_WB_CTRL_THREAD=0 = inline), so a slow controller engine never holds up the head.
//
// Everything as in pi_server (frida pass1-3, re-dll/README.md):
//   head:        HeadsetInitialImpl -> SetTrackingMode(1) -> SetDeviceEnable(1); PutCameraImage(4 frames),
//                PutHmdImuData -> pose in the output buffer;
//   controllers: ControllerInitialImpl -> SetDeviceEnable(1) -> AddObject(hand, 15 LED, ImuToDevice, ...)
//                from slam\H1\<serial>\controller_params.json; PutCameraImage(4 LED frames, head pose, NULL),
//                PutControllerImuData -> pose in the output buffer.
// The LED engine lives in axes (x, z, -y) of the SLAM world (world rotation Rx(-90 deg), body unchanged; verified on one
// head pose caught in pass1 and pass3 in two forms). The server converts the head pose into that frame and controller poses back.
// Kit: $PIBRIDGE_WINDATA or <Wine home folder>\.local\share\pibridge\windata (dlls\ + slam\).

#include <winsock2.h>
#include <windows.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include "allocwatch.h"

#include <cstdlib>
#include <cstring>
#include "proto.h"

typedef int (*InitialFn)(const char*, const char*, const char*, void*, void*);
// pimax_slam's HeadsetInitialImpl: the 5th argument's low byte is the map mode (RE 2026-11-20): 0 kNormal (Pimax Play:
// load <serial>platMap_orborb_K8L4.* to relocalize, save it back), 1 kLabMap (no load, saves LabplatMap_*),
// 2 kLabLoc (loads LabplatMap_*, never saves)
typedef int (*HeadInitialFn)(const char*, const char*, const char*, void*, uintptr_t);
typedef int (*SetByteFn)(int);
typedef int (*PutCamFn)(void*, void*, void*, void*, float);
typedef int (*PutImuFn)(void*, void*);
// HeadsetGetGroundState / HeadsetGetLocModeState: one out pointer, int status (-1 engine down, 0 not ready, 1 valid).
typedef int (*GetStateFn)(void*);

// Head frame: {u64 ts; u32 exposure_ns; u32 gain; u32 idx; w; h; stride; px} (0 exposure = frame dropped).
struct CamFrame {
    uint64_t ts;
    uint64_t shutter_gain;  // exposure | gain << 32
    uint32_t idx, width, height, stride;
    uint8_t* pixels;
};
static_assert(sizeof(CamFrame) == 40, "CamFrame must be 40 bytes");
struct ImuSample { uint64_t ts; float accel[3]; float gyro[3]; float pad[8]; };

// LED frame: {u64 ts; u32 exposure_ns; u32 idx; w; h; stride; pad; px}.
struct LedFrame {
    uint64_t ts;
    uint32_t shutter_ns, idx;
    uint32_t width, height, stride, pad;
    uint8_t* pixels;
};
static_assert(sizeof(LedFrame) == 40, "LedFrame must be 40 bytes");
struct HeadPoseForLed { float q[4]; float p[3]; float pad[9]; };
struct LedRec { int32_t id; float pos[3]; float normal[3]; };
struct MsvcString { char buf[16]; uint64_t size; uint64_t cap; };  // std::string MSVC x64, short string
typedef int (*CtrlAddFn)(int, int, const LedRec*, const float*, int, int, int, const MsvcString*);
typedef int (*CtrlPutCamFn)(void*, void*, void*, void*, const HeadPoseForLed*, void*);
typedef int (*CtrlPutImuFn)(int, void*, float*, float*);

static PutCamFn g_putCam;
static PutImuFn g_putImu;
static GetStateFn g_getGround;  // HeadsetGetGroundState (null = DLL does not export it)
static GetStateFn g_getLoc;     // HeadsetGetLocModeState
static CtrlPutCamFn g_ctrlCam;
static CtrlPutImuFn g_ctrlImu;
static bool g_ctrlOn = false;
static float g_devFromImu[2][4] = {{0, 0, 0, 1}, {0, 0, 0, 1}};  // ImuToDevice of each hand (quat x,y,z,w)
static float g_head[13] = {0, 0, 0, 1};  // latest head pose (SLAM world)
static bool g_headSeen = false;          // at least one IMU from the head arrived (otherwise world = headset)
static uint64_t g_headTs = 0;            // timestamp of the IMU sample behind g_head (LED-frame staleness)
// Head-pose history (SLAM world), so an LED camera frame is projected with the head pose AT ITS CAPTURE TIME, not the
// latest IMU pose: the LED stream arrives ~17 ms after the same-time IMU (measured), and the LED engine is given no
// timestamp for the head pose, so that lag becomes a controller error that follows head motion.
static const int kHeadRing = 128;
static uint64_t g_headRingTs[kHeadRing] = {0};
static float g_headRing[kHeadRing][13];
static int g_headRingPos = 0;
static uint64_t g_headUsedTs = 0;        // ts of the pose head_at() last returned (for the staleness diagnostic)
static bool g_ledHeadAlign = true;       // PIBRIDGE_LED_HEAD_ALIGN=0 feeds the latest pose instead (A/B test)

// ---- LED engine axes: v_led = (x, z, -y), q_led = Q * q_slam, Q = Rx(-90 deg) ----
static const float kQ[4] = {-0.70710678f, 0.0f, 0.0f, 0.70710678f};
static void qmul(const float a[4], const float b[4], float o[4]) {
    o[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    o[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    o[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    o[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}
static void to_led(const float v[3], float o[3]) { o[0] = v[0]; o[1] = v[2]; o[2] = -v[1]; }
static void from_led(const float v[3], float o[3]) { o[0] = v[0]; o[1] = -v[2]; o[2] = v[1]; }

// The head pose whose timestamp is nearest `ts` from recent history (nearest sample; IMU ~490 Hz so the residual is
// well under 1 ms), copied into `out`. Falls back to the latest pose while the ring is still filling. The head path
// (main thread) writes the ring, the controller worker reads it: both under g_headLock.
static SRWLOCK g_headLock = SRWLOCK_INIT;
static void head_at(uint64_t ts, float out[13]) {
    const float* best = g_head;
    uint64_t bestTs = g_headTs, bestd = UINT64_MAX;
    for (int i = 0; i < kHeadRing; ++i) {
        if (g_headRingTs[i] == 0) continue;
        uint64_t d = g_headRingTs[i] > ts ? g_headRingTs[i] - ts : ts - g_headRingTs[i];
        if (d < bestd) { bestd = d; best = g_headRing[i]; bestTs = g_headRingTs[i]; }
    }
    g_headUsedTs = bestTs;
    memcpy(out, best, 13 * sizeof(float));
}

// ---- timing of the engine calls (logged every 5 s per thread: which call makes the bridge slow) ----
static double now_ms() {
    static LARGE_INTEGER f = [] { LARGE_INTEGER x; QueryPerformanceFrequency(&x); return x; }();
    LARGE_INTEGER c;
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart * 1000.0 / (double)f.QuadPart;
}
struct CallStat {
    const char* name;
    long n = 0;
    double sum = 0, max = 0;
    void add(double ms) { ++n; sum += ms; if (ms > max) max = ms; }
    void print_and_reset() {
        if (n) printf(" %s n=%ld mean %.2f max %.2f ms;", name, n, sum / n, max);
        n = 0; sum = max = 0;
    }
};

static bool recv_all(SOCKET s, void* buf, int n) {
    char* p = (char*)buf;
    while (n > 0) { int r = recv(s, p, n, 0); if (r <= 0) return false; p += r; n -= r; }
    return true;
}
static bool send_all(SOCKET s, const void* buf, int n) {
    const char* p = (const char*)buf;
    while (n > 0) { int r = send(s, p, n, 0); if (r <= 0) return false; p += r; n -= r; }
    return true;
}
static bool send_msg(SOCKET s, uint32_t type, const void* payload, uint32_t len) {
    MsgHeader h{type, len};
    return send_all(s, &h, sizeof(h)) && (len == 0 || send_all(s, payload, (int)len));
}

static char g_dlls[MAX_PATH], g_slam[MAX_PATH], g_calib[MAX_PATH];

static bool find_kit() {
    // $PIBRIDGE_WINDATA (Windows path; this is how PiBridge launches it), otherwise the home folder from $WINEHOMEDIR
    // ("\??\Z:\home\<user>": Wine does not pass HOME to the Windows process) + \.local\share\pibridge\windata
    const char* kit = getenv("PIBRIDGE_WINDATA");
    const char* wh = getenv("WINEHOMEDIR");
    char base[MAX_PATH];
    if (kit && *kit) {
        snprintf(base, sizeof(base), "%s", kit);
    } else if (wh && *wh) {
        if (strncmp(wh, "\\??\\", 4) == 0) wh += 4;
        snprintf(base, sizeof(base), "%s\\.local\\share\\pibridge\\windata", wh);
    } else {
        fprintf(stderr, "no PIBRIDGE_WINDATA and WINEHOMEDIR\n");
        return false;
    }
    for (char* p = base; *p; ++p) if (*p == '/') *p = '\\';
    snprintf(g_dlls, sizeof(g_dlls), "%s\\dlls", base);
    snprintf(g_slam, sizeof(g_slam), "%s\\slam", base);
    // calibration for the specific headset: device_calibration_<serial>.xml, copied from Pimax Play
    char pat[MAX_PATH];
    snprintf(pat, sizeof(pat), "%s\\device_calibration_*.xml", g_slam);
    WIN32_FIND_DATAA fd;
    HANDLE fh = FindFirstFileA(pat, &fd);
    if (fh == INVALID_HANDLE_VALUE) { fprintf(stderr, "no %s\n", pat); return false; }
    FindClose(fh);
    snprintf(g_calib, sizeof(g_calib), "%s\\%s", g_slam, fd.cFileName);
    printf("calibration: %s\n", fd.cFileName);
    SetDllDirectoryA(g_dlls);
    return true;
}

static bool init_slam() {
    char vocab[MAX_PATH], dll[MAX_PATH];
    snprintf(vocab, sizeof(vocab), "%s\\voc_GEN_8X4.dbow", g_slam);
    snprintf(dll, sizeof(dll), "%s\\pimax_slam.pi.dll", g_dlls);
    HMODULE h = LoadLibraryA(dll);
    if (!h) { fprintf(stderr, "LoadLibrary %s -> %lu\n", dll, GetLastError()); return false; }
    auto initial = (InitialFn)GetProcAddress(h, "HeadsetInitialImpl");
    auto setMode = (SetByteFn)GetProcAddress(h, "HeadsetSetTrackingMode");
    auto enable  = (SetByteFn)GetProcAddress(h, "HeadsetSetDeviceEnable");
    g_putCam = (PutCamFn)GetProcAddress(h, "HeadsetPutCameraImage");
    g_putImu = (PutImuFn)GetProcAddress(h, "HeadsetPutHmdImuData");
    if (!initial || !setMode || !enable || !g_putCam || !g_putImu) {
        fprintf(stderr, "not all SLAM exports found\n"); return false;
    }
    // optional getters: the floor and the localization/recognition state (MSG_ROOMREQ). Absent on older DLLs.
    g_getGround = (GetStateFn)GetProcAddress(h, "HeadsetGetGroundState");
    g_getLoc = (GetStateFn)GetProcAddress(h, "HeadsetGetLocModeState");
    printf("room state: GroundState=%s LocModeState=%s\n", g_getGround ? "yes" : "no", g_getLoc ? "yes" : "no");
    // PIBRIDGE_WB_SLAM_MAP: normal (default, as Pimax Play), loc (relocalize in a pinned copy of the saved map, never
    // save: the map stops growing and no 200 MB save runs during play), fresh (no map loaded, a new one saved to
    // LabplatMap_*). loc copies <serial>platMap_* to LabplatMap_* when the Lab copy is missing or older.
    uintptr_t map_mode = 0;
    const char* map_env = getenv("PIBRIDGE_WB_SLAM_MAP");
    if (map_env && strcmp(map_env, "loc") == 0) {
        map_mode = 2;
        // the serial: device_calibration_<serial>.xml (the map file is <serial>platMap_orborb_K8L4.bin)
        std::string serial_bin;
        const char* cbase = strrchr(g_calib, '\\');
        cbase = cbase ? cbase + 1 : g_calib;
        const char* cpre = "device_calibration_";
        if (strncmp(cbase, cpre, strlen(cpre)) == 0) {
            std::string ser(cbase + strlen(cpre));
            if (const auto dot = ser.rfind('.'); dot != std::string::npos) {
                ser.resize(dot);
            }
            serial_bin = ser + "platMap_orborb_K8L4.bin";
        }
        if (!serial_bin.empty()) {
            const std::string stem = serial_bin.substr(0, serial_bin.size() - strlen("platMap_orborb_K8L4.bin"));
            for (const char* ext : {".bin", ".yaml"}) {
                const std::string src = std::string(g_slam) + "\\" + stem + "platMap_orborb_K8L4" + ext;
                const std::string dst = std::string(g_slam) + "\\LabplatMap_orborb_K8L4" + ext;
                WIN32_FILE_ATTRIBUTE_DATA a{}, b{};
                const bool have_src = GetFileAttributesExA(src.c_str(), GetFileExInfoStandard, &a);
                const bool have_dst = GetFileAttributesExA(dst.c_str(), GetFileExInfoStandard, &b);
                if (have_src && (!have_dst || CompareFileTime(&a.ftLastWriteTime, &b.ftLastWriteTime) > 0)) {
                    printf("SLAM map: pinned copy %s -> %s: %s\n", src.c_str(), dst.c_str(),
                           CopyFileA(src.c_str(), dst.c_str(), FALSE) ? "ok" : "FAILED");
                }
            }
        } else {
            printf("SLAM map: no saved map to pin; relocalization has nothing to load\n");
        }
    } else if (map_env && strcmp(map_env, "fresh") == 0) {
        map_mode = 1;
    }
    printf("SLAM map mode: %s\n", map_mode == 2 ? "loc (pinned LabplatMap, never saved)"
                                   : map_mode == 1 ? "fresh (no map loaded, saved to LabplatMap)"
                                                   : "normal (load and save the saved map, as Pimax Play)");
    int r = ((HeadInitialFn)initial)(g_calib, g_slam, vocab, nullptr, map_mode);
    // SetTrackingMode(1) as Pimax Play: the 6DoF / 3DoF switch, not the map handling (that is InitialImpl's 5th arg)
    const int mode = 1;
    int m = setMode(mode);
    int e = enable(1);
    printf("SLAM: Initial=%d SetTrackingMode(%d)=%d SetDeviceEnable(1)=%d\n", r, mode, m, e);
    return r == 0 && m == 0 && e == 0;
}

// ---- controllers ----
static bool numbers_after(const char* s, const char* key, double* out, int count) {
    const char* p = strstr(s, key);
    if (!p || !(p = strchr(p, '['))) return false;
    ++p;
    for (int i = 0; i < count; ++i) {
        char* e = nullptr;
        out[i] = strtod(p, &e);
        if (e == p) return false;
        p = e;
        while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') ++p;
    }
    return true;
}

// controller_params.json: hand, LEDs {pos, normal}, ImuToDevice (4x4 row-major) -> quat(x,y,z,w) + translation
struct CtrlModel { int hand = -1; int n = 0; LedRec leds[32]; float imu7[8] = {0}; };
static bool load_ctrl_model(const char* path, CtrlModel& m) {
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    static char buf[16384];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = 0;
    m.hand = strstr(buf, "\"Right\"") ? 1 : (strstr(buf, "\"Left\"") ? 0 : -1);
    for (int i = 0; i < 32; ++i) {
        char key[32];
        snprintf(key, sizeof(key), "\"Point%d\"", i);
        double v[6];
        if (!numbers_after(buf, key, v, 6)) break;
        m.leds[m.n++] = LedRec{i, {(float)v[0], (float)v[1], (float)v[2]}, {(float)v[3], (float)v[4], (float)v[5]}};
    }
    double t[16];
    if (!numbers_after(buf, "\"ImuToDevice\"", t, 16)) return false;
    double qw, qx, qy, qz, tr = t[0] + t[5] + t[10];
    if (tr > 0) {
        double s = sqrt(tr + 1.0) * 2;
        qw = 0.25 * s; qx = (t[9] - t[6]) / s; qy = (t[2] - t[8]) / s; qz = (t[4] - t[1]) / s;
    } else if (t[0] > t[5] && t[0] > t[10]) {
        double s = sqrt(1.0 + t[0] - t[5] - t[10]) * 2;
        qw = (t[9] - t[6]) / s; qx = 0.25 * s; qy = (t[1] + t[4]) / s; qz = (t[2] + t[8]) / s;
    } else if (t[5] > t[10]) {
        double s = sqrt(1.0 + t[5] - t[0] - t[10]) * 2;
        qw = (t[2] - t[8]) / s; qx = (t[1] + t[4]) / s; qy = 0.25 * s; qz = (t[6] + t[9]) / s;
    } else {
        double s = sqrt(1.0 + t[10] - t[0] - t[5]) * 2;
        qw = (t[4] - t[1]) / s; qx = (t[2] + t[8]) / s; qy = (t[6] + t[9]) / s; qz = 0.25 * s;
    }
    const float imu7[8] = {(float)qx, (float)qy, (float)qz, (float)qw, (float)t[3], (float)t[7], (float)t[11], 0.0f};
    memcpy(m.imu7, imu7, sizeof(imu7));
    return m.hand >= 0 && m.n > 0;
}

// Controllers are optional: without them the head works as before.
static void init_ctrl() {
    char dll[MAX_PATH], db[MAX_PATH];
    snprintf(dll, sizeof(dll), "%s\\LedObjectPoseEstimator.dll", g_dlls);
    snprintf(db, sizeof(db), "%s\\pimax_database.bin", g_slam);
    HMODULE h = LoadLibraryA(dll);
    if (!h) { fprintf(stderr, "controllers off: LoadLibrary %s -> %lu\n", dll, GetLastError()); return; }
    auto initial = (InitialFn)GetProcAddress(h, "ControllerInitialImpl");
    auto enable = (SetByteFn)GetProcAddress(h, "ControllerSetDeviceEnable");
    auto add = (CtrlAddFn)GetProcAddress(h, "ControllerAddObject");
    g_ctrlCam = (CtrlPutCamFn)GetProcAddress(h, "ControllerPutCameraImage");
    g_ctrlImu = (CtrlPutImuFn)GetProcAddress(h, "ControllerPutControllerImuData");
    if (!initial || !enable || !add || !g_ctrlCam || !g_ctrlImu) {
        fprintf(stderr, "controllers off: not all LED engine exports found\n"); return;
    }
    const int r = initial(g_calib, g_slam, db, nullptr, nullptr);
    const int e = enable(1);
    printf("controllers: Initial=%d SetDeviceEnable(1)=%d\n", r, e);
    if (r != 0 || e != 0) return;
    // models: slam\H1\<controller serial>\controller_params.json (from ProgramData\Pimax\slam\H1)
    char pat[MAX_PATH];
    snprintf(pat, sizeof(pat), "%s\\H1\\*", g_slam);
    WIN32_FIND_DATAA fd;
    HANDLE fh = FindFirstFileA(pat, &fd);
    const MsvcString sub = {"\\H1\\", 4, 15};
    bool have[2] = {false, false};
    while (fh != INVALID_HANDLE_VALUE) {
        if ((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && fd.cFileName[0] != '.') {
            char path[MAX_PATH];
            snprintf(path, sizeof(path), "%s\\H1\\%s\\controller_params.json", g_slam, fd.cFileName);
            CtrlModel m;
            if (load_ctrl_model(path, m) && !have[m.hand]) {
                const int a = add(m.hand, m.n, m.leds, m.imu7, 2, 0, 0x34, &sub);
                printf("controller %s: %d LEDs, AddObject=%d\n", m.hand ? "right" : "left", m.n, a);
                have[m.hand] = (a == 0);
                memcpy(g_devFromImu[m.hand], m.imu7, sizeof(g_devFromImu[m.hand]));
            }
        }
        if (!FindNextFileA(fh, &fd)) { FindClose(fh); break; }
    }
    g_ctrlOn = have[0] || have[1];
    if (!g_ctrlOn) fprintf(stderr, "controllers off: no models in %s\\H1\n", g_slam);
}

// ---- the controller engine on its own thread ----
// The LED engine's calls (ControllerPutCameraImage, ControllerPutControllerImuData) can take tens of milliseconds while
// its matching search is slow. Made inline in the socket loop, they held up the head's IMU and camera messages behind
// them, the client's queue overflowed (dropped messages and frame sets), the engine missed frames, lost the
// controllers and searched even longer. Now one worker thread makes every controller engine call (still one thread for
// that DLL, in arrival order, as before); the socket loop only hands the data over and answers at once:
//   - LED frames: a latest-wins mailbox (3 buffers, no copies); a frame the worker could not start before the next
//     one arrived is replaced (counted), never queued;
//   - controller IMU: a bounded queue (the oldest dropped when full); MSG_CPOSE carries the latest pose the worker got
//     for that hand (the client keeps the last tracked pose and judges its age by its timestamp anyway).
// PIBRIDGE_WB_CTRL_THREAD=0: the old inline calls (A/B).
struct CtrlImuItem { uint32_t hand; uint64_t ts; float accel[3], gyro[3]; };
static bool g_ctrlThread = true;
static SRWLOCK g_ctrlLock = SRWLOCK_INIT;
static CONDITION_VARIABLE g_ctrlCv = CONDITION_VARIABLE_INIT;
static const int kCtrlQueue = 4096;  // ~2 s of both hands' IMU
static CtrlImuItem g_ctrlQ[kCtrlQueue];
static int g_ctrlQHead = 0, g_ctrlQCount = 0, g_ctrlQMax = 0;
static long g_ctrlQDropped = 0, g_ledReplaced = 0;
static uint8_t g_ledBuf[3][4][640 * 480];
static LedHeader g_ledHdr[3];
static int g_ledFill = 0, g_ledWork = 1, g_ledFree = 2, g_ledPending = -1;  // buffer roles (see led_publish/led_take)
static CtrlPosePayload g_ctrlLatest[2];  // the latest pose per hand (tracked = 0 until there is one)
static uint8_t g_black[640 * 480];
static float g_ctrlScratch[2][64];

// One controller IMU sample through the engine; the pose (if the engine gave one this time) into p, p.tracked set.
static void ctrl_imu_process(const CtrlImuItem& in, CtrlPosePayload& p, CallStat& st) {
    p = CtrlPosePayload{};
    p.hand = in.hand;
    p.ts = in.ts;
    memcpy(p.dev_from_imu, g_devFromImu[in.hand], sizeof(p.dev_from_imu));
    ImuSample s{};
    s.ts = in.ts;
    memcpy(s.accel, in.accel, sizeof(s.accel));
    memcpy(s.gyro, in.gyro, sizeof(s.gyro));
    float out[64] = {0};
    const double t0 = now_ms();
    const int imuRet = g_ctrlImu((int)in.hand, &s, out, g_ctrlScratch[in.hand]);
    st.add(now_ms() - t0);
    // DIAG (pibridge): ControllerPutControllerImuData's return + the out[] tail past the pose (we use
    // out[0..12]); one of these may be the per-hand matched-LED count or a tracking confidence.
    static long imuLogN[2] = {0, 0};
    if (++imuLogN[in.hand] % 250 == 0) {
        printf("ctrl %u: ControllerPutControllerImuData=%d, out[13..26] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
               in.hand, imuRet, out[13], out[14], out[15], out[16], out[17], out[18], out[19], out[20],
               out[21], out[22], out[23], out[24], out[25], out[26]);
    }
    // a pose only when the engine says it is valid (out[23] = 1: tracked and a vision update within 66 ms): out[] is
    // filled with the IMU-only propagation otherwise, which drifts (RE of LedObjectPoseEstimator 2.11.2);
    // PIBRIDGE_WB_CTRL_ANY=1: the old test (any non-zero position)
    static const bool any_pose = [] {
        const char* e = getenv("PIBRIDGE_WB_CTRL_ANY");
        return e && strcmp(e, "1") == 0;
    }();
    const bool valid = any_pose ? (out[4] != 0.0f || out[5] != 0.0f || out[6] != 0.0f) : out[23] > 0.5f;
    if (valid) {
        // PIBRIDGE_WB_CTRL_POSE_TS=1: stamp the pose with the engine's own pose time, out[20..21] as u64 ns (the IMU
        // sample time less its 5.4 ms); default: the IMU sample time as before (to A/B it first)
        static const bool pose_ts = [] {
            const char* e = getenv("PIBRIDGE_WB_CTRL_POSE_TS");
            return e && strcmp(e, "1") == 0;
        }();
        uint64_t pts = 0;
        memcpy(&pts, &out[20], sizeof(pts));
        if (pose_ts && pts != 0) {
            p.ts = pts;
        }
        // from LED engine axes into the SLAM world: q = Q^-1 * q_led, vectors (x, -z, y)
        const float qinv[4] = {-kQ[0], -kQ[1], -kQ[2], kQ[3]};
        qmul(qinv, out, p.pose);
        from_led(out + 4, p.pose + 4);
        from_led(out + 7, p.pose + 7);
        from_led(out + 10, p.pose + 10);
        p.tracked = 1;
        // head pose from which the LED engine computed the controller (without head data: identity)
        AcquireSRWLockShared(&g_headLock);
        if (g_headSeen) {
            memcpy(p.head, g_head, 7 * sizeof(float));
        } else {
            const float id[7] = {0, 0, 0, 1, 0, 0, 0};
            memcpy(p.head, id, sizeof(id));
        }
        ReleaseSRWLockShared(&g_headLock);
    }
}

// One set of LED frames through the engine, projected with the head pose at the frames' capture time.
static void led_process(const LedHeader& lh, uint8_t (*leds)[640 * 480], CallStat& st) {
    const uint32_t n = lh.ncam > 4 ? 4 : lh.ncam;
    LedFrame lf[4];
    for (int i = 0; i < 4; ++i)
        lf[i] = LedFrame{lh.ts, lh.shutter_ns ? lh.shutter_ns : 40000u, (uint32_t)i, lh.w, lh.h, lh.w, 0,
                         (uint32_t)i < n ? leds[i] : g_black};
    // head pose at the frame time (latest from SLAM) in LED engine axes; without head data -- identity
    // (controller poses are then relative to the headset)
    HeadPoseForLed hp{};
    hp.q[3] = 1.0f;
    float head[13];
    bool seen;
    uint64_t usedTs;
    AcquireSRWLockShared(&g_headLock);
    if (g_ledHeadAlign) {
        head_at(lh.ts, head);
    } else {
        memcpy(head, g_head, sizeof(head));
    }
    seen = g_headSeen;
    usedTs = g_ledHeadAlign ? g_headUsedTs : g_headTs;
    ReleaseSRWLockShared(&g_headLock);
    if (seen) {
        qmul(kQ, head, hp.q);
        to_led(head + 4, hp.p);
    }
    // PIBRIDGE_WB_CAM_ARG6=1: the sixth argument a zeroed buffer instead of nullptr (Pimax Play passes a heap object
    // there, frida pass 5; A/B for the LED engine's growth over a session)
    static const bool arg6 = [] {
        const char* e = getenv("PIBRIDGE_WB_CAM_ARG6");
        if (e && strcmp(e, "1") == 0) {
            printf("LED engine: ControllerPutCameraImage gets a zeroed 1 MiB buffer as its sixth argument\n");
            return true;
        }
        return false;
    }();
    static uint8_t* arg6buf = arg6 ? static_cast<uint8_t*>(calloc(1, 1 << 20)) : nullptr;
    const double t0 = now_ms();
    const int camRet = g_ctrlCam(&lf[0], &lf[1], &lf[2], &lf[3], &hp, arg6buf);
    st.add(now_ms() - t0);
    // DIAG (pibridge): ControllerPutCameraImage's return -- the candidate for how many LED points the
    // cameras matched this frame. Logged ~1/s so we can see it drop on fast swings / bad angles.
    static long ledFrameN = 0;
    if (++ledFrameN % 90 == 0) {
        // Head pose fed into the LED projection: how stale it is vs the LED frame (during a head turn
        // this delta times the head's angular velocity is a controller error that follows head motion),
        // the raw SLAM head pose, and the LED-axis pose actually passed to the engine.
        const double stale_ms = (double)(int64_t)(lh.ts - usedTs) / 1e6;
        printf("LED optical: ControllerPutCameraImage=%d (frame %ld); head stale %.2f ms (align=%d); "
               "slam q[%.4f %.4f %.4f %.4f] p[%.3f %.3f %.3f]; led q[%.4f %.4f %.4f %.4f] p[%.3f %.3f %.3f]\n",
               camRet, ledFrameN, stale_ms, g_ledHeadAlign ? 1 : 0, head[0], head[1], head[2], head[3],
               head[4], head[5], head[6], hp.q[0], hp.q[1], hp.q[2], hp.q[3], hp.p[0], hp.p[1], hp.p[2]);
    }
}

// The socket loop finished receiving LED frames into g_ledBuf[g_ledFill]: hand them to the worker. A frame set the
// worker has not started yet is replaced (its buffer becomes the next fill buffer).
static void led_publish() {
    AcquireSRWLockExclusive(&g_ctrlLock);
    if (g_ledPending >= 0) {
        const int old = g_ledPending;
        g_ledPending = g_ledFill;
        g_ledFill = old;
        ++g_ledReplaced;
    } else {
        g_ledPending = g_ledFill;
        g_ledFill = g_ledFree;
        g_ledFree = -1;
    }
    ReleaseSRWLockExclusive(&g_ctrlLock);
    WakeConditionVariable(&g_ctrlCv);
}

static void ctrl_imu_push(const CtrlImuItem& it) {
    AcquireSRWLockExclusive(&g_ctrlLock);
    if (g_ctrlQCount == kCtrlQueue) {  // the worker is far behind: the oldest sample goes
        g_ctrlQHead = (g_ctrlQHead + 1) % kCtrlQueue;
        --g_ctrlQCount;
        ++g_ctrlQDropped;
    }
    g_ctrlQ[(g_ctrlQHead + g_ctrlQCount) % kCtrlQueue] = it;
    ++g_ctrlQCount;
    if (g_ctrlQCount > g_ctrlQMax) g_ctrlQMax = g_ctrlQCount;
    ReleaseSRWLockExclusive(&g_ctrlLock);
    WakeConditionVariable(&g_ctrlCv);
}

// A new client: no queued samples, no pending frames, no poses from the last connection.
static void ctrl_reset() {
    AcquireSRWLockExclusive(&g_ctrlLock);
    g_ctrlQHead = g_ctrlQCount = 0;
    if (g_ledPending >= 0) { g_ledFree = g_ledPending; g_ledPending = -1; }
    g_ctrlLatest[0] = CtrlPosePayload{};
    g_ctrlLatest[1] = CtrlPosePayload{};
    ReleaseSRWLockExclusive(&g_ctrlLock);
}

static DWORD WINAPI ctrl_worker(void*) {
    CallStat camSt{"ControllerPutCameraImage"}, imuSt{"ControllerPutControllerImuData"};
    double lastPrint = now_ms();
    CtrlImuItem batch[256];
    for (;;) {
        int n = 0;
        bool led = false;
        AcquireSRWLockExclusive(&g_ctrlLock);
        while (g_ctrlQCount == 0 && g_ledPending < 0) {
            SleepConditionVariableSRW(&g_ctrlCv, &g_ctrlLock, 1000, 0);
            if (now_ms() - lastPrint >= 5000) break;
        }
        // the IMU samples queued so far (in arrival order), then the newest LED frame set: the samples up to the
        // frames' time are in the engine before the frames, as when they came in one by one
        while (g_ctrlQCount > 0 && n < 256) {
            batch[n++] = g_ctrlQ[g_ctrlQHead];
            g_ctrlQHead = (g_ctrlQHead + 1) % kCtrlQueue;
            --g_ctrlQCount;
        }
        if (g_ctrlQCount == 0 && g_ledPending >= 0) {
            g_ledFree = g_ledWork;
            g_ledWork = g_ledPending;
            g_ledPending = -1;
            led = true;
        }
        ReleaseSRWLockExclusive(&g_ctrlLock);
        for (int i = 0; i < n; ++i) {
            CtrlPosePayload p;
            ctrl_imu_process(batch[i], p, imuSt);
            if (p.tracked) {
                AcquireSRWLockExclusive(&g_ctrlLock);
                g_ctrlLatest[p.hand] = p;
                ReleaseSRWLockExclusive(&g_ctrlLock);
            }
        }
        if (led) led_process(g_ledHdr[g_ledWork], g_ledBuf[g_ledWork], camSt);
        if (now_ms() - lastPrint >= 5000) {
            AcquireSRWLockExclusive(&g_ctrlLock);
            const int qmax = g_ctrlQMax;
            const long dropped = g_ctrlQDropped, replaced = g_ledReplaced;
            g_ctrlQMax = 0;
            g_ctrlQDropped = g_ledReplaced = 0;
            ReleaseSRWLockExclusive(&g_ctrlLock);
            printf("timing 5 s, controller worker:");
            camSt.print_and_reset();
            imuSt.print_and_reset();
            printf(" IMU queue max %d, dropped %ld; LED frame sets replaced %ld\n", qmax, dropped, replaced);
            lastPrint = now_ms();
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    int port = (argc > 1) ? atoi(argv[1]) : WINBRIDGE_PORT;
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (const char* a = getenv("PIBRIDGE_LED_HEAD_ALIGN")) g_ledHeadAlign = (atoi(a) != 0);
    if (const char* a = getenv("PIBRIDGE_WB_CTRL_THREAD")) g_ctrlThread = (atoi(a) != 0);
    if (!find_kit() || !init_slam()) return 1;
    init_ctrl();
    // PIBRIDGE_WB_ALLOC=1: which code in the Pimax DLLs keeps heap memory, once a minute (allocwatch.h)
    if (const char* aw = getenv("PIBRIDGE_WB_ALLOC"); aw && strcmp(aw, "1") == 0) {
        if (const char* ev = getenv("PIBRIDGE_WB_ALLOC_EVERY"); ev && atoi(ev) > 0) {
            allocwatch::every = static_cast<unsigned>(atoi(ev));
        }
        const int n = allocwatch::install();
        printf("allocwatch: %d heap imports hooked in all, a report every 60 s\n", n);
        allocwatch::start_reporter(60000);
    }
    if (g_ctrlOn && g_ctrlThread) {
        HANDLE th = CreateThread(nullptr, 0, ctrl_worker, nullptr, 0, nullptr);
        if (th) {
            CloseHandle(th);
        } else {
            g_ctrlThread = false;
        }
    }
    printf("controller engine: %s\n", !g_ctrlOn ? "off" : g_ctrlThread ? "own thread" : "inline (PIBRIDGE_WB_CTRL_THREAD=0)");

    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    SOCKET ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    BOOL one = TRUE;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one));
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons((u_short)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(ls, (sockaddr*)&a, sizeof(a)) != 0 || listen(ls, 1) != 0) {
        fprintf(stderr, "bind/listen 127.0.0.1:%d -> %d\n", port, WSAGetLastError()); return 1;
    }
    printf("winbridge listening on 127.0.0.1:%d\n", port);

    static uint8_t frames[4][640 * 480];
    uint8_t (&black)[640 * 480] = g_black;
    CallStat slamCamSt{"HeadsetPutCameraImage"}, slamImuSt{"HeadsetPutHmdImuData"};
    CallStat inlineCamSt{"ControllerPutCameraImage"}, inlineImuSt{"ControllerPutControllerImuData"};
    for (;;) {
        SOCKET c = accept(ls, nullptr, nullptr);
        if (c == INVALID_SOCKET) continue;
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
        printf("client connected\n");
        ctrl_reset();
        double lastPrint = now_ms();
        long imuN = 0, camN = 0, ledN = 0, cimuN[2] = {0, 0}, ctrackN[2] = {0, 0};
        MsgHeader h;
        while (recv_all(c, &h, sizeof(h))) {
            if (now_ms() - lastPrint >= 5000) {
                printf("timing 5 s, socket loop:");
                slamCamSt.print_and_reset();
                slamImuSt.print_and_reset();
                if (!g_ctrlThread) {
                    inlineCamSt.print_and_reset();
                    inlineImuSt.print_and_reset();
                }
                printf("\n");
                lastPrint = now_ms();
            }
            if (h.type == MSG_IMU && h.len == sizeof(ImuPayload)) {
                ImuPayload in;
                if (!recv_all(c, &in, sizeof(in))) break;
                ImuSample s{};
                s.ts = in.ts;
                memcpy(s.accel, in.accel, sizeof(s.accel));
                memcpy(s.gyro, in.gyro, sizeof(s.gyro));
                float out[32] = {0};
                const double t0 = now_ms();
                g_putImu(&s, out);
                slamImuSt.add(now_ms() - t0);
                AcquireSRWLockExclusive(&g_headLock);
                memcpy(g_head, out, sizeof(g_head));
                g_headSeen = true;
                g_headTs = in.ts;
                memcpy(g_headRing[g_headRingPos], out, sizeof(g_head));
                g_headRingTs[g_headRingPos] = in.ts;
                g_headRingPos = (g_headRingPos + 1) % kHeadRing;
                ReleaseSRWLockExclusive(&g_headLock);
                PosePayload p;
                p.ts = in.ts;
                memcpy(p.pose, out, sizeof(p.pose));
                if (!send_msg(c, MSG_POSE, &p, sizeof(p))) break;
                ++imuN;
            } else if (h.type == MSG_CAM && h.len >= sizeof(CamHeader)) {
                CamHeader ch;
                if (!recv_all(c, &ch, sizeof(ch))) break;
                const uint32_t n = ch.ncam > 4 ? 4 : ch.ncam;
                if (ch.w != 640 || ch.h != 480 || n == 0) { fprintf(stderr, "bad frame %ux%u x%u\n", ch.w, ch.h, ch.ncam); break; }
                bool ok = true;
                for (uint32_t i = 0; i < ch.ncam && ok; ++i) ok = recv_all(c, i < 4 ? frames[i] : frames[3], (int)(ch.w * ch.h));
                if (!ok) break;
                // SLAM expects all 4 cameras (as from pi_server). A missing camera (old 2-camera recording)
                // gets a black frame: with no points it contributes nothing, while another camera's image would give false geometry.
                CamFrame cam[4];
                for (int i = 0; i < 4; ++i)
                    cam[i] = CamFrame{ch.ts, ch.shutter_ns, (uint32_t)i, ch.w, ch.h, ch.w, (uint32_t)i < n ? frames[i] : black};
                const double t0 = now_ms();
                g_putCam(&cam[0], &cam[1], &cam[2], &cam[3], 0.019f);
                slamCamSt.add(now_ms() - t0);
                if (!send_msg(c, MSG_ACK, nullptr, 0)) break;
                ++camN;
            } else if (h.type == MSG_LED && h.len >= sizeof(LedHeader)) {
                // received straight into the fill buffer of the worker's mailbox
                LedHeader& lh = g_ledHdr[g_ledFill];
                if (!recv_all(c, &lh, sizeof(lh))) break;
                const uint32_t n = lh.ncam > 4 ? 4 : lh.ncam;
                if (lh.w != 640 || lh.h != 480 || n == 0) { fprintf(stderr, "bad LED frame %ux%u x%u\n", lh.w, lh.h, lh.ncam); break; }
                bool ok = true;
                uint8_t (*leds)[640 * 480] = g_ledBuf[g_ledFill];
                for (uint32_t i = 0; i < lh.ncam && ok; ++i) ok = recv_all(c, i < 4 ? leds[i] : leds[3], (int)(lh.w * lh.h));
                if (!ok) break;
                if (g_ctrlOn) {
                    if (g_ctrlThread) {
                        led_publish();
                    } else {
                        led_process(lh, leds, inlineCamSt);
                    }
                }
                if (!send_msg(c, MSG_ACK, nullptr, 0)) break;
                ++ledN;
            } else if (h.type == MSG_CIMU && h.len == sizeof(CtrlImuPayload)) {
                CtrlImuPayload in;
                if (!recv_all(c, &in, sizeof(in))) break;
                CtrlPosePayload p{};
                p.hand = in.hand;
                p.ts = in.ts;
                if (in.hand < 2) memcpy(p.dev_from_imu, g_devFromImu[in.hand], sizeof(p.dev_from_imu));
                if (g_ctrlOn && in.hand < 2) {
                    CtrlImuItem it{in.hand, in.ts, {in.accel[0], in.accel[1], in.accel[2]},
                                   {in.gyro[0], in.gyro[1], in.gyro[2]}};
                    if (g_ctrlThread) {
                        // answered with the latest pose the worker has for this hand (its own timestamp)
                        ctrl_imu_push(it);
                        AcquireSRWLockShared(&g_ctrlLock);
                        if (g_ctrlLatest[in.hand].tracked) p = g_ctrlLatest[in.hand];
                        ReleaseSRWLockShared(&g_ctrlLock);
                    } else {
                        ctrl_imu_process(it, p, inlineImuSt);
                    }
                    if (p.tracked) ++ctrackN[in.hand];
                    ++cimuN[in.hand];
                }
                if (!send_msg(c, MSG_CPOSE, &p, sizeof(p))) break;
            } else if (h.type == MSG_ROOMREQ && h.len == 0) {
                // the room state: the Pimax engine's own floor and localization getters. The engine writes a small
                // struct into the out buffer (flag + floats for the floor, one byte for the localization state); give
                // it a generous zeroed buffer and forward the first bytes verbatim.
                RoomStatePayload p{};
                if (g_getGround) {
                    uint8_t buf[256] = {0};
                    p.ground_ret = g_getGround(buf);
                    memcpy(p.ground, buf, sizeof(p.ground));
                } else {
                    p.ground_ret = -2;
                }
                if (g_getLoc) {
                    uint8_t buf[64] = {0};
                    p.loc_ret = g_getLoc(buf);
                    p.loc_state = buf[0];
                } else {
                    p.loc_ret = -2;
                }
                if (!send_msg(c, MSG_ROOMSTATE, &p, sizeof(p))) break;
            } else {
                fprintf(stderr, "unknown message type=%u len=%u\n", h.type, h.len);
                break;
            }
        }
        printf("client disconnected: imu=%ld cam=%ld led=%ld; controllers imu %ld/%ld, with pose %ld/%ld\n", imuN, camN,
               ledN, cimuN[0], cimuN[1], ctrackN[0], ctrackN[1]);
        closesocket(c);
    }
}
