// pibridge winbridge server: hosts the real Pimax Play tracking engines under Wine -- head SLAM
// (pimax_slam.pi.dll) and controller LED tracking (LedObjectPoseEstimator.dll) -- and serves one
// Linux client (libpibridge_vit_winbridge.so in Monado) over TCP localhost (proto.h).
// Build: x86_64-w64-mingw32-g++ -O2 -static server.cpp -lws2_32 -o winbridge.exe
// Run: WINEDEBUG=-all wine winbridge.exe [port]
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
#include <cstdlib>
#include <cstring>
#include "proto.h"

typedef int (*InitialFn)(const char*, const char*, const char*, void*, void*);
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
// well under 1 ms). Falls back to the latest pose while the ring is still filling.
static const float* head_at(uint64_t ts) {
    const float* best = g_head;
    uint64_t bestTs = g_headTs, bestd = UINT64_MAX;
    for (int i = 0; i < kHeadRing; ++i) {
        if (g_headRingTs[i] == 0) continue;
        uint64_t d = g_headRingTs[i] > ts ? g_headRingTs[i] - ts : ts - g_headRingTs[i];
        if (d < bestd) { bestd = d; best = g_headRing[i]; bestTs = g_headRingTs[i]; }
    }
    g_headUsedTs = bestTs;
    return best;
}

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
    int r = initial(g_calib, g_slam, vocab, nullptr, nullptr);
    int m = setMode(1);
    int e = enable(1);
    printf("SLAM: Initial=%d SetTrackingMode(1)=%d SetDeviceEnable(1)=%d\n", r, m, e);
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

int main(int argc, char** argv) {
    int port = (argc > 1) ? atoi(argv[1]) : WINBRIDGE_PORT;
    setvbuf(stdout, nullptr, _IONBF, 0);
    if (const char* a = getenv("PIBRIDGE_LED_HEAD_ALIGN")) g_ledHeadAlign = (atoi(a) != 0);
    if (!find_kit() || !init_slam()) return 1;
    init_ctrl();

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

    static uint8_t frames[4][640 * 480], leds[4][640 * 480], black[640 * 480];
    static float ctrlScratch[2][64];
    for (;;) {
        SOCKET c = accept(ls, nullptr, nullptr);
        if (c == INVALID_SOCKET) continue;
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));
        printf("client connected\n");
        long imuN = 0, camN = 0, ledN = 0, cimuN[2] = {0, 0}, ctrackN[2] = {0, 0};
        MsgHeader h;
        while (recv_all(c, &h, sizeof(h))) {
            if (h.type == MSG_IMU && h.len == sizeof(ImuPayload)) {
                ImuPayload in;
                if (!recv_all(c, &in, sizeof(in))) break;
                ImuSample s{};
                s.ts = in.ts;
                memcpy(s.accel, in.accel, sizeof(s.accel));
                memcpy(s.gyro, in.gyro, sizeof(s.gyro));
                float out[32] = {0};
                g_putImu(&s, out);
                memcpy(g_head, out, sizeof(g_head));
                g_headSeen = true;
                g_headTs = in.ts;
                memcpy(g_headRing[g_headRingPos], out, sizeof(g_head));
                g_headRingTs[g_headRingPos] = in.ts;
                g_headRingPos = (g_headRingPos + 1) % kHeadRing;
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
                g_putCam(&cam[0], &cam[1], &cam[2], &cam[3], 0.019f);
                if (!send_msg(c, MSG_ACK, nullptr, 0)) break;
                ++camN;
            } else if (h.type == MSG_LED && h.len >= sizeof(LedHeader)) {
                LedHeader lh;
                if (!recv_all(c, &lh, sizeof(lh))) break;
                const uint32_t n = lh.ncam > 4 ? 4 : lh.ncam;
                if (lh.w != 640 || lh.h != 480 || n == 0) { fprintf(stderr, "bad LED frame %ux%u x%u\n", lh.w, lh.h, lh.ncam); break; }
                bool ok = true;
                for (uint32_t i = 0; i < lh.ncam && ok; ++i) ok = recv_all(c, i < 4 ? leds[i] : leds[3], (int)(lh.w * lh.h));
                if (!ok) break;
                if (g_ctrlOn) {
                    LedFrame lf[4];
                    for (int i = 0; i < 4; ++i)
                        lf[i] = LedFrame{lh.ts, lh.shutter_ns ? lh.shutter_ns : 40000u, (uint32_t)i, lh.w, lh.h, lh.w, 0,
                                         (uint32_t)i < n ? leds[i] : black};
                    // head pose at the frame time (latest from SLAM) in LED engine axes; without head data --
                    // identity (controller poses are then relative to the headset)
                    HeadPoseForLed hp{};
                    hp.q[3] = 1.0f;
                    const float* head = g_ledHeadAlign ? head_at(lh.ts) : g_head;
                    if (g_headSeen) {
                        qmul(kQ, head, hp.q);
                        to_led(head + 4, hp.p);
                    }
                    const int camRet = g_ctrlCam(&lf[0], &lf[1], &lf[2], &lf[3], &hp, nullptr);
                    // DIAG (pibridge): ControllerPutCameraImage's return -- the candidate for how many LED points the
                    // cameras matched this frame. Logged ~1/s so we can see it drop on fast swings / bad angles.
                    static long ledFrameN = 0;
                    if (++ledFrameN % 90 == 0) {
                        // Head pose fed into the LED projection: how stale it is vs the LED frame (during a head turn
                        // this Δt times the head's angular velocity is a controller error that follows head motion),
                        // the raw SLAM head pose, and the LED-axis pose actually passed to the engine.
                        const uint64_t usedTs = g_ledHeadAlign ? g_headUsedTs : g_headTs;
                        const double stale_ms = (double)(int64_t)(lh.ts - usedTs) / 1e6;
                        printf("LED optical: ControllerPutCameraImage=%d (frame %ld); head stale %.2f ms (align=%d); "
                               "slam q[%.4f %.4f %.4f %.4f] p[%.3f %.3f %.3f]; led q[%.4f %.4f %.4f %.4f] p[%.3f %.3f %.3f]\n",
                               camRet, ledFrameN, stale_ms, g_ledHeadAlign ? 1 : 0, head[0], head[1], head[2], head[3],
                               head[4], head[5], head[6], hp.q[0], hp.q[1], hp.q[2], hp.q[3], hp.p[0], hp.p[1], hp.p[2]);
                        fflush(stdout);
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
                    ImuSample s{};
                    s.ts = in.ts;
                    memcpy(s.accel, in.accel, sizeof(s.accel));
                    memcpy(s.gyro, in.gyro, sizeof(s.gyro));
                    float out[64] = {0};
                    const int imuRet = g_ctrlImu((int)in.hand, &s, out, ctrlScratch[in.hand]);
                    // DIAG (pibridge): ControllerPutControllerImuData's return + the out[] tail past the pose (we use
                    // out[0..12]); one of these may be the per-hand matched-LED count or a tracking confidence.
                    static long imuLogN[2] = {0, 0};
                    if (++imuLogN[in.hand] % 250 == 0) {
                        printf("ctrl %d: ControllerPutControllerImuData=%d, out[13..26] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
                               in.hand, imuRet, out[13], out[14], out[15], out[16], out[17], out[18], out[19], out[20],
                               out[21], out[22], out[23], out[24], out[25], out[26]);
                        fflush(stdout);
                    }
                    if (out[4] != 0.0f || out[5] != 0.0f || out[6] != 0.0f) {
                        // from LED engine axes into the SLAM world: q = Q^-1 * q_led, vectors (x, -z, y)
                        const float qinv[4] = {-kQ[0], -kQ[1], -kQ[2], kQ[3]};
                        qmul(qinv, out, p.pose);
                        from_led(out + 4, p.pose + 4);
                        from_led(out + 7, p.pose + 7);
                        from_led(out + 10, p.pose + 10);
                        p.tracked = 1;
                        // head pose from which the LED engine computed the controller (without head data: identity)
                        if (g_headSeen) {
                            memcpy(p.head, g_head, 7 * sizeof(float));
                        } else {
                            const float id[7] = {0, 0, 0, 1, 0, 0, 0};
                            memcpy(p.head, id, sizeof(id));
                        }
                        ++ctrackN[in.hand];
                    }
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
