// libpibridge_vit_winbridge.so: a VIT tracker for Monado backed by the real Pimax head SLAM
// (pimax_slam.pi.dll under Wine, the winbridge.exe process, proto.h over TCP localhost).
//
// Monado: PIMAX_SLAM=1 VIT_SYSTEM_LIBRARY_PATH=<path>/libpibridge_vit_winbridge.so
// The Pimax driver is unchanged: this is just a different VIT library in place of libbasalt.so.
//
// Threads: push_imu/push_img are called from Monado threads and only enqueue data;
// one worker thread owns the socket and runs the lockstep exchange (IMU -> POSE, CAM -> ACK).
// If winbridge is unavailable, data is dropped and the thread reconnects once a second:
// Monado stays in 3DoF, VR does not stop.
//
// Environment: PIBRIDGE_WINBRIDGE_PORT (48291), PIBRIDGE_SHUTTER_NS (10000000; VIT does not pass the exposure,
// and Pimax SLAM drops frames with zero exposure; the value does not affect frame timing, verified at 0.1/10/20 ms),
// PIBRIDGE_VIT_POSE_RATE (cam | imu, default cam: one pose per frame, like Basalt; at IMU rate the driver's output
// layer jitters 1.0 mm in motion vs 0.36 and snaps corrections more often, pibridge-slam headreplay on a recording).
//
// Build: g++ -O2 -std=c++20 -fPIC -shared vit_winbridge.cpp -o libpibridge_vit_winbridge.so -pthread

#define VIT_INTERFACE_IMPLEMENTATION  // exposes the function prototypes (we are the VIT implementation)
#include "vit/vit_interface.h"
#include "proto.h"
#include "pibridge_winbridge_ext.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

namespace {

constexpr uint32_t kW = 640, kH = 480;
constexpr size_t kMaxQueue = 4096;   // ~4 s of IMU; beyond that the oldest is dropped
constexpr uint32_t kMaxFrames = 6;   // frame sets queued (1.2 MB each); extra new ones are dropped

struct Item {
    enum Kind { Imu, Cam, Led, CtrlImu } kind = Imu;
    ImuPayload imu;
    CamHeader ch;
    LedHeader lh;
    CtrlImuPayload cimu;
    std::vector<uint8_t> px;  // ncam*W*H
    bool frame() const { return kind == Cam || kind == Led; }
};

struct PoseOut { vit_pose_data_t d; };

}  // namespace

struct vit_pose { vit_pose_data_t data; };

struct vit_tracker {
    uint32_t cam_count = 2;
    int port = WINBRIDGE_PORT;
    uint64_t shutter_ns = 10000000;
    bool imu_rate_poses = false;  // PIBRIDGE_VIT_POSE_RATE=imu

    std::mutex m;
    std::condition_variable cv;
    std::deque<Item> q;
    std::deque<vit_pose_data_t> poses;

    // collect a pair of frames with the same timestamp
    int64_t pend_ts = -1;
    uint32_t pend_mask = 0;
    std::vector<uint8_t> pend_px;

    std::atomic<bool> running{false};
    std::atomic<bool> quit{false};
    std::thread worker;
    long dropped = 0;
    uint32_t frames_queued = 0;
    long frames_dropped = 0;

    // controllers (pibridge_winbridge_ext.h): the latest pose of each hand
    pibridge_wb_ctrl_pose ctrl[2] = {};
    bool ctrl_tracked[2] = {false, false};

    // The engine tracks in the saved map's frame, whose origin is tens of meters away (head poses ~95 m from 0).
    // Basalt starts at zero; we remove this constant offset on the first pose so that the head, the room map (roommap
    // poses keyframes with the raw SLAM pose) and the anchor live near the origin and do not lose float precision far from zero.
    // Held for the tracker's whole life (reconnect/reset -> same map -> same offset). Controller positions in
    // the driver are relative to the head, we leave them alone.
    bool have_origin = false;
    float origin[3] = {0, 0, 0};
};

// The tracker the driver extension talks to (Monado creates one).
static std::atomic<vit_tracker*> g_tracker{nullptr};

namespace {

bool send_all(int s, const void* b, size_t n) {
    auto p = static_cast<const char*>(b);
    while (n) { ssize_t r = ::send(s, p, n, MSG_NOSIGNAL); if (r <= 0) return false; p += r; n -= (size_t)r; }
    return true;
}
bool recv_all(int s, void* b, size_t n) {
    auto p = static_cast<char*>(b);
    while (n) { ssize_t r = ::recv(s, p, n, 0); if (r <= 0) return false; p += r; n -= (size_t)r; }
    return true;
}

int connect_bridge(int port) {
    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    if (::connect(s, (sockaddr*)&a, sizeof(a)) != 0) { ::close(s); return -1; }
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return s;
}

// A pose from the Pimax SLAM buffer (quat x,y,z,w; pos; angVel; linVel) -> VIT.
// Quaternion order (x,y,z,w) confirmed: at rest with a=(0,0,g) it gave (~0,~0,~0,1).
bool to_vit(const PosePayload& p, vit_pose_data_t& o) {
    const float* v = p.pose;
    float n = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3];
    bool tracked = (v[4] != 0.0f || v[5] != 0.0f || v[6] != 0.0f) && n > 0.98f && n < 1.02f;
    if (!tracked) return false;  // before localization SLAM gives orientation only
    o.timestamp = (int64_t)p.ts;
    o.px = v[4]; o.py = v[5]; o.pz = v[6];
    o.ox = v[0]; o.oy = v[1]; o.oz = v[2]; o.ow = v[3];
    o.vx = v[10]; o.vy = v[11]; o.vz = v[12];
    return true;
}

// Reply reader for one connection. The server answers every message in order, so the reply kind
// is known from the queue of what was sent (inflight).
void read_replies(vit_tracker* t, int s, std::deque<Item::Kind>* inflight, std::atomic<bool>* alive) {
    bool want_pose = false;  // after a frame's reply, emit the first pose with a position (one pose per frame)
    for (;;) {
        MsgHeader h{};
        if (!recv_all(s, &h, sizeof(h))) break;
        Item::Kind k;
        {
            std::lock_guard<std::mutex> lk(t->m);
            if (inflight->empty()) break;
            k = inflight->front();
            inflight->pop_front();
        }
        if (h.type == MSG_POSE && h.len == sizeof(PosePayload) && k == Item::Imu) {
            PosePayload p{};
            if (!recv_all(s, &p, sizeof(p))) break;
            vit_pose_data_t d{};
            if ((t->imu_rate_poses || want_pose) && to_vit(p, d)) {
                want_pose = false;
                std::lock_guard<std::mutex> lk(t->m);
                if (!t->have_origin) {
                    t->have_origin = true;
                    t->origin[0] = d.px; t->origin[1] = d.py; t->origin[2] = d.pz;
                }
                d.px -= t->origin[0]; d.py -= t->origin[1]; d.pz -= t->origin[2];
                t->poses.push_back(d);
                if (t->poses.size() > kMaxQueue) t->poses.pop_front();
            }
        } else if (h.type == MSG_CPOSE && h.len == sizeof(CtrlPosePayload) && k == Item::CtrlImu) {
            CtrlPosePayload p{};
            if (!recv_all(s, &p, sizeof(p))) break;
            if (p.hand < 2) {
                std::lock_guard<std::mutex> lk(t->m);
                t->ctrl_tracked[p.hand] = p.tracked != 0;
                if (p.tracked) {
                    pibridge_wb_ctrl_pose& c = t->ctrl[p.hand];
                    c.ts_ns = (int64_t)p.ts;
                    memcpy(c.q, p.pose, sizeof(c.q));
                    memcpy(c.p, p.pose + 4, sizeof(c.p));
                    memcpy(c.ang_vel, p.pose + 7, sizeof(c.ang_vel));
                    memcpy(c.lin_vel, p.pose + 10, sizeof(c.lin_vel));
                    memcpy(c.head_q, p.head, sizeof(c.head_q));
                    memcpy(c.head_p, p.head + 4, sizeof(c.head_p));
                }
                memcpy(t->ctrl[p.hand].dev_from_imu, p.dev_from_imu, sizeof(p.dev_from_imu));
            }
        } else if (h.type == MSG_ACK && h.len == 0 && (k == Item::Cam || k == Item::Led)) {
            if (k == Item::Cam) want_pose = true;
        } else {
            fprintf(stderr, "pibridge-vit: unexpected reply type=%u len=%u\n", h.type, h.len);
            break;
        }
    }
    alive->store(false);
    t->cv.notify_all();
}

// Pipeline: this thread only sends (it does not wait for a reply to each message: a round trip through Wine takes longer
// than a millisecond, while the head and two hands' IMUs total several kHz); read_replies reads the replies.
void run(vit_tracker* t) {
    while (!t->quit) {
        const int s = connect_bridge(t->port);
        if (s < 0) {
            {   // unavailable: drop what accumulated, wait
                std::lock_guard<std::mutex> lk(t->m);
                t->dropped += (long)t->q.size();
                t->q.clear();
                t->frames_queued = 0;
                t->ctrl_tracked[0] = t->ctrl_tracked[1] = false;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        fprintf(stderr, "pibridge-vit: connected to winbridge 127.0.0.1:%d\n", t->port);
        std::deque<Item::Kind> inflight;
        std::atomic<bool> alive{true};
        std::thread reader(read_replies, t, s, &inflight, &alive);
        while (!t->quit && alive) {
            Item it;
            {
                std::unique_lock<std::mutex> lk(t->m);
                t->cv.wait_for(lk, std::chrono::milliseconds(100), [&] { return t->quit || !alive || !t->q.empty(); });
                if (t->quit || !alive) break;
                if (t->q.empty()) continue;
                it = std::move(t->q.front());
                t->q.pop_front();
                if (it.frame()) --t->frames_queued;
                inflight.push_back(it.kind);
            }
            bool ok;
            switch (it.kind) {
            case Item::Imu: {
                const MsgHeader h{MSG_IMU, sizeof(ImuPayload)};
                ok = send_all(s, &h, sizeof(h)) && send_all(s, &it.imu, sizeof(it.imu));
                break;
            }
            case Item::CtrlImu: {
                const MsgHeader h{MSG_CIMU, sizeof(CtrlImuPayload)};
                ok = send_all(s, &h, sizeof(h)) && send_all(s, &it.cimu, sizeof(it.cimu));
                break;
            }
            case Item::Cam: {
                const MsgHeader h{MSG_CAM, (uint32_t)(sizeof(CamHeader) + it.px.size())};
                ok = send_all(s, &h, sizeof(h)) && send_all(s, &it.ch, sizeof(it.ch)) &&
                     send_all(s, it.px.data(), it.px.size());
                break;
            }
            case Item::Led:
            default: {
                const MsgHeader h{MSG_LED, (uint32_t)(sizeof(LedHeader) + it.px.size())};
                ok = send_all(s, &h, sizeof(h)) && send_all(s, &it.lh, sizeof(it.lh)) &&
                     send_all(s, it.px.data(), it.px.size());
                break;
            }
            }
            if (!ok) break;
        }
        if (!t->quit) fprintf(stderr, "pibridge-vit: winbridge connection lost, reconnecting\n");
        ::shutdown(s, SHUT_RDWR);  // wakes the reader
        reader.join();
        ::close(s);
        std::lock_guard<std::mutex> lk(t->m);
        t->ctrl_tracked[0] = t->ctrl_tracked[1] = false;
    }
}

void enqueue(vit_tracker* t, Item&& it) {
    {
        std::lock_guard<std::mutex> lk(t->m);
        if (it.frame()) {
            if (t->frames_queued >= kMaxFrames) {  // the bridge is lagging: this frame set is dropped
                ++t->frames_dropped;
                return;
            }
            ++t->frames_queued;
        }
        if (t->q.size() >= kMaxQueue) {
            if (t->q.front().frame()) --t->frames_queued;
            t->q.pop_front();
            ++t->dropped;
        }
        t->q.push_back(std::move(it));
    }
    t->cv.notify_one();
}

}  // namespace

extern "C" {

vit_result_t vit_api_get_version(uint32_t* major, uint32_t* minor, uint32_t* patch) {
    *major = VIT_HEADER_VERSION_MAJOR;
    *minor = VIT_HEADER_VERSION_MINOR;
    *patch = VIT_HEADER_VERSION_PATCH;
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_create(const vit_config_t* config, vit_tracker_t** out) {
    if (!config || !out || config->cam_count == 0 || config->cam_count > 4) return VIT_ERROR_INVALID_VALUE;
    auto* t = new (std::nothrow) vit_tracker;
    if (!t) return VIT_ERROR_ALLOCATION_FAILURE;
    t->cam_count = config->cam_count;
    if (const char* e = getenv("PIBRIDGE_WINBRIDGE_PORT")) t->port = atoi(e);
    if (const char* e = getenv("PIBRIDGE_SHUTTER_NS")) t->shutter_ns = strtoull(e, nullptr, 10);
    if (const char* e = getenv("PIBRIDGE_VIT_POSE_RATE")) t->imu_rate_poses = strcmp(e, "imu") == 0;
    t->pend_px.assign((size_t)t->cam_count * kW * kH, 0);
    *out = t;
    g_tracker = t;
    return VIT_SUCCESS;
}

void vit_tracker_destroy(vit_tracker_t* t) {
    if (!t) return;
    vit_tracker_t* expected = t;
    g_tracker.compare_exchange_strong(expected, nullptr);
    vit_tracker_stop(t);
    delete t;
}

vit_result_t vit_tracker_has_image_format(const vit_tracker_t*, vit_image_format_t f, bool* out) {
    *out = (f == VIT_IMAGE_FORMAT_L8);
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_get_supported_extensions(const vit_tracker_t*, vit_tracker_extension_set_t* out) {
    // We accept and ignore calibration: Pimax SLAM takes its own from device_calibration_<SN>.xml.
    *out = {true, true, false, false};
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_get_enabled_extensions(const vit_tracker_t* t, vit_tracker_extension_set_t* out) {
    return vit_tracker_get_supported_extensions(t, out);
}

vit_result_t vit_tracker_enable_extension(vit_tracker_t*, vit_tracker_extension_t ext, bool value) {
    if (ext == VIT_TRACKER_EXTENSION_ADD_CAMERA_CALIBRATION || ext == VIT_TRACKER_EXTENSION_ADD_IMU_CALIBRATION)
        return VIT_SUCCESS;
    return value ? VIT_ERROR_NOT_SUPPORTED : VIT_SUCCESS;
}

vit_result_t vit_tracker_start(vit_tracker_t* t) {
    if (t->running) return VIT_SUCCESS;
    t->quit = false;
    t->worker = std::thread(run, t);
    t->running = true;
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_stop(vit_tracker_t* t) {
    if (!t->running) return VIT_SUCCESS;
    t->quit = true;
    t->cv.notify_all();
    if (t->worker.joinable()) t->worker.join();
    t->running = false;
    fprintf(stderr, "pibridge-vit: stopped; dropped %ld queued messages, %ld frame sets (bridge too slow or down)\n",
            t->dropped, t->frames_dropped);
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_reset(vit_tracker_t* t) {
    std::lock_guard<std::mutex> lk(t->m);
    t->q.clear();
    t->frames_queued = 0;
    t->poses.clear();
    t->pend_ts = -1;
    t->pend_mask = 0;
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_is_running(const vit_tracker_t* t, bool* out) {
    *out = t->running;
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_push_imu_sample(vit_tracker_t* t, const vit_imu_sample_t* s) {
    if (!t->running) return VIT_SUCCESS;
    Item it{};
    it.kind = Item::Imu;
    it.imu.ts = (uint64_t)s->timestamp;
    it.imu.accel[0] = s->ax; it.imu.accel[1] = s->ay; it.imu.accel[2] = s->az;
    it.imu.gyro[0] = s->wx;  it.imu.gyro[1] = s->wy;  it.imu.gyro[2] = s->wz;
    enqueue(t, std::move(it));
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_push_img_sample(vit_tracker_t* t, const vit_img_sample_t* s) {
    if (!t->running) return VIT_SUCCESS;
    if (s->format != VIT_IMAGE_FORMAT_L8 || s->width != kW || s->height != kH || s->cam_index >= t->cam_count)
        return VIT_ERROR_INVALID_VALUE;
    Item it{};
    bool full = false;
    {
        std::lock_guard<std::mutex> lk(t->m);
        if (s->timestamp != t->pend_ts) {  // new timestamp: an unfinished pair is dropped
            t->pend_ts = s->timestamp;
            t->pend_mask = 0;
        }
        uint8_t* dst = t->pend_px.data() + (size_t)s->cam_index * kW * kH;
        for (uint32_t y = 0; y < kH; ++y) memcpy(dst + (size_t)y * kW, s->data + (size_t)y * s->stride, kW);
        t->pend_mask |= 1u << s->cam_index;
        if (t->pend_mask == (1u << t->cam_count) - 1) {
            full = true;
            it.kind = Item::Cam;
            it.ch.ts = (uint64_t)s->timestamp;
            it.ch.shutter_ns = t->shutter_ns;
            it.ch.w = kW; it.ch.h = kH; it.ch.ncam = t->cam_count;
            it.px = t->pend_px;
            t->pend_mask = 0;
            t->pend_ts = -1;
        }
    }
    if (full) enqueue(t, std::move(it));
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_add_imu_calibration(vit_tracker_t*, const vit_imu_calibration_t*) { return VIT_SUCCESS; }
vit_result_t vit_tracker_add_camera_calibration(vit_tracker_t*, const vit_camera_calibration_t*) { return VIT_SUCCESS; }

vit_result_t vit_tracker_pop_pose(vit_tracker_t* t, vit_pose_t** out) {
    std::lock_guard<std::mutex> lk(t->m);
    if (t->poses.empty()) { *out = nullptr; return VIT_SUCCESS; }
    auto* p = new (std::nothrow) vit_pose;
    if (!p) return VIT_ERROR_ALLOCATION_FAILURE;
    p->data = t->poses.front();
    t->poses.pop_front();
    *out = p;
    return VIT_SUCCESS;
}

vit_result_t vit_tracker_get_timing_titles(const vit_tracker_t*, vit_tracker_timing_titles*) {
    return VIT_ERROR_NOT_SUPPORTED;
}

void vit_pose_destroy(vit_pose_t* p) { delete p; }

vit_result_t vit_pose_get_data(const vit_pose_t* p, vit_pose_data_t* out) {
    *out = p->data;
    return VIT_SUCCESS;
}

vit_result_t vit_pose_get_timing(const vit_pose_t*, vit_pose_timing_t*) { return VIT_ERROR_NOT_SUPPORTED; }

vit_result_t vit_pose_get_features(const vit_pose_t*, uint32_t, vit_pose_features_t*) {
    return VIT_ERROR_NOT_SUPPORTED;
}

// ---- extension for the Pimax driver: controllers (pibridge_winbridge_ext.h) ----

__attribute__((visibility("default"))) int pibridge_wb_ext_version(void) { return PIBRIDGE_WB_EXT_VERSION; }

__attribute__((visibility("default"))) int pibridge_wb_push_led(int64_t ts_ns, uint32_t shutter_ns, uint32_t ncam,
                                                             const uint8_t* const* tiles, const uint32_t* strides) {
    vit_tracker* t = g_tracker.load();
    if (!t || !t->running || ncam == 0 || ncam > 4 || !tiles) return -1;
    Item it{};
    it.kind = Item::Led;
    it.lh = LedHeader{(uint64_t)ts_ns, shutter_ns, kW, kH, ncam};
    it.px.resize((size_t)ncam * kW * kH);
    for (uint32_t c = 0; c < ncam; ++c) {
        const uint32_t stride = strides ? strides[c] : kW;
        uint8_t* dst = it.px.data() + (size_t)c * kW * kH;
        for (uint32_t y = 0; y < kH; ++y) memcpy(dst + (size_t)y * kW, tiles[c] + (size_t)y * stride, kW);
    }
    enqueue(t, std::move(it));
    return 0;
}

__attribute__((visibility("default"))) int pibridge_wb_push_ctrl_imu(uint32_t hand, int64_t ts_ns, const float accel[3],
                                                                  const float gyro[3]) {
    vit_tracker* t = g_tracker.load();
    if (!t || !t->running || hand > 1) return -1;
    Item it{};
    it.kind = Item::CtrlImu;
    it.cimu.hand = hand;
    it.cimu.ts = (uint64_t)ts_ns;
    memcpy(it.cimu.accel, accel, sizeof(it.cimu.accel));
    memcpy(it.cimu.gyro, gyro, sizeof(it.cimu.gyro));
    enqueue(t, std::move(it));
    return 0;
}

__attribute__((visibility("default"))) int pibridge_wb_ctrl_pose(uint32_t hand, pibridge_wb_ctrl_pose* out) {
    vit_tracker* t = g_tracker.load();
    if (!t || hand > 1 || !out) return 0;
    std::lock_guard<std::mutex> lk(t->m);
    if (!t->ctrl_tracked[hand]) return 0;
    *out = t->ctrl[hand];
    return 1;
}

}  // extern "C"
