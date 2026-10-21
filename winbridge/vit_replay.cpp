// Offline test bench for the VIT library: loads the .so via dlopen, like Monado (t_vit_loader: all 21 symbols,
// major version 2), runs replay.bin through the VIT API and writes poses to CSV.
// Coordinate-system check: acceleration (at rest ~ +g up), rotated into the world by the IMU pose,
// should give ~(0,0,+9.8) if the world is Z-up, as t_tracker_slam expects (gravity_correction 0,0,-g).
// Build: g++ -O2 -std=c++20 vit_replay.cpp -o vit-replay -ldl
// Run: vit-replay ./libpibridge_vit_winbridge.so replay.bin poses.csv [speed]
//         vit-replay ./libpibridge_vit_winbridge.so --ctrl ctrl.bin OUT_PREFIX   (controllers via the extension
//         pibridge_winbridge_ext.h, as the driver will call them: LED frames + hand IMU, hand poses in OUT_PREFIX-*.csv)

#include "vit/vit_interface.h"
#include "pibridge_winbridge_ext.h"

#include <dlfcn.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#define SYMS(X) X(api_get_version) X(tracker_create) X(tracker_destroy) X(tracker_has_image_format) \
    X(tracker_get_supported_extensions) X(tracker_get_enabled_extensions) X(tracker_enable_extension)   \
    X(tracker_start) X(tracker_stop) X(tracker_reset) X(tracker_is_running) X(tracker_push_imu_sample)  \
    X(tracker_push_img_sample) X(tracker_add_imu_calibration) X(tracker_add_camera_calibration)         \
    X(tracker_pop_pose) X(tracker_get_timing_titles) X(pose_destroy) X(pose_get_data)                   \
    X(pose_get_timing) X(pose_get_features)

struct Vit {
#define DECL(n) PFN_vit_##n n;
    SYMS(DECL)
#undef DECL
};

// v_world = q * v * q^-1, q = (x,y,z,w)
static void rotate(const float q[4], const float v[3], double o[3]) {
    double x = q[0], y = q[1], z = q[2], w = q[3];
    double tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    o[0] = v[0] + w * tx + (y * tz - z * ty);
    o[1] = v[1] + w * ty + (z * tx - x * tz);
    o[2] = v[2] + w * tz + (x * ty - y * tx);
}

// Controllers via the extension: a ctrl_prep recording ("CREP": LED frames + both hands' IMU) at recording pace.
static int run_ctrl(const Vit& v, void* h, const char* path, const char* prefix) {
    auto ver = (pibridge_wb_ext_version_fn)dlsym(h, "pibridge_wb_ext_version");
    auto led = (pibridge_wb_push_led_fn)dlsym(h, "pibridge_wb_push_led");
    auto imu = (pibridge_wb_push_ctrl_imu_fn)dlsym(h, "pibridge_wb_push_ctrl_imu");
    auto pose = (pibridge_wb_ctrl_pose_fn)dlsym(h, "pibridge_wb_ctrl_pose");
    if (!ver || !led || !imu || !pose || ver() != PIBRIDGE_WB_EXT_VERSION) { fprintf(stderr, "no extension\n"); return 1; }
    printf("extension pibridge_wb v%d present\n", ver());
    vit_config_t cfg{nullptr, 4, 1, false};
    vit_tracker_t* t = nullptr;
    if (v.tracker_create(&cfg, &t) != VIT_SUCCESS || v.tracker_start(t) != VIT_SUCCESS) return 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));  // connect to winbridge
    FILE* f = fopen(path, "rb");
    char magic[4]; uint32_t fv;
    if (!f || fread(magic, 1, 4, f) != 4 || fread(&fv, 4, 1, f) != 1 || memcmp(magic, "CREP", 4)) return 2;
    FILE* csv[2];
    for (int k = 0; k < 2; ++k) {
        std::string p = std::string(prefix) + (k ? "-right.csv" : "-left.csv");
        csv[k] = fopen(p.c_str(), "w");
        fprintf(csv[k], "ts,qx,qy,qz,qw,px,py,pz\n");
    }
    std::vector<uint8_t> tiles(4 * 640 * 480);
    const uint32_t strides[4] = {640, 640, 640, 640};
    auto wall0 = std::chrono::steady_clock::now();
    uint64_t t0 = 0;
    long nLed = 0, nImu = 0, nPose[2] = {0, 0};
    int64_t last[2] = {0, 0};
    for (;;) {
        uint8_t typ;
        if (fread(&typ, 1, 1, f) != 1) break;
        if (typ == 1) {
            uint64_t ts; uint32_t sh;
            if (fread(&ts, 8, 1, f) != 1 || fread(&sh, 4, 1, f) != 1 || fread(tiles.data(), 1, tiles.size(), f) != tiles.size()) break;
            if (!t0) t0 = ts;
            std::this_thread::sleep_until(wall0 + std::chrono::nanoseconds(ts - t0));
            const uint8_t* tp[4] = {&tiles[0], &tiles[640 * 480], &tiles[2 * 640 * 480], &tiles[3 * 640 * 480]};
            led((int64_t)ts, sh, 4, tp, strides);
            ++nLed;
        } else {
            uint32_t hand; uint64_t ts; float a[3], g[3];
            if (fread(&hand, 4, 1, f) != 1 || fread(&ts, 8, 1, f) != 1 || fread(a, 4, 3, f) != 3 || fread(g, 4, 3, f) != 3) break;
            if (!t0) t0 = ts;
            imu(hand, (int64_t)ts, a, g);
            ++nImu;
        }
        for (uint32_t k = 0; k < 2; ++k) {  // like the driver: poll the latest pose
            pibridge_wb_ctrl_pose p;
            if (pose(k, &p) == 1 && p.ts_ns != last[k]) {
                last[k] = p.ts_ns;
                ++nPose[k];
                fprintf(csv[k], "%lld,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n", (long long)p.ts_ns, p.q[0], p.q[1], p.q[2],
                        p.q[3], p.p[0], p.p[1], p.p[2]);
            }
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    v.tracker_stop(t);
    v.tracker_destroy(t);
    fclose(f); fclose(csv[0]); fclose(csv[1]);
    printf("ctrl: LED frames %ld, IMU %ld; poses with optics: left %ld, right %ld\n", nLed, nImu, nPose[0], nPose[1]);
    return nPose[0] + nPose[1] > 0 ? 0 : 3;
}

int main(int argc, char** argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s lib.so replay.bin poses.csv [speed]\n", argv[0]); return 2; }
    double speed = argc > 4 ? atof(argv[4]) : 1.0;
    void* h = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
    if (!h) { fprintf(stderr, "dlopen: %s\n", dlerror()); return 1; }
    Vit v{};
    int missing = 0;
#define LOAD(n) v.n = (PFN_vit_##n)dlsym(h, "vit_" #n); if (!v.n) { fprintf(stderr, "missing vit_" #n "\n"); ++missing; }
    SYMS(LOAD)
#undef LOAD
    if (missing) return 1;
    uint32_t ma, mi, pa;
    v.api_get_version(&ma, &mi, &pa);
    printf("VIT %u.%u.%u, all 21 symbols present\n", ma, mi, pa);
    if (ma != VIT_HEADER_VERSION_MAJOR) return 1;
    if (strcmp(argv[2], "--ctrl") == 0) return run_ctrl(v, h, argv[3], argc > 4 ? argv[4] : "ctrlpose");

    FILE* f = fopen(argv[2], "rb");
    FILE* csv = fopen(argv[3], "w");
    if (!f || !csv) { perror("open"); return 2; }
    char magic[4]; uint32_t hdr[4];
    if (fread(magic, 1, 4, f) != 4 || fread(hdr, 4, 4, f) != 4 || memcmp(magic, "FREP", 4)) return 2;
    uint32_t W = hdr[1], H = hdr[2], ncam = hdr[3];

    vit_config_t cfg{nullptr, ncam, 1, false};
    vit_tracker_t* t = nullptr;
    if (v.tracker_create(&cfg, &t) != VIT_SUCCESS || v.tracker_start(t) != VIT_SUCCESS) return 1;

    fprintf(csv, "ts,px,py,pz,ox,oy,oz,ow\n");
    std::vector<uint8_t> px((size_t)W * H);
    auto wall0 = std::chrono::steady_clock::now();
    int64_t t0 = -1;
    long nImu = 0, nImg = 0, nPose = 0;
    float lastA[3] = {0, 0, 0};
    double gsum[3] = {0, 0, 0};
    long gn = 0;

    auto drain = [&] {
        vit_pose_t* p = nullptr;
        while (v.tracker_pop_pose(t, &p) == VIT_SUCCESS && p) {
            vit_pose_data_t d;
            v.pose_get_data(p, &d);
            fprintf(csv, "%lld,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f\n", (long long)d.timestamp, d.px, d.py, d.pz,
                    d.ox, d.oy, d.oz, d.ow);
            float q[4] = {d.ox, d.oy, d.oz, d.ow};
            double g[3];
            rotate(q, lastA, g);
            gsum[0] += g[0]; gsum[1] += g[1]; gsum[2] += g[2]; ++gn;
            v.pose_destroy(p);
            ++nPose;
        }
    };

    for (;;) {
        uint8_t typ;
        if (fread(&typ, 1, 1, f) != 1) break;
        if (typ == 0) {
            vit_imu_sample_t s{};
            float a[3], w[3];
            if (fread(&s.timestamp, 8, 1, f) != 1 || fread(a, 4, 3, f) != 3 || fread(w, 4, 3, f) != 3) break;
            if (t0 < 0) t0 = s.timestamp;
            s.ax = a[0]; s.ay = a[1]; s.az = a[2]; s.wx = w[0]; s.wy = w[1]; s.wz = w[2];
            memcpy(lastA, a, sizeof(lastA));
            v.tracker_push_imu_sample(t, &s);
            ++nImu;
        } else {
            int64_t ts;
            if (fread(&ts, 8, 1, f) != 1) break;
            if (t0 < 0) t0 = ts;
            std::this_thread::sleep_until(wall0 + std::chrono::nanoseconds((long long)((ts - t0) / speed)));
            for (uint32_t c = 0; c < ncam; ++c) {
                if (fread(px.data(), 1, px.size(), f) != px.size()) break;
                vit_img_sample_t s{};
                s.cam_index = c; s.timestamp = ts; s.data = px.data();
                s.width = W; s.height = H; s.stride = W; s.size = W * H; s.format = VIT_IMAGE_FORMAT_L8;
                v.tracker_push_img_sample(t, &s);
            }
            ++nImg;
            drain();
        }
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));  // let the worker thread flush the queue
    drain();
    v.tracker_stop(t);
    v.tracker_destroy(t);
    fclose(f); fclose(csv);
    printf("vit-replay: imu=%ld frames=%ld poses=%ld\n", nImu, nImg, nPose);
    if (gn) printf("acceleration in world (mean) = (%.2f, %.2f, %.2f) m/s^2  [Z-up expect ~(0,0,+9.8)]\n",
                   gsum[0] / gn, gsum[1] / gn, gsum[2] / gn);
    return nPose > 0 ? 0 : 3;
}
