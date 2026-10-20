// ctrl_prep: a controller take (.pxcam with the LED frames + the pimax-ctrlfuse logs of both hands) -> one
// time-sorted replay file for loader.exe ctrlplay (Pimax's LED engine under Wine), plus the take's optical fixes as a
// reference CSV per hand.
//
// Replay "CREP" v1: header magic + u32 version; then records, u8 type:
//   1 = LED frame set: u64 host_ns; u32 shutter_ns; 4 x 640x480 Y8 (cameras 0..3)
//   2 = controller IMU: u32 hand (0 left, 1 right); u64 host_ns; f32 accel[3] m/s^2; f32 gyro[3] rad/s (raw axes)
// IMU scales as the Monado driver uses them (pimax_ctrl.c): accel 1024 LSB/g, gyro 16.4 LSB/(deg/s).
//
// Build: g++ -O2 -std=c++23 ctrl_prep.cpp -I../libpibridge-cam/include -I../common \
//        ../build-app2/libpibridge-cam/libpibridge_cam.a -lz -o ctrl-prep
// Run:   ctrl-prep TAKE.pxcam LEFT.csv RIGHT.csv OUT.bin REF_PREFIX [max_seconds]

#include <pibridge/cam/frame.h>
#include <pibridge/cam/record.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace cam = pibridge::cam;

struct Imu {
    uint32_t hand;
    uint64_t ts;
    float a[3], g[3];
};

static std::vector<Imu> read_imu(const char* path, uint32_t hand, const std::string& ref_path) {
    std::vector<Imu> v;
    std::ifstream f(path);
    std::ofstream ref(ref_path);
    ref << "frame_ns,px,py,pz,qx,qy,qz,qw,rms_px,leds,verdict\n";
    std::string line;
    const double g0 = 9.80665, dps = M_PI / 180.0 / 16.4;
    while (std::getline(f, line)) {
        if (line.size() < 3 || line[0] == '#') continue;
        std::vector<std::string> c;
        std::stringstream ss(line);
        for (std::string x; std::getline(ss, x, ',');) c.push_back(x);
        if (c[0] == "I" && c.size() >= 9) {
            Imu s{};
            s.hand = hand;
            s.ts = std::stoull(c[1]);
            for (int i = 0; i < 3; ++i) s.g[i] = (float)(std::stod(c[3 + i]) * dps);
            for (int i = 0; i < 3; ++i) s.a[i] = (float)(std::stod(c[6 + i]) / 1024.0 * g0);
            v.push_back(s);
        } else if (c[0] == "O" && c.size() >= 19) {
            // optical fix of the take's tracker: frame_ns, world_from_aim pos+quat, ..., rms_px, leds, verdict
            ref << c[1];
            for (int i = 2; i <= 8; ++i) ref << ',' << c[i];
            ref << ',' << c[16] << ',' << c[17] << ',' << c[18] << '\n';
        }
    }
    return v;
}

int main(int argc, char** argv) {
    if (argc < 6) {
        std::fprintf(stderr, "usage: %s TAKE.pxcam LEFT.csv RIGHT.csv OUT.bin REF_PREFIX [max_seconds]\n", argv[0]);
        return 2;
    }
    const double max_s = argc > 6 ? std::atof(argv[6]) : 1e9;
    std::vector<Imu> imu = read_imu(argv[2], 0, std::string(argv[5]) + "-left.csv");
    std::vector<Imu> right = read_imu(argv[3], 1, std::string(argv[5]) + "-right.csv");
    imu.insert(imu.end(), right.begin(), right.end());
    std::sort(imu.begin(), imu.end(), [](const Imu& x, const Imu& y) { return x.ts < y.ts; });
    if (imu.empty()) {
        std::fprintf(stderr, "no IMU samples\n");
        return 1;
    }
    const uint64_t imu0 = imu.front().ts, imu1 = imu.back().ts;

    auto rd = cam::Reader::open(argv[1]);
    if (!rd) {
        std::fprintf(stderr, "open %s failed\n", argv[1]);
        return 1;
    }
    std::FILE* out = std::fopen(argv[4], "wb");
    const uint32_t ver = 1;
    std::fwrite("CREP", 1, 4, out);
    std::fwrite(&ver, 4, 1, out);

    std::vector<uint8_t> tiles(4 * cam::TILE_W * cam::TILE_H);
    cam::Record rec;
    size_t k = 0;  // next IMU sample to write
    long led = 0, skipped = 0, imu_n = 0;
    uint64_t first = 0;
    auto write_imu_until = [&](uint64_t t) {
        for (; k < imu.size() && imu[k].ts <= t; ++k) {
            const uint8_t type = 2;
            std::fwrite(&type, 1, 1, out);
            std::fwrite(&imu[k].hand, 4, 1, out);
            std::fwrite(&imu[k].ts, 8, 1, out);
            std::fwrite(imu[k].a, 4, 3, out);
            std::fwrite(imu[k].g, 4, 3, out);
            ++imu_n;
        }
    };
    while (true) {
        auto r = (*rd)->next_record(rec);
        if (!r || !*r) break;
        if (rec.host_ts_ns < imu0 || rec.host_ts_ns > imu1) { ++skipped; continue; }
        std::span<const uint8_t> data(rec.data.data(), rec.data.size());
        auto meta = cam::Meta::parse(data.subspan(0, std::min<size_t>(rec.stride, data.size())));
        if (!meta || meta->kind() != cam::FrameKind::Led) continue;
        if (first == 0) {
            first = rec.host_ts_ns;
            // IMU from 0.5 s before the first LED frame on (earlier samples would arrive as one burst)
            while (k < imu.size() && imu[k].ts + 500000000ULL < first) ++k;
        }
        if ((rec.host_ts_ns - first) / 1e9 > max_s) break;
        const cam::TileRows rows = cam::tile_rows(&*meta, data, rec.stride);
        for (size_t t = 0; t < 4; ++t)
            cam::copy_tile(data, rec.stride, t, rows.y[t],
                           std::span<uint8_t>(tiles.data() + t * cam::TILE_W * cam::TILE_H, cam::TILE_W * cam::TILE_H),
                           cam::TILE_W);
        write_imu_until(rec.host_ts_ns);
        const uint8_t type = 1;
        const uint32_t shutter = 40000;  // as Pimax Play hands the LED frames (40 us)
        std::fwrite(&type, 1, 1, out);
        std::fwrite(&rec.host_ts_ns, 8, 1, out);
        std::fwrite(&shutter, 4, 1, out);
        std::fwrite(tiles.data(), 1, tiles.size(), out);
        ++led;
    }
    std::fclose(out);
    std::printf("ctrl-prep: %ld LED frame sets, %ld IMU samples (both hands), %ld frames outside the IMU span\n", led,
                imu_n, skipped);
    return led > 0 ? 0 : 1;
}
