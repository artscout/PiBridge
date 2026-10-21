// pibridge winbridge client (native Linux): runs replay.bin (EuRoC -> prep_replay.py)
// through the winbridge server and writes poses to CSV. This is a protocol regression test; later, instead of
// replay.bin there will be a live stream from libpibridge cam/IMU.
// Build: g++ -O2 -std=c++20 client.cpp -o winbridge-client
// Run: winbridge-client replay.bin poses.csv [speed] [port]

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>
#include "proto.h"

static bool send_all(int s, const void* buf, size_t n) {
    const char* p = (const char*)buf;
    while (n > 0) { ssize_t r = ::send(s, p, n, 0); if (r <= 0) return false; p += r; n -= (size_t)r; }
    return true;
}
static bool recv_all(int s, void* buf, size_t n) {
    char* p = (char*)buf;
    while (n > 0) { ssize_t r = ::recv(s, p, n, 0); if (r <= 0) return false; p += r; n -= (size_t)r; }
    return true;
}

int main(int argc, char** argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s replay.bin poses.csv [speed] [port]\n", argv[0]); return 2; }
    double speed = argc > 3 ? atof(argv[3]) : 1.0;
    int port = argc > 4 ? atoi(argv[4]) : WINBRIDGE_PORT;
    const uint64_t shutterNs = 10000000;  // exposure 10 ms (EuRoC has none)

    FILE* f = fopen(argv[1], "rb");
    FILE* csv = fopen(argv[2], "w");
    if (!f || !csv) { perror("open"); return 2; }
    char magic[4]; uint32_t hdr[4];
    if (fread(magic, 1, 4, f) != 4 || fread(hdr, 4, 4, f) != 4 || memcmp(magic, "FREP", 4) != 0) {
        fprintf(stderr, "not a replay.bin file\n"); return 2;
    }
    uint32_t W = hdr[1], H = hdr[2], ncam = hdr[3];

    int s = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port);
    inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    if (connect(s, (sockaddr*)&a, sizeof(a)) != 0) { perror("connect"); return 1; }
    int one = 1; setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    fprintf(csv, "ts,q0,q1,q2,q3,px,py,pz\n");
    std::vector<uint8_t> camBuf(sizeof(CamHeader) + (size_t)ncam * W * H);
    auto wall0 = std::chrono::steady_clock::now();
    uint64_t t0 = 0; bool haveT0 = false;
    long imuN = 0, camN = 0, posN = 0;

    for (;;) {
        uint8_t typ;
        if (fread(&typ, 1, 1, f) != 1) break;
        if (typ == 0) {
            ImuPayload in;
            if (fread(&in.ts, 8, 1, f) != 1 || fread(in.accel, 4, 3, f) != 3 || fread(in.gyro, 4, 3, f) != 3) break;
            if (!haveT0) { t0 = in.ts; haveT0 = true; }
            MsgHeader h{MSG_IMU, sizeof(in)};
            if (!send_all(s, &h, sizeof(h)) || !send_all(s, &in, sizeof(in))) break;
            MsgHeader rh; PosePayload p;
            if (!recv_all(s, &rh, sizeof(rh)) || rh.type != MSG_POSE || !recv_all(s, &p, sizeof(p))) break;
            fprintf(csv, "%llu,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f\n", (unsigned long long)p.ts,
                    p.pose[0], p.pose[1], p.pose[2], p.pose[3], p.pose[4], p.pose[5], p.pose[6]);
            if (p.pose[4] != 0.0f || p.pose[5] != 0.0f || p.pose[6] != 0.0f) ++posN;
            ++imuN;
        } else {
            CamHeader ch{}; ch.shutter_ns = shutterNs; ch.w = W; ch.h = H; ch.ncam = ncam;
            if (fread(&ch.ts, 8, 1, f) != 1) break;
            uint8_t* px = camBuf.data() + sizeof(CamHeader);
            if (fread(px, 1, (size_t)ncam * W * H, f) != (size_t)ncam * W * H) break;
            memcpy(camBuf.data(), &ch, sizeof(ch));
            if (!haveT0) { t0 = ch.ts; haveT0 = true; }
            // real-time pacing: SLAM runs on the server's background thread
            auto due = wall0 + std::chrono::nanoseconds((long long)((ch.ts - t0) / speed));
            std::this_thread::sleep_until(due);
            MsgHeader h{MSG_CAM, (uint32_t)camBuf.size()};
            if (!send_all(s, &h, sizeof(h)) || !send_all(s, camBuf.data(), camBuf.size())) break;
            MsgHeader rh;
            if (!recv_all(s, &rh, sizeof(rh)) || rh.type != MSG_ACK) break;
            ++camN;
        }
    }
    close(s); fclose(f); fclose(csv);
    printf("client: imu=%ld cam=%ld, poses with position=%ld\n", imuN, camN, posN);
    return posN > 0 ? 0 : 3;
}
