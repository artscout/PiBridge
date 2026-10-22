// SPDX-License-Identifier: MIT
// Read a pose file through Monado's own reader (pimax_ctrltrack_shm.h of the Pimax driver, included from
// third_party/monado, not copied) and print what Monado sees: per hand whether it was ever seen and every field as
// exact hex floats. The times (frame_ns, publish_ns) are clock values of the run, so only "seen" is printed. Then the
// header and the heartbeat fields that do not depend on timing.
#include "pimax_ctrltrack_shm.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        return 2;
    }
    struct pimax_ctrltrack_reader r = {0};
    if (!pimax_ctrltrack_open(&r, argv[1])) {
        printf("monado reader: cannot open %s\n", argv[1]);
        return 0;
    }
    struct pimax_ctrltrack_hand h[2];
    if (!pimax_ctrltrack_read(&r, h)) {
        printf("monado reader: no snapshot\n");
        return 0;
    }
    for (int i = 0; i < 2; i++) {
        printf("hand %d seen %d pos %a %a %a quat %a %a %a %a rms %a leds %u cams %u aim_from_imu %a %a %a %a\n", i,
               h[i].frame_ns != 0, h[i].pos[0], h[i].pos[1], h[i].pos[2], h[i].quat[0], h[i].quat[1], h[i].quat[2],
               h[i].quat[3], h[i].rms_px, h[i].leds, h[i].cams, h[i].aim_from_imu[0], h[i].aim_from_imu[1],
               h[i].aim_from_imu[2], h[i].aim_from_imu[3]);
    }
    uint32_t version, seq, up, reopens;
    uint64_t beat;
    memcpy(&version, r.map + 4, 4);
    memcpy(&seq, r.map + 8, 4);
    memcpy(&beat, r.map + 208, 8);
    memcpy(&up, r.map + 224, 4);
    memcpy(&reopens, r.map + 228, 4);
    printf("version %u seq even %d beat %d source_up %u reopens %u\n", version, (seq & 1u) == 0, beat != 0, up,
           reopens);
    pimax_ctrltrack_close(&r);
    return 0;
}
