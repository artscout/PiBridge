/* SPDX-License-Identifier: MIT OR Apache-2.0
 * The gaze that `omarchyvr eyed` publishes in /dev/shm/omarchyvr-eye-gaze (rust/pimax-eye/src/shm.rs), for readers in
 * C or C++: the SteamVR driver (CustomHeadsetOpenVR, linux/crystal-super/chovr-linux-eye-gaze.patch) and others.
 *
 * Head frame of OpenXR: x right, y up, -z forward; metres; times are CLOCK_MONOTONIC nanoseconds.
 *
 * Reading without locks: load seq_end, copy the struct, load seq_begin (with acquire fences between, or read the
 * three parts with separate read()/pread() calls); keep the copy only when seq_begin == seq_end, it is even and not 0,
 * and magic/version match. Otherwise retry. A gaze older than ~100 ms (t_ns) is stale: the tracker stopped.
 */
#ifndef OMARCHYVR_EYE_GAZE_H
#define OMARCHYVR_EYE_GAZE_H

#include <stdint.h>

#define OMARCHYVR_EYE_GAZE_PATH "/dev/shm/omarchyvr-eye-gaze"
#define OMARCHYVR_EYE_GAZE_MAGIC 0x4745564fu /* "OVEG" little endian */
#define OMARCHYVR_EYE_GAZE_VERSION 1u

#define OMARCHYVR_EYE_GAZE_LEFT 1u          /* left direction valid */
#define OMARCHYVR_EYE_GAZE_RIGHT 2u         /* right direction valid */
#define OMARCHYVR_EYE_GAZE_COMBINED 4u      /* combined direction valid */
#define OMARCHYVR_EYE_GAZE_LEFT_ORIGIN 8u   /* left origin valid */
#define OMARCHYVR_EYE_GAZE_RIGHT_ORIGIN 16u /* right origin valid */

#define OMARCHYVR_EYE_STATE_STREAMING 2u

#pragma pack(push, 1)
struct omarchyvr_eye_gaze {
    uint64_t seq_begin;
    uint32_t magic;
    uint32_t version;
    int64_t t_ns;       /* sample time (CLOCK_MONOTONIC, latency corrected) */
    int64_t written_ns; /* when it was published */
    uint32_t flags;
    float ipd_mm;       /* from the eye chip, 0 = unknown */
    float left_origin[3], left_dir[3];
    float right_origin[3], right_dir[3];
    float combined_origin[3], combined_dir[3];
    uint64_t sample;    /* tracker sample number */
    uint32_t state;     /* 0 searching, 1 connecting, 2 streaming */
    uint32_t reserved;
    uint64_t seq_end;
};
#pragma pack(pop)

#ifdef __cplusplus
static_assert(sizeof(struct omarchyvr_eye_gaze) == 136, "layout of rust/pimax-eye/src/shm.rs");
#endif

#endif
