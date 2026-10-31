// Copyright 2026, PiBridge contributors
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The original Pimax Crystal's own inside-out tracking, read from PiBridge's shared memory.
 *
 * The original Crystal (P3B) tracks its head and its controllers itself; pibridge-keepalived reads what it streams
 * over its HID and rewrites /dev/shm/pibridge-crystal-pose for every report (PiBridge libpibridge-hmd pose_shm.h, the
 * layout below). This makes Monado devices of it: the head (a head source for the Pimax HMD,
 * pimax_hmd_set_head_source) and the two controllers as Touch controllers (as the Windows driver presents them).
 * PIMAX_POSE_SHM picks another file.
 *
 * @ingroup drv_pimax
 */
#pragma once

#include "xrt/xrt_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * The head and the controllers of the shared memory file (out_head: a tracker posed by the headset's own inside-out,
 * input XRT_INPUT_GENERIC_HEAD_POSE; out_ctrls[0] left, [1] right). The head's manual recenter
 * (pimax_hmd_get_recenter) moves the controllers too; the Pimax button held recenters. The file is opened when it
 * is read (again after keepalived restarts); until it has fresh data the devices have no pose. False: out of memory.
 */
bool
pimax_poseshm_create(struct xrt_device **out_head, struct xrt_device *out_ctrls[2]);

#ifdef __cplusplus
}
#endif
