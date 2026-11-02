// SPDX-License-Identifier: MIT
// The monitors' PipeWire streams of a desktop portal session (pw_capture.c): one input stream per node on the portal's
// PipeWire remote, raw video in shared memory (BGRx / BGRA / RGBx / RGBA; no DMA-BUF is offered, so KWin sends memfd
// buffers), handed to a callback on PipeWire's own thread. C, since PipeWire's format builders are C macros.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// The pixel layouts a frame comes in (pibridge::desktop::Source has the same order).
enum pibridge_pw_format { PIBRIDGE_PW_BGRA = 0, PIBRIDGE_PW_BGRX = 1, PIBRIDGE_PW_RGBA = 2, PIBRIDGE_PW_RGBX = 3 };

/// One frame of stream `index` (the order the nodes were given in), valid during the call only.
typedef void (*pibridge_pw_frame_cb)(void* user, unsigned index, const uint8_t* data, uint32_t width, uint32_t height,
                                     uint32_t stride, int format);
/// A stream stopped for good (the portal session ended, an error): its index and PipeWire's words.
typedef void (*pibridge_pw_error_cb)(void* user, unsigned index, const char* words);

typedef struct pibridge_pw pibridge_pw;

/// Connect to the portal's PipeWire remote `fd` (taken over: closed by pibridge_pw_stop) and start a stream for each of
/// the `n` nodes. NULL with the reason in `err` when it cannot.
pibridge_pw* pibridge_pw_start(int fd, const uint32_t* nodes, unsigned n, pibridge_pw_frame_cb frame,
                               pibridge_pw_error_cb error, void* user, char* err, size_t errlen);
/// Stop the streams and the thread; `pw` may be NULL.
void pibridge_pw_stop(pibridge_pw* pw);

#ifdef __cplusplus
}
#endif
