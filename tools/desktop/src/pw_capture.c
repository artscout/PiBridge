// SPDX-License-Identifier: MIT
#include "pw_capture.h"

#include <pipewire/pipewire.h>
#include <spa/param/video/format-utils.h>
#include <spa/param/buffers.h>
#include <spa/param/param.h>
#include <spa/pod/builder.h>
#include <spa/utils/result.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define PIBRIDGE_PW_MAX 8

struct stream {
    struct pibridge_pw* owner;
    unsigned index;
    struct pw_stream* s;
    struct spa_hook listener;
    uint32_t width, height;
    int format;
    int failed;
};

struct pibridge_pw {
    struct pw_thread_loop* loop;
    struct pw_context* context;
    struct pw_core* core;
    struct spa_hook core_listener;
    struct stream streams[PIBRIDGE_PW_MAX];
    unsigned n;
    pibridge_pw_frame_cb frame;
    pibridge_pw_error_cb error;
    void* user;
};

static int pw_inited = 0;

static int format_of(uint32_t f) {
    switch (f) {
    case SPA_VIDEO_FORMAT_BGRA: return PIBRIDGE_PW_BGRA;
    case SPA_VIDEO_FORMAT_BGRx: return PIBRIDGE_PW_BGRX;
    case SPA_VIDEO_FORMAT_RGBA: return PIBRIDGE_PW_RGBA;
    case SPA_VIDEO_FORMAT_RGBx: return PIBRIDGE_PW_RGBX;
    default: return -1;
    }
}

static void on_param_changed(void* data, uint32_t id, const struct spa_pod* param) {
    struct stream* st = data;
    if (param == NULL || id != SPA_PARAM_Format) {
        return;
    }
    struct spa_video_info_raw info;
    memset(&info, 0, sizeof info);
    uint32_t media_type = 0, media_subtype = 0;
    if (spa_format_parse(param, &media_type, &media_subtype) < 0 || media_type != SPA_MEDIA_TYPE_video ||
        media_subtype != SPA_MEDIA_SUBTYPE_raw || spa_format_video_raw_parse(param, &info) < 0) {
        return;
    }
    st->width = info.size.width;
    st->height = info.size.height;
    st->format = format_of(info.format);
    // shared memory only (mapped for us): no DMA-BUF
    uint8_t buffer[256];
    struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof buffer);
    const struct spa_pod* params[1];
    params[0] = spa_pod_builder_add_object(&b, SPA_TYPE_OBJECT_ParamBuffers, SPA_PARAM_Buffers,
                                           SPA_PARAM_BUFFERS_dataType,
                                           SPA_POD_CHOICE_FLAGS_Int((1 << SPA_DATA_MemPtr) | (1 << SPA_DATA_MemFd)));
    pw_stream_update_params(st->s, params, 1);
}

static void on_state_changed(void* data, enum pw_stream_state old, enum pw_stream_state state, const char* error) {
    struct stream* st = data;
    (void)old;
    if ((state == PW_STREAM_STATE_ERROR || state == PW_STREAM_STATE_UNCONNECTED) && !st->failed) {
        st->failed = 1;
        if (st->owner->error != NULL) {
            st->owner->error(st->owner->user, st->index,
                             error != NULL ? error : (state == PW_STREAM_STATE_ERROR ? "stream error" : "stream ended"));
        }
    }
}

static void on_process(void* data) {
    struct stream* st = data;
    struct pw_buffer* b = pw_stream_dequeue_buffer(st->s);
    if (b == NULL) {
        return;
    }
    struct spa_buffer* buf = b->buffer;
    struct spa_meta_header* mh = spa_buffer_find_meta_data(buf, SPA_META_Header, sizeof *mh);
    const int corrupted = mh != NULL && (mh->flags & SPA_META_HEADER_FLAG_CORRUPTED) != 0;
    if (!corrupted && buf->n_datas > 0 && buf->datas[0].data != NULL && buf->datas[0].chunk != NULL && st->format >= 0 &&
        st->width > 0 && st->height > 0) {
        const struct spa_data* d = &buf->datas[0];
        const uint32_t stride = d->chunk->stride > 0 ? (uint32_t)d->chunk->stride : st->width * 4;
        const uint64_t need = (uint64_t)stride * (st->height - 1) + (uint64_t)st->width * 4;
        if (d->chunk->size > 0 && need <= d->maxsize - d->chunk->offset) {
            st->owner->frame(st->owner->user, st->index, (const uint8_t*)d->data + d->chunk->offset, st->width,
                             st->height, stride, st->format);
        }
    }
    pw_stream_queue_buffer(st->s, b);
}

static const struct pw_stream_events stream_events = {
    PW_VERSION_STREAM_EVENTS,
    .state_changed = on_state_changed,
    .param_changed = on_param_changed,
    .process = on_process,
};

static void on_core_error(void* data, uint32_t id, int seq, int res, const char* message) {
    struct pibridge_pw* pw = data;
    (void)seq;
    if (id == PW_ID_CORE && res == -EPIPE && pw->error != NULL) {
        for (unsigned i = 0; i < pw->n; i++) {
            if (!pw->streams[i].failed) {
                pw->streams[i].failed = 1;
                pw->error(pw->user, i, message != NULL ? message : "PipeWire went away");
            }
        }
    }
}

static const struct pw_core_events core_events = {
    PW_VERSION_CORE_EVENTS,
    .error = on_core_error,
};

pibridge_pw* pibridge_pw_start(int fd, const uint32_t* nodes, unsigned n, pibridge_pw_frame_cb frame,
                               pibridge_pw_error_cb error, void* user, char* err, size_t errlen) {
    if (n == 0 || n > PIBRIDGE_PW_MAX || frame == NULL) {
        snprintf(err, errlen, "no streams");
        if (fd >= 0) {
            close(fd);
        }
        return NULL;
    }
    if (!pw_inited) {
        pw_init(NULL, NULL);
        pw_inited = 1;
    }
    struct pibridge_pw* pw = calloc(1, sizeof *pw);
    pw->frame = frame;
    pw->error = error;
    pw->user = user;
    pw->loop = pw_thread_loop_new("pibridge-desktop", NULL);
    if (pw->loop == NULL) {
        snprintf(err, errlen, "cannot make PipeWire's loop");
        close(fd);
        free(pw);
        return NULL;
    }
    pw->context = pw_context_new(pw_thread_loop_get_loop(pw->loop), NULL, 0);
    if (pw->context == NULL || pw_thread_loop_start(pw->loop) < 0) {
        snprintf(err, errlen, "cannot start PipeWire's loop");
        close(fd);
        pibridge_pw_stop(pw);
        return NULL;
    }
    pw_thread_loop_lock(pw->loop);
    pw->core = pw_context_connect_fd(pw->context, fd, NULL, 0);
    if (pw->core == NULL) {
        pw_thread_loop_unlock(pw->loop);
        snprintf(err, errlen, "cannot connect to the portal's PipeWire remote");
        pibridge_pw_stop(pw);
        return NULL;
    }
    pw_core_add_listener(pw->core, &pw->core_listener, &core_events, pw);
    for (unsigned i = 0; i < n; i++) {
        struct stream* st = &pw->streams[i];
        st->owner = pw;
        st->index = i;
        st->format = -1;
        st->s = pw_stream_new(pw->core, "pibridge-desktop",
                              pw_properties_new(PW_KEY_MEDIA_TYPE, "Video", PW_KEY_MEDIA_CATEGORY, "Capture",
                                                PW_KEY_MEDIA_ROLE, "Screen", NULL));
        if (st->s == NULL) {
            continue;
        }
        pw_stream_add_listener(st->s, &st->listener, &stream_events, st);
        uint8_t buffer[1024];
        struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof buffer);
        const struct spa_pod* params[1];
        struct spa_rectangle def_size = SPA_RECTANGLE(1920, 1080), min_size = SPA_RECTANGLE(1, 1),
                             max_size = SPA_RECTANGLE(8192, 8192);
        struct spa_fraction def_rate = SPA_FRACTION(60, 1), min_rate = SPA_FRACTION(0, 1), max_rate = SPA_FRACTION(240, 1);
        params[0] = spa_pod_builder_add_object(
            &b, SPA_TYPE_OBJECT_Format, SPA_PARAM_EnumFormat, SPA_FORMAT_mediaType, SPA_POD_Id(SPA_MEDIA_TYPE_video),
            SPA_FORMAT_mediaSubtype, SPA_POD_Id(SPA_MEDIA_SUBTYPE_raw), SPA_FORMAT_VIDEO_format,
            SPA_POD_CHOICE_ENUM_Id(5, SPA_VIDEO_FORMAT_BGRx, SPA_VIDEO_FORMAT_BGRx, SPA_VIDEO_FORMAT_BGRA,
                                   SPA_VIDEO_FORMAT_RGBx, SPA_VIDEO_FORMAT_RGBA),
            SPA_FORMAT_VIDEO_size, SPA_POD_CHOICE_RANGE_Rectangle(&def_size, &min_size, &max_size),
            SPA_FORMAT_VIDEO_framerate, SPA_POD_CHOICE_RANGE_Fraction(&def_rate, &min_rate, &max_rate));
        pw_stream_connect(st->s, PW_DIRECTION_INPUT, nodes[i],
                          (enum pw_stream_flags)(PW_STREAM_FLAG_AUTOCONNECT | PW_STREAM_FLAG_MAP_BUFFERS), params, 1);
    }
    pw->n = n;
    pw_thread_loop_unlock(pw->loop);
    return pw;
}

void pibridge_pw_stop(pibridge_pw* pw) {
    if (pw == NULL) {
        return;
    }
    if (pw->loop != NULL) {
        pw_thread_loop_lock(pw->loop);
        for (unsigned i = 0; i < pw->n; i++) {
            if (pw->streams[i].s != NULL) {
                pw_stream_disconnect(pw->streams[i].s);
                pw_stream_destroy(pw->streams[i].s);
                pw->streams[i].s = NULL;
            }
        }
        if (pw->core != NULL) {
            pw_core_disconnect(pw->core);
            pw->core = NULL;
        }
        pw_thread_loop_unlock(pw->loop);
        pw_thread_loop_stop(pw->loop);
    }
    if (pw->context != NULL) {
        pw_context_destroy(pw->context);
    }
    if (pw->loop != NULL) {
        pw_thread_loop_destroy(pw->loop);
    }
    free(pw);
}
