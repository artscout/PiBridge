// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/camera.rs (MIT OR Apache-2.0).
#include "pibridge/cam/camera.h"

namespace pibridge::cam {

Camera::Camera(std::unique_ptr<Source> src) : src_(std::move(src)) {
    generation_ = src_->generation();
}

Result<std::unique_ptr<Camera>> Camera::open(std::string_view spec, std::uint32_t fps, ReplayOptions replay) {
    auto s = open_source(spec, fps, replay);
    if (!s) {
        return fail(s.error());
    }
    return std::make_unique<Camera>(std::move(*s));
}

Result<void> Camera::start_recording(const std::string& path, std::optional<std::uint8_t> zlib) {
    auto r = AsyncRecorder::start(path, FRAME_W, zlib, 64);
    if (!r) {
        return fail(r.error());
    }
    // a recording already running is finished first (Rust drops it, which joins its writer)
    if (recorder_) {
        (void)recorder_->finish();
    }
    recorder_ = std::move(*r);
    return {};
}

Result<std::optional<std::pair<std::uint64_t, std::uint64_t>>> Camera::stop_recording() {
    if (!recorder_) {
        return std::optional<std::pair<std::uint64_t, std::uint64_t>>{};
    }
    std::unique_ptr<AsyncRecorder> r = std::move(recorder_);
    stats.record_dropped += r->dropped;
    auto res = r->finish();
    if (!res) {
        return fail(res.error());
    }
    return std::optional{*res};
}

Result<std::optional<FrameInfo>> Camera::next_frame(int timeout_ms) {
    auto a = src_->advance(timeout_ms);
    if (!a) {
        return fail(a.error());
    }
    if (!*a) {
        return std::optional<FrameInfo>{};
    }
    const std::uint64_t generation = src_->generation();
    if (generation != generation_) {
        // reopened camera or new broker: the device clock and the frame counter may have restarted
        generation_ = generation;
        clock_ = HostClockMap();
        last_counter_.reset();
        last_kind_ = FrameKind::Unknown;
        stats.reopens++;
    }
    const bool replay = src_->is_replay();
    const auto raw = src_->current();
    if (!raw) {
        return fail(IoError::other("source has no current frame"));
    }
    std::string err;
    if (!check_raw(raw->data.size(), raw->stride, &err)) {
        return fail(IoError::other(err));
    }
    const std::size_t stride = raw->stride;
    if (recorder_) {
        recorder_->push(*raw);
    }
    std::optional<Meta> meta = Meta::parse(raw->data.first(META_LEN));
    if (meta && !meta->looks_valid()) {
        meta.reset();
    }
    const TileRows rows = tile_rows(meta ? &*meta : nullptr, raw->data, stride);
    std::uint32_t flags = replay ? FLAG_REPLAY : 0;
    Stats& st = stats;
    st.frames++;
    if (rows.source == RowSource::Scan) {
        flags |= FLAG_ROWS_SCANNED;
        st.scanned_rows++;
    } else if (verify_rows) {
        for (std::size_t t = 0; t < TILE_COUNT; t++) {
            if (!tile_start_consistent(raw->data, stride, t, rows.y[t])) {
                st.row_mismatch++;
                break;
            }
        }
    }
    const Meta m = meta.value_or(Meta{});
    FrameKind kind = FrameKind::Unknown;
    if (meta) {
        flags |= FLAG_META_VALID;
        kind = m.kind();
    }
    switch (kind) {
    case FrameKind::Tracking: st.tracking++; break;
    case FrameKind::Led: st.led++; break;
    default: st.unknown++; break;
    }
    if (kind != FrameKind::Unknown && kind == last_kind_) {
        st.kind_repeats++;
    }
    last_kind_ = kind;
    std::uint32_t dropped = 0;
    if (meta) {
        if (last_counter_) {
            const std::uint32_t gap = m.counter - *last_counter_;
            // a jump backwards or by more than 10 s worth of frames is a restart, not a loss
            if (gap >= 1 && gap <= 500) {
                dropped = gap - 1;
            }
        }
        last_counter_ = m.counter;
    }
    st.dropped += dropped;
    const std::uint64_t mono = meta ? clock_.map(m.ts_us, raw->host_ts_ns) : raw->host_ts_ns;
    rows_ = rows;
    FrameInfo i;
    i.host_ts_ns = raw->host_ts_ns;
    i.dev_ts_us = m.ts_us;
    i.mono_ts_ns = mono;
    i.counter = m.counter;
    i.sequence = raw->sequence;
    i.dropped = dropped;
    i.kind = static_cast<std::uint32_t>(kind);
    i.frame_type = m.frame_type;
    i.stream_index = m.stream_index;
    i.exposure = m.exposure;
    i.gain = m.gain;
    for (std::size_t t = 0; t < 4; t++) {
        i.y_offset[t] = static_cast<std::uint16_t>(rows.y[t]);
    }
    i.tile_width = static_cast<std::uint32_t>(TILE_W);
    i.tile_height = static_cast<std::uint32_t>(TILE_H);
    i.flags = flags;
    i.reserved = 0;
    info_ = i;
    return std::optional{info_};
}

Result<void> Camera::copy_tile(std::size_t t, std::span<std::uint8_t> dst, std::size_t dst_stride) const {
    if (t >= TILE_COUNT) {
        return fail(IoError::other("tile index out of range"));
    }
    if (dst_stride < TILE_W || dst.size() < dst_stride * (TILE_H - 1) + TILE_W) {
        return fail(IoError::other("destination too small"));
    }
    const auto raw = src_->current();
    if (!raw || !rows_) {
        return fail(IoError::other("no current frame"));
    }
    cam::copy_tile(raw->data, raw->stride, t, rows_->y[t], dst, dst_stride);
    return {};
}

Result<TileView> Camera::tile_view(std::size_t t) const {
    if (t >= TILE_COUNT) {
        return fail(IoError::other("tile index out of range"));
    }
    const auto raw = src_->current();
    if (!raw || !rows_) {
        return fail(IoError::other("no current frame"));
    }
    const std::size_t need = (rows_->y[t] + TILE_H - 1) * raw->stride + t * TILE_W + TILE_W;
    if (raw->data.size() < need) {
        return fail(IoError::other("tile outside the frame"));
    }
    return TileView{raw->data, raw->stride, t * TILE_W, rows_->y[t], TILE_W, TILE_H};
}

Result<std::vector<std::uint8_t>> Camera::tile(std::size_t t) const {
    std::vector<std::uint8_t> v(TILE_W * TILE_H);
    if (auto r = copy_tile(t, v, TILE_W); !r) {
        return fail(r.error());
    }
    return v;
}

std::optional<std::span<const std::uint8_t>> Camera::raw() const {
    const auto r = src_->current();
    if (!r) {
        return std::nullopt;
    }
    return r->data;
}

} // namespace pibridge::cam
