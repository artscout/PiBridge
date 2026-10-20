// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/record.rs (MIT OR Apache-2.0).
#include "pibridge/cam/record.h"

#include "pibridge/cam/clock.h"
#include "pibridge/cam/frame.h"
#include "pibridge/cam/rustfmt.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <format>
#include <unistd.h>
#include <zlib.h>

namespace pibridge::cam {

namespace {

constexpr std::size_t HDR_LEN = 32;
constexpr std::size_t REC_LEN = 32;
constexpr std::size_t IO_BUF = 4u << 20;

void put16(std::uint8_t* p, std::uint16_t v) {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
}

void put32(std::uint8_t* p, std::uint32_t v) {
    for (int i = 0; i < 4; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

void put64(std::uint8_t* p, std::uint64_t v) {
    for (int i = 0; i < 8; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

std::uint16_t rd16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t rd32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::uint64_t rd64(const std::uint8_t* p) {
    std::uint64_t v = 0;
    for (int i = 7; i >= 0; i--) {
        v = (v << 8) | p[i];
    }
    return v;
}

IoError invalid(std::string m) {
    return IoError::custom(ErrorKind::InvalidData, std::move(m));
}

// read_full: as many bytes as the file still has, up to len; an I/O error is an error.
Result<std::size_t> read_full(std::FILE* f, std::uint8_t* buf, std::size_t len) {
    const std::size_t n = std::fread(buf, 1, len, f);
    if (n < len && std::ferror(f)) {
        const int e = errno;
        std::clearerr(f);
        return fail(IoError::from_errno(e != 0 ? e : EIO));
    }
    return n;
}

Result<void> write_all(std::FILE* f, const void* p, std::size_t n) {
    if (n > 0 && std::fwrite(p, 1, n, f) != n) {
        return fail(IoError::from_errno(errno != 0 ? errno : EIO));
    }
    return {};
}

} // namespace

/*
 * Recorder
 */

Result<std::unique_ptr<Recorder>> Recorder::create(const std::string& path, std::size_t stride,
                                                   std::optional<std::uint8_t> compress_level) {
    std::FILE* f = std::fopen(path.c_str(), "wbe");
    if (f == nullptr) {
        return fail(IoError::last_os());
    }
    std::setvbuf(f, nullptr, _IOFBF, IO_BUF);
    std::unique_ptr<Recorder> r(new Recorder());
    r->f_ = f;
    r->compress_ = compress_level;
    r->stride_ = stride;
    std::uint8_t h[HDR_LEN] = {};
    std::memcpy(h, PXCAM_MAGIC, 8);
    put16(h + 8, 1);
    put16(h + 10, static_cast<std::uint16_t>(HDR_LEN));
    put16(h + 12, static_cast<std::uint16_t>(FRAME_W));
    put16(h + 14, static_cast<std::uint16_t>(FRAME_H));
    put32(h + 16, static_cast<std::uint32_t>(stride));
    put32(h + 20, compress_level ? 1u : 0u);
    if (auto w = write_all(f, h, HDR_LEN); !w) {
        return fail(w.error());
    }
    r->bytes = HDR_LEN;
    return r;
}

Recorder::~Recorder() {
    if (f_ != nullptr) {
        std::fclose(f_);
    }
}

Result<void> Recorder::write(const RawFrame& fr) {
    if (fr.stride != stride_) {
        return fail(IoError::other(std::format("stride {} != recording stride {}", fr.stride, stride_)));
    }
    const std::size_t raw_len = stride_ * (FRAME_H - 1) + FRAME_W;
    if (fr.data.size() < raw_len) {
        return fail(IoError::other("short frame"));
    }
    const std::uint8_t* payload = fr.data.data();
    std::size_t plen = raw_len;
    std::uint32_t flags = 0;
    if (compress_) {
        uLongf dlen = compressBound(static_cast<uLong>(raw_len));
        zbuf_.resize(dlen);
        const int zr = compress2(zbuf_.data(), &dlen, fr.data.data(), static_cast<uLong>(raw_len), *compress_);
        if (zr != Z_OK) {
            return fail(IoError::other(std::format("zlib compress failed ({})", zr)));
        }
        payload = zbuf_.data();
        plen = dlen;
        flags = PXCAM_REC_ZLIB;
    }
    std::uint8_t h[REC_LEN] = {};
    put32(h, PXCAM_REC_MAGIC);
    put32(h + 4, flags);
    put32(h + 8, static_cast<std::uint32_t>(plen));
    put32(h + 12, static_cast<std::uint32_t>(raw_len));
    put64(h + 16, fr.host_ts_ns);
    put32(h + 24, fr.sequence);
    if (auto w = write_all(f_, h, REC_LEN); !w) {
        return w;
    }
    if (auto w = write_all(f_, payload, plen); !w) {
        return w;
    }
    frames++;
    bytes += REC_LEN + plen;
    return {};
}

Result<std::pair<std::uint64_t, std::uint64_t>> Recorder::finish() {
    if (std::fflush(f_) != 0) {
        return fail(IoError::last_os());
    }
    if (::fdatasync(fileno(f_)) != 0) {
        return fail(IoError::last_os());
    }
    const int rc = std::fclose(f_);
    f_ = nullptr;
    if (rc != 0) {
        return fail(IoError::last_os());
    }
    return std::pair{frames, bytes};
}

/*
 * AsyncRecorder
 */

Result<std::unique_ptr<AsyncRecorder>> AsyncRecorder::start(const std::string& path, std::size_t stride,
                                                             std::optional<std::uint8_t> compress_level,
                                                             std::size_t depth) {
    auto rec = Recorder::create(path, stride, compress_level);
    if (!rec) {
        return fail(rec.error());
    }
    std::unique_ptr<AsyncRecorder> a(new AsyncRecorder());
    a->rec_ = std::move(*rec);
    a->stride_ = stride;
    a->depth_ = depth < 1 ? 1 : depth;
    a->running_ = true;
    a->thread_ = std::thread([p = a.get()] { p->run(); });
    pthread_setname_np(a->thread_.native_handle(), "pimax-cam-rec");
    return a;
}

void AsyncRecorder::run() {
    std::optional<IoError> err;
    for (;;) {
        Item it;
        {
            std::unique_lock lk(mu_);
            cv_.wait(lk, [&] { return closed_ || !queue_.empty(); });
            if (queue_.empty()) {
                break;
            }
            it = std::move(queue_.front());
            queue_.pop_front();
        }
        if (!err) {
            RawFrame f{it.data, stride_, it.ts, it.seq};
            if (auto w = rec_->write(f); !w) {
                err = w.error();
            }
        }
        std::lock_guard lk(mu_);
        if (pool_.size() < 8) {
            pool_.push_back(std::move(it.data));
        }
    }
    if (err) {
        result_err_ = err;
        return;
    }
    auto r = rec_->finish();
    if (!r) {
        result_err_ = r.error();
    } else {
        result_ = *r;
    }
}

void AsyncRecorder::push(const RawFrame& f) {
    if (closed_) {
        return;
    }
    const std::size_t len = stride_ * (FRAME_H - 1) + FRAME_W;
    if (f.data.size() < len || f.stride != stride_) {
        dropped++;
        return;
    }
    std::vector<std::uint8_t> buf;
    {
        std::lock_guard lk(mu_);
        if (!pool_.empty()) {
            buf = std::move(pool_.back());
            pool_.pop_back();
        }
    }
    buf.assign(f.data.begin(), f.data.begin() + static_cast<std::ptrdiff_t>(len));
    {
        std::lock_guard lk(mu_);
        if (queue_.size() >= depth_) {
            dropped++;
            return;
        }
        queue_.push_back(Item{std::move(buf), f.host_ts_ns, f.sequence});
    }
    queued++;
    cv_.notify_one();
}

Result<std::pair<std::uint64_t, std::uint64_t>> AsyncRecorder::finish() {
    if (!running_) {
        return std::pair<std::uint64_t, std::uint64_t>{0, 0};
    }
    {
        std::lock_guard lk(mu_);
        closed_ = true;
    }
    cv_.notify_one();
    thread_.join();
    running_ = false;
    if (result_err_) {
        return fail(*result_err_);
    }
    return result_;
}

AsyncRecorder::~AsyncRecorder() {
    if (running_) {
        (void)finish();
    }
}

/*
 * Reader
 */

Result<std::unique_ptr<Reader>> Reader::open(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "rbe");
    if (f == nullptr) {
        return fail(IoError::last_os());
    }
    std::setvbuf(f, nullptr, _IOFBF, IO_BUF);
    std::unique_ptr<Reader> r(new Reader());
    r->f_ = f;
    std::uint8_t h[HDR_LEN];
    auto n = read_full(f, h, HDR_LEN);
    if (!n || *n < HDR_LEN) {
        return fail(invalid("not a .pxcam file (short)"));
    }
    if (std::memcmp(h, PXCAM_MAGIC, 8) != 0) {
        return fail(invalid("not a .pxcam file (magic)"));
    }
    const std::uint16_t version = rd16(h + 8);
    const std::size_t hlen = rd16(h + 10);
    const std::size_t w = rd16(h + 12);
    const std::size_t hh = rd16(h + 14);
    const std::size_t stride = rd32(h + 16);
    if (version != 1 || hlen < HDR_LEN || w != FRAME_W || hh != FRAME_H || stride < FRAME_W) {
        return fail(invalid(std::format("unsupported .pxcam: version {} header {} {}x{} stride {}", version, hlen, w,
                                        hh, stride)));
    }
    if (hlen > HDR_LEN) {
        if (fseeko(f, static_cast<off_t>(hlen), SEEK_SET) != 0) {
            return fail(IoError::last_os());
        }
    }
    r->stride = stride;
    r->flags = rd32(h + 20);
    return r;
}

Reader::~Reader() {
    if (f_ != nullptr) {
        std::fclose(f_);
    }
}

Result<void> Reader::rewind() {
    if (fseeko(f_, static_cast<off_t>(HDR_LEN), SEEK_SET) != 0) {
        return fail(IoError::last_os());
    }
    std::clearerr(f_);
    truncated = false;
    return {};
}

Result<bool> Reader::next_record(Record& rec) {
    std::uint8_t h[REC_LEN];
    auto n = read_full(f_, h, REC_LEN);
    if (!n) {
        return fail(n.error());
    }
    if (*n == 0) {
        return false;
    }
    if (*n < REC_LEN) {
        truncated = true;
        return false;
    }
    if (rd32(h) != PXCAM_REC_MAGIC) {
        return fail(invalid("bad record magic"));
    }
    const std::uint32_t rflags = rd32(h + 4);
    const std::size_t stored = rd32(h + 8);
    const std::size_t raw_len = rd32(h + 12);
    if (raw_len != stride * (FRAME_H - 1) + FRAME_W || stored > raw_len + raw_len / 2 + 1024) {
        return fail(invalid(std::format("bad record lengths {}/{}", stored, raw_len)));
    }
    const bool z = (rflags & PXCAM_REC_ZLIB) != 0;
    std::vector<std::uint8_t>& dst = z ? payload_ : rec.data;
    dst.resize(stored);
    auto got = read_full(f_, dst.data(), stored);
    if (!got) {
        return fail(got.error());
    }
    if (*got < stored) {
        truncated = true;
        return false;
    }
    if (z) {
        rec.data.resize(raw_len);
        uLongf out = static_cast<uLongf>(raw_len);
        const int zr = uncompress(rec.data.data(), &out, payload_.data(), static_cast<uLong>(stored));
        if (zr != Z_OK) {
            return fail(invalid(std::format("zlib: {}", zr == Z_BUF_ERROR ? "HasMoreOutput" : zError(zr))));
        }
        if (out != raw_len) {
            return fail(invalid("zlib length mismatch"));
        }
    }
    rec.stride = stride;
    rec.host_ts_ns = rd64(h + 16);
    rec.sequence = rd32(h + 24);
    return true;
}

/*
 * Replay
 */

Result<std::unique_ptr<Replay>> Replay::open(const std::string& path, ReplayOptions opts) {
    auto r = Reader::open(path);
    if (!r) {
        return fail(r.error());
    }
    std::unique_ptr<Replay> p(new Replay());
    p->reader_ = std::move(*r);
    p->opts_ = opts;
    p->path_ = path;
    return p;
}

Result<bool> Replay::advance(int timeout_ms) {
    if (done_) {
        return false;
    }
    // read into a scratch record: at the end (or on an error) the current frame stays what it was
    auto got = reader_->next_record(next_);
    if (!got) {
        return fail(got.error());
    }
    if (!*got) {
        if (!opts_.looping) {
            done_ = true;
            return false;
        }
        if (auto w = reader_->rewind(); !w) {
            return fail(w.error());
        }
        loops++;
        // continue the timeline one frame period after the last frame
        loop_offset_ns_ = (last_out_ns_ > base_mono_ns_ ? last_out_ns_ - base_mono_ns_ : 0) + 20'000'000;
        base_rec_ns_.reset();
        auto again = reader_->next_record(next_);
        if (!again) {
            return fail(again.error());
        }
        if (!*again) {
            done_ = true;
            return false;
        }
    }
    std::swap(cur_, next_);
    const std::uint64_t now = monotonic_ns();
    std::uint64_t base_rec = 0;
    if (base_rec_ns_) {
        base_rec = *base_rec_ns_;
    } else {
        if (loops == 0) {
            base_mono_ns_ = now;
        }
        base_rec_ns_ = cur_.host_ts_ns;
        base_rec = cur_.host_ts_ns;
    }
    const std::uint64_t rel = cur_.host_ts_ns > base_rec ? cur_.host_ts_ns - base_rec : 0;
    std::uint64_t out = base_mono_ns_ + rel + loop_offset_ns_;
    if (opts_.realtime && out > now) {
        const std::uint64_t max_wait =
            std::max<std::uint64_t>(static_cast<std::uint64_t>(std::max(timeout_ms, 0)), 1000) * 1'000'000;
        std::this_thread::sleep_for(std::chrono::nanoseconds(std::min(out - now, max_wait)));
    }
    if (out <= last_out_ns_) {
        out = last_out_ns_ + 1;
    }
    last_out_ns_ = out;
    cur_.host_ts_ns = out;
    have_cur_ = true;
    return true;
}

std::optional<RawFrame> Replay::current() const {
    if (!have_cur_) {
        return std::nullopt;
    }
    return RawFrame{cur_.data, cur_.stride, cur_.host_ts_ns, cur_.sequence};
}

std::string Replay::describe() const {
    return std::format("replay {} (loop {}, realtime {}, zlib {})", rfmt::utf8_lossy(path_), opts_.looping,
                       opts_.realtime, (reader_->flags & 1) != 0);
}

} // namespace pibridge::cam
