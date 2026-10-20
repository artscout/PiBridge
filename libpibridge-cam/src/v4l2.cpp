// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/v4l2.rs (MIT OR Apache-2.0).
#include "pibridge/cam/v4l2.h"

#include "pibridge/cam/clock.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <format>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace pibridge::cam {

namespace {

// The struct layouts the Rust code declares by hand (checked there against this header).
static_assert(sizeof(v4l2_capability) == 104);
static_assert(sizeof(v4l2_format) == 208);
static_assert(sizeof(v4l2_requestbuffers) == 20);
static_assert(sizeof(v4l2_buffer) == 88);
static_assert(sizeof(v4l2_streamparm) == 204);
static_assert(sizeof(v4l2_frmivalenum) == 52);

template <class T>
Result<void> xioctl(int fd, unsigned long req, T* arg) {
    for (;;) {
        if (::ioctl(fd, req, arg) >= 0) {
            return {};
        }
        if (errno != EINTR) {
            return fail(IoError::last_os());
        }
    }
}

std::string cstr(const std::uint8_t* b, std::size_t n) {
    std::size_t end = 0;
    while (end < n && b[end] != 0) {
        end++;
    }
    return rfmt::utf8_lossy(std::string_view(reinterpret_cast<const char*>(b), end));
}

std::optional<std::uint32_t> probe_caps(const std::string& path) {
    const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return std::nullopt;
    }
    v4l2_capability cap{};
    const bool ok = static_cast<bool>(xioctl(fd, VIDIOC_QUERYCAP, &cap));
    ::close(fd);
    if (!ok) {
        return std::nullopt;
    }
    return (cap.capabilities & V4L2_CAP_DEVICE_CAPS) != 0 ? cap.device_caps : cap.capabilities;
}

std::string read_trim_lower(const std::string& p) {
    auto s = read_file(p);
    if (!s || !rfmt::utf8_valid(*s)) {
        return "";
    }
    std::string t(rfmt::trim(*s));
    for (char& c : t) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return t;
}

v4l2_buffer new_buffer(std::uint32_t index) {
    v4l2_buffer b{};
    b.index = index;
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    return b;
}

} // namespace

std::optional<std::string> pick_node(const std::vector<NodeEntry>& entries) {
    for (const auto& e : entries) {
        if (e.vid != CAM_VID || e.pid != CAM_PID) {
            continue;
        }
        bool ok = false;
        if (e.caps) {
            ok = (*e.caps & V4L2_CAP_VIDEO_CAPTURE) != 0 && (*e.caps & V4L2_CAP_STREAMING) != 0;
        } else {
            ok = e.index.value_or(0) == 0;
        }
        if (ok) {
            return e.path;
        }
    }
    return std::nullopt;
}

std::optional<std::string> find_device() {
    DIR* d = ::opendir("/sys/class/video4linux");
    if (d == nullptr) {
        return std::nullopt;
    }
    std::vector<std::pair<std::uint32_t, std::string>> nodes;
    while (const dirent* e = ::readdir(d)) {
        const std::string_view n(e->d_name);
        if (!n.starts_with("video")) {
            continue;
        }
        std::uint32_t num = 0;
        if (!rfmt::parse_u32(n.substr(5), num)) {
            continue;
        }
        nodes.emplace_back(num, std::string("/sys/class/video4linux/") + e->d_name);
    }
    ::closedir(d);
    std::sort(nodes.begin(), nodes.end());
    std::vector<NodeEntry> entries;
    for (const auto& [num, n] : nodes) {
        const std::string usb = n + "/device/..";
        const std::string vid = read_trim_lower(usb + "/idVendor");
        const std::string pid = read_trim_lower(usb + "/idProduct");
        if (vid != CAM_VID || pid != CAM_PID) {
            continue;
        }
        NodeEntry ne;
        ne.path = std::format("/dev/video{}", num);
        ne.vid = vid;
        ne.pid = pid;
        ne.caps = probe_caps(ne.path);
        std::uint32_t idx = 0;
        if (auto s = read_file(n + "/index"); s && rfmt::parse_u32(rfmt::trim(*s), idx)) {
            ne.index = idx;
        }
        entries.push_back(std::move(ne));
    }
    return pick_node(entries);
}

Result<std::unique_ptr<V4l2Capture>> V4l2Capture::open(const std::string& path, std::uint32_t fps,
                                                       std::uint32_t nbufs) {
    if (path.find('\0') != std::string::npos) {
        return fail(IoError::other("bad path"));
    }
    const int f = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (f < 0) {
        return fail(IoError::last_os());
    }
    std::unique_ptr<V4l2Capture> c(new V4l2Capture());
    c->fd_ = f;

    v4l2_capability cap{};
    if (auto r = xioctl(f, VIDIOC_QUERYCAP, &cap); !r) {
        return fail(r.error());
    }
    const std::uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) != 0 ? cap.device_caps : cap.capabilities;
    if ((caps & V4L2_CAP_VIDEO_CAPTURE) == 0 || (caps & V4L2_CAP_STREAMING) == 0) {
        return fail(IoError::other(std::format("{}: not a streaming capture node", rfmt::utf8_lossy(path))));
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (auto r = xioctl(f, VIDIOC_G_FMT, &fmt); !r) {
        return fail(r.error());
    }
    v4l2_pix_format pix = fmt.fmt.pix;
    pix.width = 1280;
    pix.height = 832;
    pix.pixelformat = PIX_FMT_YUYV;
    pix.field = V4L2_FIELD_NONE;
    std::memset(&fmt.fmt, 0, sizeof fmt.fmt);
    fmt.fmt.pix = pix;
    if (auto r = xioctl(f, VIDIOC_S_FMT, &fmt); !r) {
        return fail(r.error());
    }
    pix = fmt.fmt.pix;
    if (pix.pixelformat != PIX_FMT_YUYV || pix.width != 1280 || pix.height != 832) {
        return fail(IoError::other(std::format("driver chose {:#x} {}x{}, expected YUYV 1280x832", pix.pixelformat,
                                               pix.width, pix.height)));
    }

    std::vector<double> offered;
    for (std::uint32_t i = 0; i < 16; i++) {
        v4l2_frmivalenum e{};
        e.index = i;
        e.pixel_format = PIX_FMT_YUYV;
        e.width = 1280;
        e.height = 832;
        if (!xioctl(f, VIDIOC_ENUM_FRAMEINTERVALS, &e)) {
            break;
        }
        if (e.type == V4L2_FRMIVAL_TYPE_DISCRETE && e.discrete.numerator > 0) {
            offered.push_back(static_cast<double>(e.discrete.denominator) / static_cast<double>(e.discrete.numerator));
        }
    }

    if (fps > 0) {
        v4l2_streamparm p{};
        p.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        p.parm.capture.timeperframe.numerator = 1;
        p.parm.capture.timeperframe.denominator = fps;
        if (auto r = xioctl(f, VIDIOC_S_PARM, &p); !r) {
            return fail(r.error());
        }
    }
    double got_fps = 0.0;
    {
        v4l2_streamparm p{};
        p.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(f, VIDIOC_G_PARM, &p)) {
            const auto t = p.parm.capture.timeperframe;
            if (t.numerator > 0) {
                got_fps = static_cast<double>(t.denominator) / static_cast<double>(t.numerator);
            }
        }
    }

    v4l2_requestbuffers rb{};
    rb.count = std::max<std::uint32_t>(nbufs, 2);
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    if (auto r = xioctl(f, VIDIOC_REQBUFS, &rb); !r) {
        return fail(r.error());
    }
    if (rb.count < 2) {
        return fail(IoError::other("driver granted fewer than 2 buffers"));
    }
    for (std::uint32_t i = 0; i < rb.count; i++) {
        v4l2_buffer b = new_buffer(i);
        if (auto r = xioctl(f, VIDIOC_QUERYBUF, &b); !r) {
            return fail(r.error());
        }
        void* ptr = ::mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, f, static_cast<off_t>(b.m.offset));
        if (ptr == MAP_FAILED) {
            return fail(IoError::last_os());
        }
        c->maps_.push_back(Mapping{ptr, b.length});
    }
    c->info.path = path;
    c->info.driver = cstr(cap.driver, sizeof cap.driver);
    c->info.card = cstr(cap.card, sizeof cap.card);
    c->info.bus = cstr(cap.bus_info, sizeof cap.bus_info);
    c->info.width = pix.width;
    c->info.height = pix.height;
    c->info.bytesperline = pix.bytesperline;
    c->info.sizeimage = pix.sizeimage;
    c->info.fps = got_fps;
    c->info.offered_fps = offered;
    for (std::uint32_t i = 0; i < c->maps_.size(); i++) {
        if (auto r = c->queue(i); !r) {
            return fail(r.error());
        }
    }
    int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (auto r = xioctl(c->fd_, VIDIOC_STREAMON, &t); !r) {
        return fail(r.error());
    }
    c->streaming_ = true;
    return c;
}

Result<void> V4l2Capture::queue(std::uint32_t index) {
    v4l2_buffer b = new_buffer(index);
    return xioctl(fd_, VIDIOC_QBUF, &b);
}

Result<void> V4l2Capture::requeue_held() {
    if (held_) {
        const std::uint32_t i = held_->index;
        held_.reset();
        return queue(i);
    }
    return {};
}

Result<bool> V4l2Capture::advance(int timeout_ms) {
    if (auto r = requeue_held(); !r) {
        return fail(r.error());
    }
    const std::uint64_t deadline = monotonic_ns() + static_cast<std::uint64_t>(std::max(timeout_ms, 0)) * 1'000'000;
    for (;;) {
        const std::uint64_t now = monotonic_ns();
        const int left_ms = now >= deadline ? 0 : static_cast<int>((deadline - now) / 1'000'000) + 1;
        pollfd pfd{fd_, POLLIN, 0};
        const int r = ::poll(&pfd, 1, left_ms);
        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            return fail(IoError::last_os());
        }
        if (r == 0) {
            return false;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 && (pfd.revents & POLLIN) == 0) {
            return fail(IoError::other("camera device error (unplugged?)"));
        }
        v4l2_buffer b = new_buffer(0);
        if (auto d = xioctl(fd_, VIDIOC_DQBUF, &b); !d) {
            if (d.error().kind == ErrorKind::WouldBlock) {
                continue;
            }
            return fail(d.error());
        }
        const std::size_t need = info.sizeimage;
        if ((b.flags & V4L2_BUF_FLAG_ERROR) != 0 || b.bytesused < need || b.index >= maps_.size()) {
            corrupt++;
            if (b.index < maps_.size()) {
                if (auto q = queue(b.index); !q) {
                    return fail(q.error());
                }
            }
            if (monotonic_ns() >= deadline) {
                return false;
            }
            continue;
        }
        if ((b.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) != V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
            non_monotonic_ts++;
        }
        const Mapping& m = maps_[b.index];
        held_ = Held{b.index, std::min<std::size_t>(b.bytesused, m.len),
                     static_cast<std::uint64_t>(b.timestamp.tv_sec) * 1'000'000'000ull +
                         static_cast<std::uint64_t>(b.timestamp.tv_usec) * 1000ull,
                     b.sequence};
        return true;
    }
}

std::optional<RawFrame> V4l2Capture::current() const {
    if (!held_) {
        return std::nullopt;
    }
    const Mapping& m = maps_[held_->index];
    return RawFrame{std::span<const std::uint8_t>(static_cast<const std::uint8_t*>(m.ptr), held_->len),
                    info.bytesperline, held_->host_ts_ns, held_->sequence};
}

std::string V4l2Capture::describe() const {
    std::vector<double> o = info.offered_fps;
    return std::format("{} ({}, {}, {}) {}x{} YUYV bpl {} size {} at {} fps (offered {})", rfmt::utf8_lossy(info.path),
                       info.driver, info.card, info.bus, info.width, info.height, info.bytesperline, info.sizeimage,
                       rfmt::fixed(info.fps, 2), rfmt::debug_f64s(o));
}

V4l2Capture::~V4l2Capture() {
    if (fd_ < 0) {
        return;
    }
    if (streaming_) {
        int t = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)xioctl(fd_, VIDIOC_STREAMOFF, &t);
    }
    for (const auto& m : maps_) {
        ::munmap(m.ptr, m.len);
    }
    maps_.clear();
    v4l2_requestbuffers rb{};
    rb.count = 0;
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rb.memory = V4L2_MEMORY_MMAP;
    (void)xioctl(fd_, VIDIOC_REQBUFS, &rb);
    ::close(fd_);
}

} // namespace pibridge::cam
