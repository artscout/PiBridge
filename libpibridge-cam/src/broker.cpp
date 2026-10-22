// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/broker.rs (MIT OR Apache-2.0).
#include "pibridge/cam/broker.h"

#include "pibridge/cam/clock.h"
#include "pibridge/cam/frame.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <format>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>

namespace pibridge::cam::broker {

namespace {

constexpr std::size_t PAGE = 4096;
constexpr std::size_t HEADER_BYTES = PAGE;
constexpr std::size_t SLOT_HEADER = 64;
constexpr std::size_t LATEST_OFFSET = 64;    // u64: newest complete frame index + 1 (0 = none yet)
constexpr std::size_t PUBLISHED_OFFSET = 72; // u64: frames published
constexpr std::size_t CAM_GEN_OFFSET = 80;   // u64: the server's camera generation (changes on every reopen)
constexpr std::size_t CAM_STATE_OFFSET = 88; // u64: 0 = unknown (older broker), 1 = streaming, 2 = reopening
constexpr std::size_t BEAT_OFFSET = 96;      // u64: server loop heartbeat, CLOCK_MONOTONIC ns (0 = older broker)
constexpr std::size_t HELLO_FIXED = 4 + 6 * 4;

using SteadyClock = std::chrono::steady_clock;

std::string_view truncate_utf8(std::string_view s, std::size_t max) {
    if (s.size() <= max) {
        return s;
    }
    std::size_t end = max;
    // a char boundary: not a UTF-8 continuation byte
    while (end > 0 && (static_cast<unsigned char>(s[end]) & 0xc0) == 0x80) {
        end--;
    }
    return s.substr(0, end);
}

void put_u32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    for (int i = 0; i < 4; i++) {
        v.push_back(static_cast<std::uint8_t>(x >> (8 * i)));
    }
}

std::uint32_t get_u32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::uint64_t get_u64(const std::uint8_t* p) {
    std::uint64_t v = 0;
    for (int i = 7; i >= 0; i--) {
        v = (v << 8) | p[i];
    }
    return v;
}

void put_le(std::uint8_t* p, std::uint64_t v, int n) {
    for (int i = 0; i < n; i++) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

/// Closes a descriptor on scope exit unless released.
struct Fd {
    int fd = -1;
    explicit Fd(int f = -1) : fd(f) {}
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    Fd(Fd&& o) noexcept : fd(o.fd) { o.fd = -1; }
    Fd& operator=(Fd&& o) noexcept {
        if (this != &o) {
            reset();
            fd = o.fd;
            o.fd = -1;
        }
        return *this;
    }
    ~Fd() { reset(); }
    void reset() {
        if (fd >= 0) {
            ::close(fd);
        }
        fd = -1;
    }
    int release() {
        const int f = fd;
        fd = -1;
        return f;
    }
};

Result<std::uint8_t*> map(int fd, std::size_t len, bool write) {
    const int prot = write ? (PROT_READ | PROT_WRITE) : PROT_READ;
    void* p = ::mmap(nullptr, len, prot, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
        return fail(IoError::last_os());
    }
    return static_cast<std::uint8_t*>(p);
}

/*
 * Sockets (SOCK_SEQPACKET, Unix).
 */

Result<std::pair<sockaddr_un, socklen_t>> make_sockaddr(const std::string& path) {
    sockaddr_un a{};
    if (path.empty() || path.size() >= sizeof a.sun_path) {
        return fail(IoError::custom(ErrorKind::InvalidInput,
                                    std::format("socket path too long or empty: {}", rfmt::utf8_lossy(path))));
    }
    a.sun_family = AF_UNIX;
    std::memcpy(a.sun_path, path.data(), path.size());
    const auto len = static_cast<socklen_t>(sizeof(sa_family_t) + path.size() + 1);
    return std::pair{a, len};
}

Result<Fd> seqpacket(bool nonblock) {
    const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | (nonblock ? SOCK_NONBLOCK : 0), 0);
    if (fd < 0) {
        return fail(IoError::last_os());
    }
    return Fd(fd);
}

Result<Fd> connect_to(const std::string& path) {
    auto a = make_sockaddr(path);
    if (!a) {
        return fail(a.error());
    }
    auto fd = seqpacket(false);
    if (!fd) {
        return fail(fd.error());
    }
    if (::connect(fd->fd, reinterpret_cast<const sockaddr*>(&a->first), a->second) != 0) {
        return fail(IoError::last_os());
    }
    return std::move(*fd);
}

// Ok(-1) when no connection is pending.
Result<int> accept_one(int listener) {
    const int fd = ::accept4(listener, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
    if (fd < 0) {
        const int e = errno;
        if (e == EAGAIN || e == EINTR || e == ECONNABORTED) {
            return -1;
        }
        return fail(IoError::from_errno(e));
    }
    return fd;
}

Result<void> send_with_fd(int sock, std::span<const std::uint8_t> data, int fd) {
    iovec iov{const_cast<std::uint8_t*>(data.data()), data.size()};
    alignas(8) unsigned char cbuf[64] = {};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = cbuf;
    msg.msg_controllen = CMSG_SPACE(sizeof(int));
    cmsghdr* cm = CMSG_FIRSTHDR(&msg);
    cm->cmsg_level = SOL_SOCKET;
    cm->cmsg_type = SCM_RIGHTS;
    cm->cmsg_len = CMSG_LEN(sizeof(int));
    std::memcpy(CMSG_DATA(cm), &fd, sizeof(int));
    if (::sendmsg(sock, &msg, MSG_NOSIGNAL) < 0) {
        return fail(IoError::last_os());
    }
    return {};
}

Result<std::pair<std::size_t, Fd>> recv_with_fd(int sock, std::span<std::uint8_t> buf) {
    iovec iov{buf.data(), buf.size()};
    alignas(8) unsigned char cbuf[64] = {};
    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = cbuf;
    msg.msg_controllen = sizeof cbuf;
    const ssize_t n = ::recvmsg(sock, &msg, MSG_CMSG_CLOEXEC);
    if (n < 0) {
        return fail(IoError::last_os());
    }
    Fd fd;
    cmsghdr* cm = CMSG_FIRSTHDR(&msg);
    if (cm != nullptr && cm->cmsg_level == SOL_SOCKET && cm->cmsg_type == SCM_RIGHTS) {
        int raw = -1;
        std::memcpy(&raw, CMSG_DATA(cm), sizeof(int));
        fd = Fd(raw);
    }
    if ((msg.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0) {
        return fail(IoError::custom(ErrorKind::InvalidData, "camera broker message truncated"));
    }
    return std::pair{static_cast<std::size_t>(n), std::move(fd)};
}

// Wait up to timeout_ms (< 0: forever) for input on fd.
Result<bool> poll_in(int fd, int timeout_ms) {
    pollfd p{fd, POLLIN, 0};
    for (;;) {
        const int r = ::poll(&p, 1, timeout_ms);
        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            return fail(IoError::last_os());
        }
        return r > 0;
    }
}

enum class Sent { Ok, Full, Gone };

Sent send_notice(int sock, std::uint64_t k) {
    std::uint8_t b[8];
    put_le(b, k, 8);
    const ssize_t r = ::send(sock, b, sizeof b, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (r == 8) {
        return Sent::Ok;
    }
    const int e = errno;
    if (e == EAGAIN || e == ENOBUFS || e == EINTR) {
        return Sent::Full;
    }
    return Sent::Gone;
}

} // namespace

std::string default_socket() {
    if (auto d = env_os("XDG_RUNTIME_DIR"); d && !d->empty()) {
        return path_join(*d, "omarchyvr-cam-broker.sock");
    }
    return std::format("/tmp/omarchyvr-cam-broker-{}.sock", ::getuid());
}

/*
 * Hello
 */

std::vector<std::uint8_t> Hello::encode() const {
    const std::string_view d = truncate_utf8(desc, MAX_DESC);
    std::vector<std::uint8_t> v;
    v.reserve(HELLO_FIXED + d.size());
    v.insert(v.end(), MAGIC, MAGIC + 4);
    for (const std::uint32_t x : {VERSION, slots, slot_bytes, data_bytes, flags, static_cast<std::uint32_t>(d.size())}) {
        put_u32(v, x);
    }
    v.insert(v.end(), d.begin(), d.end());
    return v;
}

Result<Hello> Hello::decode(std::span<const std::uint8_t> b) {
    const auto bad = [](const std::string& m) {
        return fail(IoError::custom(ErrorKind::InvalidData, "camera broker hello: " + m));
    };
    if (b.size() < HELLO_FIXED || std::memcmp(b.data(), MAGIC, 4) != 0) {
        return bad("not a pimax-cam broker");
    }
    const auto u = [&](std::size_t i) { return get_u32(b.data() + 4 + 4 * i); };
    if (u(0) != VERSION) {
        return bad(std::format("version {} (this client speaks {})", u(0), VERSION));
    }
    Hello h;
    h.slots = u(1);
    h.slot_bytes = u(2);
    h.data_bytes = u(3);
    h.flags = u(4);
    const std::size_t dlen = u(5);
    if (b.size() != HELLO_FIXED + dlen || dlen > MAX_DESC) {
        return bad("bad length");
    }
    if (h.slots == 0 || h.slots > 1024 || h.slot_bytes < SLOT_HEADER + static_cast<std::size_t>(h.data_bytes) ||
        h.slot_bytes % PAGE != 0) {
        return bad("bad ring layout");
    }
    h.desc = rfmt::utf8_lossy(
        std::string_view(reinterpret_cast<const char*>(b.data()) + HELLO_FIXED, b.size() - HELLO_FIXED));
    return h;
}

std::size_t Hello::ring_len() const {
    return HEADER_BYTES + static_cast<std::size_t>(slots) * slot_bytes;
}

/*
 * Ring
 */

Result<std::unique_ptr<Ring>> Ring::create(std::uint32_t slots, std::size_t data_bytes, std::uint32_t flags,
                                           std::string_view desc) {
    const std::size_t slot_bytes = (SLOT_HEADER + data_bytes + PAGE - 1) / PAGE * PAGE;
    Hello layout;
    layout.slots = std::clamp<std::uint32_t>(slots, 2, 1024);
    layout.slot_bytes = static_cast<std::uint32_t>(slot_bytes);
    layout.data_bytes = static_cast<std::uint32_t>(data_bytes);
    layout.flags = flags;
    layout.desc = std::string(truncate_utf8(desc, MAX_DESC));
    const std::size_t len = layout.ring_len();
    Fd fd(::memfd_create("omarchyvr-cam-broker", MFD_CLOEXEC));
    if (fd.fd < 0) {
        return fail(IoError::last_os());
    }
    if (::ftruncate(fd.fd, static_cast<off_t>(len)) != 0) {
        return fail(IoError::last_os());
    }
    auto p = map(fd.fd, len, true);
    if (!p) {
        return fail(p.error());
    }
    std::unique_ptr<Ring> r(new Ring());
    r->ptr_ = *p;
    r->len_ = len;
    r->layout_ = std::move(layout);
    r->fd_ = fd.release();
    std::memcpy(r->ptr_, MAGIC, 4);
    put_le(r->ptr_ + 4, VERSION, 4);
    return r;
}

Result<std::unique_ptr<Ring>> Ring::open(int fd_in, Hello layout) {
    Fd fd(fd_in);
    const std::size_t len = layout.ring_len();
    struct stat st{};
    if (::fstat(fd.fd, &st) != 0) {
        return fail(IoError::last_os());
    }
    if (static_cast<std::size_t>(st.st_size) < len) {
        return fail(IoError::custom(ErrorKind::InvalidData, "camera broker ring smaller than announced"));
    }
    auto p = map(fd.fd, len, false);
    if (!p) {
        return fail(p.error());
    }
    std::unique_ptr<Ring> r(new Ring());
    r->ptr_ = *p;
    r->len_ = len;
    r->layout_ = std::move(layout);
    r->fd_ = fd.release();
    if (std::memcmp(r->ptr_, MAGIC, 4) != 0) {
        return fail(IoError::custom(ErrorKind::InvalidData, "camera broker ring has no header"));
    }
    return r;
}

Ring::~Ring() {
    if (ptr_ != nullptr) {
        ::munmap(ptr_, len_);
    }
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

std::uint64_t* Ring::word(std::size_t offset) const {
    // 8-aligned (page-aligned mapping, offsets multiples of 8), inside the mapping, and only ever accessed atomically
    // by every process
    return reinterpret_cast<std::uint64_t*>(ptr_ + offset);
}

std::size_t Ring::slot_offset(std::uint64_t k) const {
    return HEADER_BYTES + static_cast<std::size_t>(k % layout_.slots) * layout_.slot_bytes;
}

std::optional<std::uint64_t> Ring::latest() const {
    const std::uint64_t v = std::atomic_ref<std::uint64_t>(*word(LATEST_OFFSET)).load(std::memory_order_acquire);
    if (v == 0) {
        return std::nullopt;
    }
    return v - 1;
}

std::uint64_t Ring::published() const {
    return std::atomic_ref<std::uint64_t>(*word(PUBLISHED_OFFSET)).load(std::memory_order_acquire);
}

void Ring::set_camera(bool up, std::uint64_t generation, std::uint64_t now_ns) const {
    std::atomic_ref<std::uint64_t>(*word(CAM_GEN_OFFSET)).store(generation, std::memory_order_release);
    std::atomic_ref<std::uint64_t>(*word(CAM_STATE_OFFSET)).store(up ? 1 : 2, std::memory_order_release);
    std::atomic_ref<std::uint64_t>(*word(BEAT_OFFSET)).store(now_ns, std::memory_order_release);
}

std::tuple<std::optional<bool>, std::uint64_t, std::uint64_t> Ring::camera() const {
    std::optional<bool> up;
    switch (std::atomic_ref<std::uint64_t>(*word(CAM_STATE_OFFSET)).load(std::memory_order_acquire)) {
    case 1: up = true; break;
    case 2: up = false; break;
    default: break;
    }
    return {up, std::atomic_ref<std::uint64_t>(*word(CAM_GEN_OFFSET)).load(std::memory_order_acquire),
            std::atomic_ref<std::uint64_t>(*word(BEAT_OFFSET)).load(std::memory_order_acquire)};
}

Result<void> Ring::publish(std::uint64_t k, const RawFrame& f) const {
    if (f.data.size() > layout_.data_bytes) {
        return fail(IoError::custom(ErrorKind::InvalidInput, std::format("frame of {} B does not fit a {} B slot",
                                                                         f.data.size(), layout_.data_bytes)));
    }
    const std::size_t off = slot_offset(k);
    std::atomic_ref<std::uint64_t> lock(*word(off));
    lock.store(2 * k + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    // the slot (header fields + data) lies inside the read-write mapping; only this server writes it, readers detect
    // a concurrent write through the seqlock
    std::uint8_t* p = ptr_ + off;
    put_le(p + 8, f.host_ts_ns, 8);
    put_le(p + 16, f.sequence, 4);
    put_le(p + 20, f.data.size(), 4);
    put_le(p + 24, f.stride, 4);
    put_le(p + 32, k, 8);
    std::memcpy(p + SLOT_HEADER, f.data.data(), f.data.size());
    lock.store(2 * k + 2, std::memory_order_release);
    std::atomic_ref<std::uint64_t>(*word(LATEST_OFFSET)).store(k + 1, std::memory_order_release);
    std::atomic_ref<std::uint64_t>(*word(PUBLISHED_OFFSET)).fetch_add(1, std::memory_order_acq_rel);
    return {};
}

std::optional<SlotInfo> Ring::read(std::uint64_t k, std::span<std::uint8_t> out) const {
    const std::size_t off = slot_offset(k);
    std::atomic_ref<std::uint64_t> lock(*word(off));
    const std::uint64_t s1 = lock.load(std::memory_order_acquire);
    if (s1 != 2 * k + 2) {
        return std::nullopt;
    }
    std::uint8_t hdr[40];
    // inside the mapping; a concurrent overwrite is detected by the second lock read below
    std::memcpy(hdr, ptr_ + off, 40);
    const std::size_t len = get_u32(hdr + 20);
    if (len > out.size() || len > layout_.data_bytes) {
        return std::nullopt;
    }
    std::memcpy(out.data(), ptr_ + off + SLOT_HEADER, len);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (lock.load(std::memory_order_relaxed) != s1) {
        return std::nullopt; // torn: the server reused the slot while we copied
    }
    if (get_u64(hdr + 32) != k) {
        return std::nullopt;
    }
    return SlotInfo{k, get_u64(hdr + 8), get_u32(hdr + 16), len, get_u32(hdr + 24)};
}

/*
 * Server.
 */

namespace detail {

Result<int> listen(const std::string& path) {
    if (path_exists(path)) {
        if (connect_to(path)) {
            return fail(IoError::custom(ErrorKind::AddrInUse,
                                        std::format("a camera broker already serves {}", rfmt::utf8_lossy(path))));
        }
        if (::unlink(path.c_str()) != 0) {
            return fail(IoError::last_os());
        }
    }
    auto a = make_sockaddr(path);
    if (!a) {
        return fail(a.error());
    }
    auto fd = seqpacket(true);
    if (!fd) {
        return fail(fd.error());
    }
    if (::bind(fd->fd, reinterpret_cast<const sockaddr*>(&a->first), a->second) != 0 || ::listen(fd->fd, 16) != 0) {
        return fail(IoError::last_os());
    }
    return fd->release();
}

} // namespace detail

Result<ServeStats> serve(std::unique_ptr<Source> src, const ServeOptions& opts, const std::atomic<bool>& stop,
                         const std::function<void(const ServeStats&)>& on_status) {
    const std::uint32_t flags = (src->is_replay() ? FLAG_REPLAY : 0) |
                                (src->profile() == StreamProfile::JpegTiles2 ? FLAG_PROFILE_JPEG2 : 0);
    auto ring = Ring::create(opts.slots, FRAME_BYTES, flags, src->describe());
    if (!ring) {
        return fail(ring.error());
    }
    const std::vector<std::uint8_t> hello = (*ring)->layout().encode();
    auto lfd = detail::listen(opts.socket);
    if (!lfd) {
        return fail(lfd.error());
    }
    const Fd listener(*lfd);
    struct Unlink {
        const std::string& p;
        ~Unlink() { ::unlink(p.c_str()); }
    } unlink_guard{opts.socket};

    std::vector<Fd> clients;
    ServeStats st;
    auto next_status = SteadyClock::now() + opts.status_every;
    std::uint64_t k = 0;
    std::uint64_t gen0 = src->generation();
    bool was_up = src->up();
    (*ring)->set_camera(was_up, 0, monotonic_ns());
    while (!stop.load(std::memory_order_relaxed)) {
        for (;;) {
            auto c = accept_one(listener.fd);
            if (!c) {
                return fail(c.error());
            }
            if (*c < 0) {
                break;
            }
            Fd cf(*c);
            if (send_with_fd(cf.fd, hello, (*ring)->fd())) {
                clients.push_back(std::move(cf));
                st.clients_total++;
            }
        }
        auto got = src->advance(100);
        if (!got) {
            return fail(got.error());
        }
        if (*got) {
            if (const auto f = src->current()) {
                if (auto p = (*ring)->publish(k, *f); !p) {
                    return fail(p.error());
                }
                std::erase_if(clients, [&](const Fd& c) {
                    switch (send_notice(c.fd, k)) {
                    case Sent::Ok: return false;
                    case Sent::Full: st.notices_dropped++; return false;
                    case Sent::Gone: return true;
                    }
                    return true;
                });
                k++;
                st.frames = k;
            }
        } else if (src->finished()) {
            break;
        } else if (!src->is_replay()) {
            st.timeouts++;
        }
        const bool up = src->up();
        const std::uint64_t gen = src->generation();
        if (gen != gen0) {
            st.camera_reopens++;
            gen0 = gen;
        }
        if (up != was_up) {
            was_up = up;
            st.clients = clients.size();
            ServeStats now_st = st;
            now_st.camera_up = up;
            on_status(now_st); // report the change at once
        }
        st.camera_up = up;
        (*ring)->set_camera(up, st.camera_reopens, monotonic_ns());
        if (SteadyClock::now() >= next_status) {
            next_status += opts.status_every;
            st.clients = clients.size();
            on_status(st);
        }
    }
    st.clients = clients.size();
    return st;
}

/*
 * Client.
 */

Result<std::unique_ptr<BrokerSource>> BrokerSource::connect(const std::string& path) {
    auto sock = connect_to(path);
    if (!sock) {
        return fail(IoError::custom(sock.error().kind,
                                    std::format("camera broker {}: {} (start one with `pimax-cam broker`)",
                                                rfmt::utf8_lossy(path), sock.error().text())));
    }
    auto ready = poll_in(sock->fd, 3000);
    if (!ready) {
        return fail(ready.error());
    }
    if (!*ready) {
        return fail(IoError::custom(ErrorKind::TimedOut, "camera broker: no hello within 3 s"));
    }
    std::vector<std::uint8_t> b(HELLO_FIXED + MAX_DESC);
    auto got = recv_with_fd(sock->fd, b);
    if (!got) {
        return fail(got.error());
    }
    auto hello = Hello::decode(std::span(b).first(got->first));
    if (!hello) {
        return fail(hello.error());
    }
    if (got->second.fd < 0) {
        return fail(IoError::custom(ErrorKind::InvalidData, "camera broker hello without ring"));
    }
    const std::size_t data_bytes = hello->data_bytes;
    auto ring = Ring::open(got->second.release(), std::move(*hello));
    if (!ring) {
        return fail(ring.error());
    }
    std::unique_ptr<BrokerSource> s(new BrokerSource());
    s->sock_ = sock->release();
    s->ring_ = std::move(*ring);
    s->path_ = path;
    s->buf_.assign(data_bytes, 0);
    return s;
}

BrokerSource::~BrokerSource() {
    if (sock_ >= 0) {
        ::close(sock_);
    }
}

Result<std::optional<std::uint64_t>> BrokerSource::recv_notice() {
    std::uint8_t b[8];
    const ssize_t n = ::recv(sock_, b, sizeof b, MSG_DONTWAIT);
    if (n == 0) {
        eof_ = true;
        return fail(IoError::custom(ErrorKind::ConnectionAborted,
                                    std::format("camera broker {} stopped", rfmt::utf8_lossy(path_))));
    }
    if (n < 0) {
        const int e = errno;
        if (e == EAGAIN || e == EINTR) {
            return std::optional<std::uint64_t>{};
        }
        return fail(IoError::from_errno(e));
    }
    if (n != 8) {
        return fail(IoError::custom(ErrorKind::InvalidData, "camera broker notice size"));
    }
    return std::optional{get_u64(b)};
}

Result<bool> BrokerSource::advance(int timeout_ms) {
    std::optional<SteadyClock::time_point> deadline;
    if (timeout_ms >= 0) {
        deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    }
    for (;;) {
        int wait = -1;
        if (deadline) {
            const auto now = SteadyClock::now();
            wait = now >= *deadline
                       ? 0
                       : static_cast<int>(
                             std::chrono::duration_cast<std::chrono::milliseconds>(*deadline - now).count());
        }
        auto ready = poll_in(sock_, wait);
        if (!ready) {
            return fail(ready.error());
        }
        if (!*ready) {
            return false;
        }
        auto kk = recv_notice();
        if (!kk) {
            return fail(kk.error());
        }
        if (!*kk) {
            if (deadline && SteadyClock::now() >= *deadline) {
                return false;
            }
            continue;
        }
        const std::uint64_t k = **kk;
        const std::optional<std::uint64_t> last = cur_ ? std::optional(cur_->index) : std::nullopt;
        // Fell more than the ring behind: jump to the newest frame instead of reading overwritten slots.
        const std::uint64_t newest = ring_->latest().value_or(k);
        const std::uint64_t want = newest >= k + ring_->layout().slots - 1 ? newest : k;
        if (last && want <= *last) {
            continue; // an older notice for a frame we already passed
        }
        if (const auto info = ring_->read(want, buf_)) {
            if (last) {
                skipped_ += want - *last - 1;
            }
            cur_ = info;
            return true;
        }
        // overwritten while we got to it: take the next notice
    }
}

std::optional<RawFrame> BrokerSource::current() const {
    if (!cur_) {
        return std::nullopt;
    }
    return RawFrame{std::span<const std::uint8_t>(buf_).first(cur_->len), cur_->stride, cur_->host_ts_ns,
                    cur_->sequence};
}

std::string BrokerSource::describe() const {
    return std::format("broker {} ({})", rfmt::utf8_lossy(path_), ring_->layout().desc);
}

/*
 * Probe.
 */

std::string Probe::to_json() const {
    std::string d;
    for (const char c : desc) {
        if (c == '\\') {
            d += "\\\\";
        } else if (c == '"') {
            d += "\\\"";
        } else {
            d += c;
        }
    }
    return std::format("{{\"camera_up\":{},\"camera_reopens\":{},\"published\":{},\"fps\":{},\"heartbeat_age_s\":{},"
                       "\"desc\":\"{}\"}}",
                       camera_up ? (*camera_up ? "true" : "false") : "null", camera_reopens, published,
                       rfmt::fixed(fps, 1), heartbeat_age_s ? rfmt::fixed(*heartbeat_age_s, 2) : "null", d);
}

Result<Probe> probe(const std::string& path, std::chrono::steady_clock::duration window) {
    auto c = BrokerSource::connect(path);
    if (!c) {
        return fail(c.error());
    }
    const std::uint64_t n0 = (*c)->published();
    const auto t0 = SteadyClock::now();
    std::this_thread::sleep_for(window);
    const std::uint64_t n1 = (*c)->published();
    const auto [up, gen, beat] = (*c)->camera();
    const std::uint64_t now = monotonic_ns();
    Probe p;
    p.desc = (*c)->hello().desc;
    p.camera_up = up;
    p.camera_reopens = gen;
    p.published = n1;
    p.fps = static_cast<double>(n1 - n0) /
            rfmt::fmax(std::chrono::duration<double>(SteadyClock::now() - t0).count(), 1e-3);
    if (beat != 0) {
        p.heartbeat_age_s = static_cast<double>(now > beat ? now - beat : 0) * 1e-9;
    }
    return p;
}

} // namespace pibridge::cam::broker
