// SPDX-License-Identifier: MIT
// The Tobii chip's own calibration (calibration.h): the captured target table, the requests, the calibration guard,
// the sequence of one calibration and the live-session channel.
#include "pibridge/eye/calibration.h"

#include "pibridge/eye/frames.h"
#include "pibridge/eye/replay.h"
#include "pibridge/eye/session.h"

#include "bytes.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <cmath>
#include <format>

namespace pibridge::eye::calibration {

extern const unsigned char targets_data[];
extern const std::size_t targets_size;

using namespace std::chrono_literals;

bool is_calibration_op(std::uint32_t op) { return std::ranges::find(OPS, op) != OPS.end(); }

std::string op_name(std::uint32_t op) {
    switch (op) {
    case OP_START: return "start (0x3f2)";
    case OP_TARGET: return "target (0x407)";
    case OP_COMPUTE: return "compute (0x42e)";
    case OP_DOWNLOAD: return "download (0x44c)";
    case OP_FINISH: return "finish (0x3fc)";
    default: return std::format("{:#x}", op);
    }
}

std::string_view targets_text() { return {reinterpret_cast<const char*>(targets_data), targets_size}; }

namespace {

std::optional<Bytes> unhex(std::string_view s) {
    if (s.size() % 2 != 0) {
        return std::nullopt;
    }
    Bytes out;
    out.reserve(s.size() / 2);
    for (std::size_t i = 0; i < s.size(); i += 2) {
        std::uint8_t v = 0;
        const auto [p, ec] = std::from_chars(s.data() + i, s.data() + i + 2, v, 16);
        if (ec != std::errc() || p != s.data() + i + 2) {
            return std::nullopt;
        }
        out.push_back(v);
    }
    return out;
}

std::optional<std::int32_t> field(std::string_view line, std::string_view key) {
    const std::string k = std::string(" ") + std::string(key) + "=";
    const auto p = line.find(k);
    if (p == std::string_view::npos) {
        return std::nullopt;
    }
    const char* b = line.data() + p + k.size();
    const char* e = line.data() + line.size();
    std::int32_t v = 0;
    const auto [q, ec] = std::from_chars(b, e, v);
    if (ec != std::errc() || (q != e && *q != ' ')) {
        return std::nullopt;
    }
    return v;
}

// The captured layout: 00 00 | 04 len 8 x | 04 len 8 y | 04 len 8 z (S22.42 mm) | 02 len 4 eye mask.
std::expected<void, std::string> check_target_payload(const Target& t) {
    const Bytes& p = t.payload;
    if (p.size() != TARGET_PAYLOAD_LEN) {
        return std::unexpected(std::format("payload of {} bytes (the captured ones have {})", p.size(), TARGET_PAYLOAD_LEN));
    }
    if (p[0] != 0 || p[1] != 0) {
        return std::unexpected(std::string("payload does not start with 00 00"));
    }
    const std::array<std::int32_t, 3> want = {t.x_mm, t.y_mm, t.z_mm};
    for (std::size_t i = 0; i < 3; i++) {
        const std::size_t o = 2 + 13 * i;
        if (p[o] != 0x04 || detail::be32(p, o + 1) != 8) {
            return std::unexpected(std::format("item {} is not an 8-byte S22.42 value", i + 1));
        }
        const auto raw = static_cast<std::int64_t>(detail::be64(p, o + 5));
        const double mm = static_cast<double>(raw) / static_cast<double>(std::int64_t{1} << 42);
        if (std::fabs(mm - want[i]) > 0.01) {
            return std::unexpected(std::format("item {} encodes {:.4f} mm, the line says {}", i + 1, mm, want[i]));
        }
    }
    if (p[41] != 0x02 || detail::be32(p, 42) != 4) {
        return std::unexpected(std::string("the eye mask item is missing"));
    }
    return {};
}

} // namespace

std::expected<std::vector<Target>, std::string> parse_targets(std::string_view text) {
    std::vector<Target> out;
    std::size_t lineno = 0;
    while (!text.empty()) {
        const auto nl = text.find('\n');
        std::string_view line = text.substr(0, nl);
        text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
        lineno++;
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) {
            line.remove_suffix(1);
        }
        if (line.empty() || line.front() == '#') {
            continue;
        }
        if (!line.starts_with("target ")) {
            return std::unexpected(std::format("line {}: not a target line", lineno));
        }
        Target t;
        const auto x = field(line, "x");
        const auto y = field(line, "y");
        const auto z = field(line, "z");
        const auto pp = line.find(" payload=");
        if (!x || !y || !z || pp == std::string_view::npos) {
            return std::unexpected(std::format("line {}: needs x=, y=, z= and payload=", lineno));
        }
        auto hexs = line.substr(pp + 9);
        if (const auto sp = hexs.find(' '); sp != std::string_view::npos) {
            hexs = hexs.substr(0, sp);
        }
        auto b = unhex(hexs);
        if (!b) {
            return std::unexpected(std::format("line {}: payload is not hex", lineno));
        }
        t.x_mm = *x;
        t.y_mm = *y;
        t.z_mm = *z;
        t.payload = std::move(*b);
        if (auto c = check_target_payload(t); !c) {
            return std::unexpected(std::format("line {}: {}", lineno, c.error()));
        }
        out.push_back(std::move(t));
    }
    if (out.empty()) {
        return std::unexpected(std::string("no target lines"));
    }
    return out;
}

const std::vector<Target>& targets() {
    static const std::vector<Target> t = [] {
        auto r = parse_targets(targets_text());
        // the table is a build input checked by the tests; a broken one gives no targets (every request fails)
        return r ? *r : std::vector<Target>{};
    }();
    return t;
}

std::vector<Step> sequence() {
    std::vector<Step> s;
    s.push_back({OP_START, std::nullopt, 1});
    for (const auto i : PASS1_TARGETS) {
        s.push_back({OP_TARGET, i, 1});
    }
    s.push_back({OP_COMPUTE, std::nullopt, 1});
    s.push_back({OP_DOWNLOAD, std::nullopt, 1});
    s.push_back({OP_FINISH, std::nullopt, 1});
    s.push_back({OP_START, std::nullopt, 2});
    for (const auto i : PASS2_TARGETS) {
        s.push_back({OP_TARGET, i, 2});
    }
    s.push_back({OP_COMPUTE, std::nullopt, 2});
    s.push_back({OP_DOWNLOAD, std::nullopt, 2});
    s.push_back({OP_FINISH, std::nullopt, 2});
    // Pimax Play downloads once more after the last finish; that copy is the one it stores
    s.push_back({OP_DOWNLOAD, std::nullopt, 2});
    return s;
}

std::expected<ttp::Message, std::string> request(std::uint32_t seq, const Step& step) {
    if (!is_calibration_op(step.op)) {
        return std::unexpected(std::format("{:#x} is not a calibration opcode", step.op));
    }
    ttp::Message m;
    if (step.op == OP_TARGET) {
        const auto& t = targets();
        if (!step.target || *step.target >= t.size()) {
            return std::unexpected(std::string("a target request needs one of the captured targets"));
        }
        m.payload = t[*step.target].payload;
    } else if (step.target) {
        return std::unexpected(std::format("{} carries no target", op_name(step.op)));
    }
    m.header = ttp::Header{ttp::Kind::Request, seq, 0, step.op, 0, static_cast<std::uint32_t>(m.payload.size())};
    return m;
}

Bytes frame(const ttp::Message& m) {
    auto f = m.to_frames(ttp::Direction::Out, frames::OUT_MAX_BODY);
    Bytes out;
    for (auto& x : f) {
        out.insert(out.end(), x.begin(), x.end());
    }
    return out;
}

// ------------------------------------------------------------------------------------------------ guard

Guard Guard::captured() { return with_targets(targets()); }

Guard Guard::with_targets(std::span<const Target> t) {
    Guard g;
    for (const auto& x : t) {
        g.payloads_.insert(x.payload);
    }
    return g;
}

std::expected<void, std::string> Guard::check(ByteView f) const {
    if (!open_) {
        return std::unexpected(std::string("no calibration session is open"));
    }
    if (f.size() < ttp::WRAPPER_LEN + ttp::HEADER_LEN) {
        return std::unexpected(std::string("not a whole request"));
    }
    const auto w = ttp::Wrapper::parse(f);
    if (!w || w->dir != ttp::Direction::Out || w->len != f.size() - ttp::WRAPPER_LEN) {
        return std::unexpected(std::string("not one whole bulk OUT frame"));
    }
    const auto h = ttp::Header::parse(f.subspan(ttp::WRAPPER_LEN));
    if (!h || h->kind != ttp::Kind::Request || h->status != 0 || h->aux != 0) {
        return std::unexpected(std::string("not a plain request header"));
    }
    if (!is_calibration_op(h->id)) {
        return std::unexpected(std::format("opcode {:#x} is not a calibration request", h->id));
    }
    const std::size_t payload = f.size() - ttp::WRAPPER_LEN - ttp::HEADER_LEN;
    if (h->payload_len != payload) {
        return std::unexpected(std::string("payload length does not match the frame"));
    }
    if (h->id == OP_TARGET) {
        if (!payloads_.contains(Bytes(f.begin() + ttp::WRAPPER_LEN + ttp::HEADER_LEN, f.end()))) {
            return std::unexpected(std::string("target payload is not one of the captured targets"));
        }
    } else if (payload != 0) {
        return std::unexpected(std::format("{} must have no payload", op_name(h->id)));
    }
    return {};
}

Result<void> Guard::bulk_out(Transport& t, ByteView f, Duration timeout) const {
    if (auto c = check(f); !c) {
        return std::unexpected(Error::make(Errc::permission_denied, "calibration guard: " + c.error()));
    }
    return t.bulk_out(replay::EP_BULK_OUT, f, timeout);
}

// ------------------------------------------------------------------------------------------------ answers and run

std::expected<void, std::string> check_answer(const Step& step, std::uint32_t seq, const Answer& a) {
    if (a.op != step.op || a.seq != seq) {
        return std::unexpected(std::format("answer {:#x} seq {} does not belong to {} seq {}", a.op, a.seq,
                                           op_name(step.op), seq));
    }
    if (a.status != 1) {
        return std::unexpected(std::format("the chip refused {} (status {})", op_name(step.op), a.status));
    }
    return {};
}

std::expected<Bytes, std::string> downloaded_profile(const Answer& a) {
    const auto body = frames::profile_body(a.payload);
    if (!body) {
        return std::unexpected(std::format("the download ({} bytes) is not a profile", a.payload.size()));
    }
    if (auto c = frames::check_profile(*body); !c) {
        return std::unexpected("the downloaded profile: " + c.error());
    }
    return Bytes(body->begin(), body->end());
}

Run::Run() : steps_(sequence()) {}

const Step* Run::current() const { return done() || failed() ? nullptr : &steps_[i_]; }

std::expected<void, std::string> Run::answer(std::uint32_t seq, const Answer& a) {
    const Step* s = current();
    if (s == nullptr) {
        return std::unexpected(std::string("the calibration is not running"));
    }
    auto fail = [&](std::string m) -> std::expected<void, std::string> {
        error_ = m;
        return std::unexpected(std::move(m));
    };
    if (auto c = check_answer(*s, seq, a); !c) {
        return fail(c.error());
    }
    if (s->op == OP_START) {
        pass_open_ = true;
    } else if (s->op == OP_FINISH) {
        pass_open_ = false;
    } else if (s->op == OP_DOWNLOAD) {
        downloads_.push_back(a.payload.size());
        if (i_ + 1 == steps_.size()) {
            auto p = downloaded_profile(a);
            if (!p) {
                return fail(p.error());
            }
            profile_ = std::move(*p);
        }
    }
    i_++;
    return {};
}

std::optional<Step> Run::abort_step() const {
    if (!pass_open_) {
        return std::nullopt;
    }
    const std::uint8_t pass = i_ < steps_.size() ? steps_[i_].pass : steps_.back().pass;
    return Step{OP_FINISH, std::nullopt, pass};
}

Duration answer_wait(std::uint32_t op) { return op == OP_COMPUTE || op == OP_DOWNLOAD ? Duration(10s) : Duration(3s); }

std::expected<Answer, std::string> exchange(Transport& t, const Guard& g, ttp::StreamDecoder& dec, std::uint32_t seq,
                                            const Step& step, Duration wait,
                                            const std::function<void(const ttp::Message&)>& other) {
    auto m = request(seq, step);
    if (!m) {
        return std::unexpected(m.error());
    }
    if (auto r = g.bulk_out(t, frame(*m), 2s); !r) {
        return std::unexpected(r.error().to_string());
    }
    const auto until = Clock::now() + wait;
    while (Clock::now() < until) {
        auto b = t.bulk_in(replay::EP_BULK_IN, session::IN_BUFFER, 100ms);
        if (!b) {
            return std::unexpected("IN read: " + b.error().to_string());
        }
        if (!*b) {
            continue;
        }
        for (auto& r : dec.push(**b)) {
            if (!r) {
                continue;
            }
            const auto& h = r->header;
            if (h.kind == ttp::Kind::Response && h.id == step.op && h.seq == seq) {
                Answer a{h.seq, h.id, h.status, h.aux, std::move(r->payload)};
                if (auto c = check_answer(step, seq, a); !c) {
                    return std::unexpected(c.error());
                }
                return a;
            }
            if (other) {
                other(*r);
            }
        }
    }
    return std::unexpected(std::format("no answer to {} within {:.1f} s", op_name(step.op),
                                       std::chrono::duration<double>(wait).count()));
}

// ------------------------------------------------------------------------------------------------ link

Link::Link(Guard g) : guard_(std::move(g)) {}

bool Link::attached() const {
    std::lock_guard l(mu_);
    return channel_.has_value();
}

std::expected<void, std::string> Link::open_session() {
    std::lock_guard l(mu_);
    if (!channel_) {
        return std::unexpected(std::string("the eye chip is not connected"));
    }
    guard_.open();
    return {};
}

void Link::close_session() {
    std::lock_guard l(mu_);
    guard_.close();
    for (auto& [_, p] : pending_) {
        if (!p.sent && !p.result) {
            p.result = std::unexpected(std::string("the calibration session was closed"));
        }
    }
    cv_.notify_all();
}

bool Link::session_open() const {
    std::lock_guard l(mu_);
    return guard_.is_open();
}

std::expected<std::uint64_t, std::string> Link::submit(const Step& step) {
    std::lock_guard l(mu_);
    if (!channel_) {
        return std::unexpected(std::string("the eye chip is not connected"));
    }
    if (!guard_.is_open()) {
        return std::unexpected(std::string("no calibration session is open"));
    }
    const std::uint32_t seq = std::max({next_seq_, max_seq_ + 1, FIRST_SEQ});
    auto m = request(seq, step);
    if (!m) {
        return std::unexpected(m.error());
    }
    Bytes f = frame(*m);
    if (auto c = guard_.check(f); !c) {
        return std::unexpected("calibration guard: " + c.error());
    }
    next_seq_ = seq + 1;
    const std::uint64_t ticket = next_ticket_++;
    pending_[ticket] = Pending{step, seq, std::move(f), false, {}, std::nullopt};
    return ticket;
}

void Link::expire_locked(Clock::time_point now) {
    for (auto& [_, p] : pending_) {
        if (p.sent && !p.result && now - p.sent_at > answer_wait(p.step.op)) {
            p.result = std::unexpected(std::format("no answer to {} within {:.0f} s", op_name(p.step.op),
                                                   std::chrono::duration<double>(answer_wait(p.step.op)).count()));
        }
    }
}

std::optional<std::expected<Answer, std::string>> Link::poll(std::uint64_t ticket) {
    std::lock_guard l(mu_);
    expire_locked(Clock::now());
    const auto it = pending_.find(ticket);
    if (it == pending_.end()) {
        return std::expected<Answer, std::string>(std::unexpected(std::string("unknown calibration request")));
    }
    if (!it->second.result) {
        return std::nullopt;
    }
    auto r = std::move(*it->second.result);
    pending_.erase(it);
    return r;
}

std::expected<Answer, std::string> Link::wait(std::uint64_t ticket, Duration timeout) {
    const auto until = Clock::now() + timeout;
    while (true) {
        if (auto r = poll(ticket)) {
            return std::move(*r);
        }
        if (Clock::now() >= until) {
            return std::unexpected(std::string("no answer in time"));
        }
        std::unique_lock l(mu_);
        cv_.wait_for(l, 20ms);
    }
}

std::uint32_t Link::seq_of(std::uint64_t ticket) const {
    std::lock_guard l(mu_);
    const auto it = pending_.find(ticket);
    return it == pending_.end() ? 0 : it->second.seq;
}

void Link::attach(std::uint64_t id) {
    std::lock_guard l(mu_);
    channel_ = id;
    max_seq_ = 0;
    next_seq_ = 0;
}

void Link::detach(std::uint64_t id) {
    std::lock_guard l(mu_);
    if (channel_ != id) {
        return;
    }
    channel_.reset();
    guard_.close();
    for (auto& [_, p] : pending_) {
        if (!p.result) {
            p.result = std::unexpected(std::string("the eye chip link ended during the calibration"));
        }
    }
    cv_.notify_all();
}

void Link::note_out(ByteView f) {
    const auto s = ttp::captured_start(ttp::Direction::Out, f);
    if (!s || s->header.kind != ttp::Kind::Request) {
        return;
    }
    std::lock_guard l(mu_);
    max_seq_ = std::max(max_seq_, s->header.seq);
}

std::optional<Link::Outbox> Link::take_unsent() {
    std::lock_guard l(mu_);
    if (!guard_.is_open()) {
        return std::nullopt;
    }
    std::optional<Outbox> out;
    const auto now = Clock::now();
    for (auto& [ticket, p] : pending_) {
        if (p.sent || p.result) {
            continue;
        }
        if (!out) {
            out.emplace(Outbox{guard_, {}});
        }
        p.sent = true;
        p.sent_at = now;
        out->frames.emplace_back(ticket, p.frame);
    }
    return out;
}

void Link::sent(std::uint64_t ticket, std::expected<void, std::string> r) {
    if (r) {
        return;
    }
    std::lock_guard l(mu_);
    if (auto it = pending_.find(ticket); it != pending_.end() && !it->second.result) {
        it->second.result = std::unexpected(r.error());
    }
    cv_.notify_all();
}

void Link::deliver(const ttp::Message& m) {
    const auto& h = m.header;
    if (h.kind != ttp::Kind::Response || !is_calibration_op(h.id)) {
        return;
    }
    std::lock_guard l(mu_);
    for (auto& [_, p] : pending_) {
        if (p.sent && !p.result && p.seq == h.seq && p.step.op == h.id) {
            Answer a{h.seq, h.id, h.status, h.aux, m.payload};
            if (auto c = check_answer(p.step, p.seq, a); !c) {
                p.result = std::unexpected(c.error());
            } else {
                p.result = std::move(a);
            }
            cv_.notify_all();
            return;
        }
    }
}

// ------------------------------------------------------------------------------------------------ channel

namespace {
std::atomic<std::uint64_t> next_channel_id{1};
} // namespace

Channel::Channel(std::shared_ptr<Transport> inner, std::shared_ptr<Link> link)
    : inner_(std::move(inner)), link_(std::move(link)), id_(next_channel_id.fetch_add(1)) {
    link_->attach(id_);
}

Channel::~Channel() { link_->detach(id_); }

Result<Bytes> Channel::control(const Setup& setup, ByteView out, Duration timeout) {
    return inner_->control(setup, out, timeout);
}

Result<void> Channel::bulk_out(std::uint8_t ep, ByteView data, Duration timeout) {
    link_->note_out(data);
    return inner_->bulk_out(ep, data, timeout);
}

Result<std::optional<Bytes>> Channel::bulk_in(std::uint8_t ep, std::size_t max_len, Duration timeout) {
    if (auto ob = link_->take_unsent()) {
        for (const auto& [ticket, f] : ob->frames) {
            auto r = ob->guard.bulk_out(*inner_, f, 2s);
            link_->sent(ticket, r ? std::expected<void, std::string>{}
                                  : std::expected<void, std::string>(std::unexpected(r.error().to_string())));
        }
    }
    auto r = inner_->bulk_in(ep, max_len, timeout);
    if (r && *r) {
        for (auto& m : dec_.push(**r)) {
            if (m) {
                link_->deliver(*m);
            }
        }
    }
    return r;
}

} // namespace pibridge::eye::calibration
