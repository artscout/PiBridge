// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/src/hid.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/allowlist.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <cstring>

namespace pibridge::hmd {

namespace {

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
        s.remove_prefix(1);
    }
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
        s.remove_suffix(1);
    }
    return s;
}

std::vector<std::string_view> split_ws(std::string_view s) {
    std::vector<std::string_view> v;
    std::size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
            i++;
        }
        const std::size_t b = i;
        while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) {
            i++;
        }
        if (i > b) {
            v.push_back(s.substr(b, i - b));
        }
    }
    return v;
}

std::optional<std::uint8_t> hex_byte(std::string_view t) {
    unsigned v = 0;
    const auto [p, ec] = std::from_chars(t.data(), t.data() + t.size(), v, 16);
    if (t.empty() || ec != std::errc{} || p != t.data() + t.size() || v > 0xff) {
        return std::nullopt;
    }
    return static_cast<std::uint8_t>(v);
}

std::expected<std::uint32_t, SpecError> num(std::string_view s0) {
    const std::string_view s = trim(s0);
    std::string_view body = s;
    int base = 10;
    if (body.starts_with("0x")) {
        body.remove_prefix(2);
        base = 16;
    }
    if (body.starts_with('+')) {
        body.remove_prefix(1); // u32::from_str_radix accepts one '+'
    }
    std::uint32_t v = 0;
    const auto [p, ec] = std::from_chars(body.data(), body.data() + body.size(), v, base);
    if (body.empty() || ec != std::errc{} || p != body.data() + body.size()) {
        return std::unexpected(SpecError{"bad number \"" + std::string(s) + "\""});
    }
    return v;
}

} // namespace

bool Constraint::allows(std::uint32_t v) const {
    if (is_range) {
        return v >= lo && v <= hi;
    }
    return std::find(set.begin(), set.end(), v) != set.end();
}

std::string Refused::message() const {
    switch (kind) {
    case Kind::Denied: return "refused (denied command): " + why;
    case Kind::NotListed: return "refused: not an approved command for this headset";
    case Kind::OutOfRange: return "refused: " + cmd + "." + param + " = " + std::to_string(value) + " is out of range";
    case Kind::Padding: return "refused: " + cmd + " has non-zero bytes after the approved pattern";
    case Kind::Length: return "refused: report length " + std::to_string(got);
    case Kind::UnknownCommand: return "refused: unknown command " + cmd;
    case Kind::MissingParam: return "refused: " + cmd + " needs " + param;
    case Kind::TooSoon: return "refused: " + cmd + " sent again too soon";
    }
    return "refused";
}

std::string SendError::message() const {
    if (const auto* r = refusal()) {
        return r->message();
    }
    return std::string("hidraw write failed: ") + std::strerror(std::get<int>(e));
}

std::expected<Denied, SpecError> Denied::parse(std::string_view prefix, std::string_view why) {
    Denied d;
    for (const auto t : split_ws(prefix)) {
        if (t == "??") {
            d.prefix.push_back(std::nullopt);
            continue;
        }
        const auto b = hex_byte(t);
        if (!b) {
            return std::unexpected(SpecError{"bad denied byte " + std::string(t)});
        }
        d.prefix.push_back(*b);
    }
    if (d.prefix.empty()) {
        return std::unexpected(SpecError{"empty denied prefix"});
    }
    d.why = std::string(why);
    return d;
}

bool Denied::hits(std::span<const std::uint8_t> r) const {
    if (r.size() < prefix.size()) {
        return false;
    }
    for (std::size_t i = 0; i < prefix.size(); i++) {
        if (prefix[i] && *prefix[i] != r[i]) {
            return false;
        }
    }
    return true;
}

std::expected<std::vector<Tok>, SpecError> parse_pattern(std::string_view s) {
    std::vector<Tok> out;
    for (const auto t : split_ws(s)) {
        if (t.starts_with('{') && t.ends_with('}') && t.size() >= 2) {
            const auto f = t.substr(1, t.size() - 2);
            const auto colon = f.find(':');
            if (colon == std::string_view::npos) {
                return std::unexpected(SpecError{"field " + std::string(t) + " needs a type"});
            }
            const auto ty = f.substr(colon + 1);
            std::size_t width = 0;
            if (ty == "u8") {
                width = 1;
            } else if (ty == "u16le") {
                width = 2;
            } else {
                return std::unexpected(SpecError{"field type " + std::string(ty) + " (u8 or u16le)"});
            }
            out.push_back(Tok{true, 0, std::string(f.substr(0, colon)), width});
        } else if (t.size() == 2) {
            const auto b = hex_byte(t);
            if (!b) {
                return std::unexpected(SpecError{"bad byte " + std::string(t)});
            }
            out.push_back(Tok{false, *b, {}, 0});
        } else {
            return std::unexpected(SpecError{"bad token " + std::string(t)});
        }
    }
    return out;
}

std::expected<std::pair<std::string, Constraint>, SpecError> parse_constraint(std::string_view s) {
    const auto eq = s.find('=');
    if (eq == std::string_view::npos) {
        return std::unexpected(SpecError{"constraint \"" + std::string(s) + "\" needs name="});
    }
    const auto name = trim(s.substr(0, eq));
    const auto c = s.substr(eq + 1);
    Constraint k;
    if (const auto dots = c.find(".."); dots != std::string_view::npos) {
        const auto lo = num(c.substr(0, dots));
        if (!lo) {
            return std::unexpected(lo.error());
        }
        const auto hi = num(c.substr(dots + 2));
        if (!hi) {
            return std::unexpected(hi.error());
        }
        if (*lo > *hi) {
            return std::unexpected(SpecError{"empty range in \"" + std::string(s) + "\""});
        }
        k.is_range = true;
        k.lo = *lo;
        k.hi = *hi;
    } else {
        k.is_range = false;
        std::size_t start = 0;
        while (true) {
            const auto bar = c.find('|', start);
            const auto part = c.substr(start, bar == std::string_view::npos ? std::string_view::npos : bar - start);
            const auto v = num(part);
            if (!v) {
                return std::unexpected(v.error());
            }
            k.set.push_back(*v);
            if (bar == std::string_view::npos) {
                break;
            }
            start = bar + 1;
        }
    }
    return std::pair{std::string(name), k};
}

std::expected<CommandSpec, SpecError> CommandSpec::from_section(const toml::Section& s) {
    const auto need = [&](std::string_view k) -> std::expected<std::string, SpecError> {
        if (auto v = s.str(k)) {
            return *v;
        }
        return std::unexpected(SpecError{"[[hid]] at line " + std::to_string(s.line) + " lacks " + std::string(k)});
    };
    CommandSpec spec;
    auto id = need("id");
    if (!id) {
        return std::unexpected(id.error());
    }
    spec.id = *id;
    auto bytes = need("bytes");
    if (!bytes) {
        return std::unexpected(bytes.error());
    }
    auto toks = parse_pattern(*bytes);
    if (!toks) {
        return std::unexpected(toks.error());
    }
    spec.toks = std::move(*toks);
    for (const auto& p : s.strs("params")) {
        auto c = parse_constraint(p);
        if (!c) {
            return std::unexpected(c.error());
        }
        spec.params.push_back(std::move(*c));
    }
    // every field needs exactly one constraint, and every constraint a field
    for (const auto& t : spec.toks) {
        if (t.field && std::none_of(spec.params.begin(), spec.params.end(), [&](const auto& p) { return p.first == t.name; })) {
            return std::unexpected(SpecError{spec.id + ": field " + t.name + " has no constraint"});
        }
    }
    for (const auto& [n, c] : spec.params) {
        if (std::none_of(spec.toks.begin(), spec.toks.end(), [&](const Tok& t) { return t.field && t.name == n; })) {
            return std::unexpected(SpecError{spec.id + ": constraint " + n + " has no field"});
        }
    }
    const std::string pol = s.str("policy").value_or("session");
    if (pol == "session") {
        spec.policy = Policy::Session;
    } else if (pol == "once_per_request") {
        spec.policy = Policy::OncePerRequest;
    } else if (pol == "once_per_enumeration") {
        spec.policy = Policy::OncePerEnumeration;
    } else {
        return std::unexpected(SpecError{spec.id + ": policy " + pol});
    }
    spec.report_len = static_cast<std::size_t>(s.integer("report_len").value_or(64));
    spec.interface = static_cast<std::uint8_t>(s.integer("interface").value_or(0));
    if (const auto m = s.integer("min_interval_ms")) {
        spec.min_interval = std::chrono::milliseconds(*m);
    }
    auto src = need("source");
    if (!src) {
        return std::unexpected(src.error());
    }
    spec.source = *src;
    spec.approved = s.str("approved").value_or("");
    if (spec.pattern_len() > spec.report_len) {
        return std::unexpected(SpecError{spec.id + ": pattern longer than the report"});
    }
    if (spec.policy != Policy::Session && spec.approved.empty()) {
        return std::unexpected(SpecError{spec.id + ": a state-changing command needs `approved`"});
    }
    return spec;
}

std::size_t CommandSpec::pattern_len() const {
    std::size_t n = 0;
    for (const auto& t : toks) {
        n += t.field ? t.width : 1;
    }
    return n;
}

std::expected<void, Refused> CommandSpec::matches(std::span<const std::uint8_t> r) const {
    using K = Refused::Kind;
    if (r.size() != report_len) {
        return std::unexpected(Refused{K::NotListed});
    }
    std::size_t i = 0;
    std::optional<Refused> range_err;
    for (const auto& t : toks) {
        if (!t.field) {
            if (r[i] != t.lit) {
                return std::unexpected(Refused{K::NotListed});
            }
            i++;
            continue;
        }
        const std::uint32_t v = t.width == 1 ? r[i] : static_cast<std::uint32_t>(r[i] | (r[i + 1] << 8));
        const auto it = std::find_if(params.begin(), params.end(), [&](const auto& p) { return p.first == t.name; });
        if (it != params.end() && !it->second.allows(v) && !range_err) {
            Refused e{K::OutOfRange};
            e.cmd = id;
            e.param = t.name;
            e.value = v;
            range_err = e;
        }
        i += t.width;
    }
    if (range_err) {
        return std::unexpected(*range_err);
    }
    if (std::any_of(r.begin() + static_cast<std::ptrdiff_t>(i), r.end(), [](std::uint8_t b) { return b != 0; })) {
        Refused e{K::Padding};
        e.cmd = id;
        return std::unexpected(e);
    }
    return {};
}

std::expected<std::vector<std::uint8_t>, Refused>
CommandSpec::encode(std::span<const std::pair<std::string_view, std::uint32_t>> vals) const {
    using K = Refused::Kind;
    std::vector<std::uint8_t> r(report_len, 0);
    std::size_t i = 0;
    for (const auto& t : toks) {
        if (!t.field) {
            r[i++] = t.lit;
            continue;
        }
        const auto it = std::find_if(vals.begin(), vals.end(), [&](const auto& p) { return p.first == t.name; });
        if (it == vals.end()) {
            Refused e{K::MissingParam};
            e.cmd = id;
            e.param = t.name;
            return std::unexpected(e);
        }
        const std::uint32_t v = it->second;
        const std::uint32_t max = t.width == 1 ? 0xff : 0xffff;
        if (v > max) {
            Refused e{K::OutOfRange};
            e.cmd = id;
            e.param = t.name;
            e.value = v;
            return std::unexpected(e);
        }
        for (std::size_t k = 0; k < t.width; k++) {
            r[i + k] = static_cast<std::uint8_t>(v >> (8 * k));
        }
        i += t.width;
    }
    return r;
}

std::expected<const CommandSpec*, Refused> Allowlist::check(std::uint8_t interface,
                                                           std::span<const std::uint8_t> report) const {
    using K = Refused::Kind;
    const auto bare = !report.empty() && report[0] == 0 ? report.subspan(1) : report;
    for (const auto& d : denied) {
        if (d.hits(bare)) {
            Refused e{K::Denied};
            e.why = d.why;
            return std::unexpected(e);
        }
    }
    std::optional<Refused> best;
    bool len_ok = false;
    bool any = false;
    for (const auto& c : commands) {
        if (c.interface != interface) {
            continue;
        }
        any = true;
        len_ok |= bare.size() == c.report_len;
        const auto m = c.matches(bare);
        if (m) {
            return &c;
        }
        if (m.error().kind != K::NotListed && !best) {
            best = m.error();
        }
    }
    if (best) {
        return std::unexpected(*best);
    }
    if (any && !len_ok) {
        Refused e{K::Length};
        e.got = bare.size();
        return std::unexpected(e);
    }
    return std::unexpected(Refused{K::NotListed});
}

std::expected<std::vector<std::uint8_t>, Refused> Allowlist::build(std::string_view id, const ParamValues& vals) const {
    const CommandSpec* c = get(id);
    if (!c) {
        Refused e{Refused::Kind::UnknownCommand};
        e.cmd = std::string(id);
        return std::unexpected(e);
    }
    auto r = c->encode(vals);
    if (!r) {
        return r;
    }
    if (auto m = c->matches(*r); !m) {
        return std::unexpected(m.error());
    }
    for (const auto& d : denied) {
        if (d.hits(*r)) {
            Refused e{Refused::Kind::Denied};
            e.why = d.why;
            return std::unexpected(e);
        }
    }
    return r;
}

const CommandSpec* Allowlist::get(std::string_view id) const {
    for (const auto& c : commands) {
        if (c.id == id) {
            return &c;
        }
    }
    return nullptr;
}

Gate::Gate(const Allowlist& allow, std::uint8_t interface, Writer out)
    : allow_(&allow), interface_(interface), out_(std::move(out)) {}

std::expected<std::string, SendError> Gate::send(std::span<const std::uint8_t> report) {
    const auto spec = allow_->check(interface_, report);
    if (!spec) {
        return std::unexpected(SendError{spec.error()});
    }
    const CommandSpec& c = **spec;
    if (c.min_interval) {
        const auto it = last_.find(c.id);
        if (it != last_.end() && Clock::now() - it->second < *c.min_interval) {
            Refused e{Refused::Kind::TooSoon};
            e.cmd = c.id;
            return std::unexpected(SendError{e});
        }
    }
    const auto bare = !report.empty() && report[0] == 0 ? report.subspan(1) : report;
    std::vector<std::uint8_t> buf;
    buf.reserve(bare.size() + 1);
    buf.push_back(0);
    buf.insert(buf.end(), bare.begin(), bare.end());
    const long n = out_(buf);
    if (n < 0) {
        return std::unexpected(SendError{static_cast<int>(-n)});
    }
    if (static_cast<std::size_t>(n) != buf.size()) {
        return std::unexpected(SendError{EIO});
    }
    last_[c.id] = Clock::now();
    return c.id;
}

Gate::Writer vector_writer(std::vector<std::uint8_t>& v) {
    return [&v](std::span<const std::uint8_t> b) -> long {
        v.insert(v.end(), b.begin(), b.end());
        return static_cast<long>(b.size());
    };
}

} // namespace pibridge::hmd
