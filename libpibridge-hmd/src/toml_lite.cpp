// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-device/src/toml_lite.rs (MIT OR Apache-2.0).
#include "pibridge/hmd/toml_lite.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>

namespace pibridge::hmd::toml {

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

std::expected<void, std::string> check_key(std::string_view k) {
    const bool ok = !k.empty() && std::all_of(k.begin(), k.end(), [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-';
    });
    if (ok) {
        return {};
    }
    return std::unexpected("bad key \"" + std::string(k) + "\" (bare keys only)");
}

// Cut a `#` comment that is not inside a string.
std::string_view strip_comment(std::string_view l) {
    bool in_str = false;
    bool esc = false;
    for (std::size_t i = 0; i < l.size(); i++) {
        const char c = l[i];
        if (esc) {
            esc = false;
        } else if (c == '\\' && in_str) {
            esc = true;
        } else if (c == '"') {
            in_str = !in_str;
        } else if (c == '#' && !in_str) {
            return l.substr(0, i);
        }
    }
    return l;
}

// Split on commas outside strings.
std::expected<std::vector<std::string_view>, std::string> split_top(std::string_view s) {
    std::vector<std::string_view> parts;
    std::size_t start = 0;
    bool in_str = false;
    bool esc = false;
    for (std::size_t i = 0; i < s.size(); i++) {
        const char c = s[i];
        if (esc) {
            esc = false;
        } else if (c == '\\' && in_str) {
            esc = true;
        } else if (c == '"') {
            in_str = !in_str;
        } else if (c == ',' && !in_str) {
            parts.push_back(s.substr(start, i - start));
            start = i + 1;
        }
    }
    if (in_str) {
        return std::unexpected(std::string("unterminated string in array"));
    }
    parts.push_back(s.substr(start));
    return parts;
}

std::expected<std::string, std::string> unescape(std::string_view s) {
    std::string out;
    for (std::size_t i = 0; i < s.size(); i++) {
        const char c = s[i];
        if (c != '\\') {
            if (c == '"') {
                return std::unexpected(std::string("unescaped quote in string"));
            }
            out += c;
            continue;
        }
        if (i + 1 >= s.size()) {
            return std::unexpected(std::string("unsupported escape \\"));
        }
        const char n = s[++i];
        switch (n) {
        case '"': out += '"'; break;
        case '\\': out += '\\'; break;
        case 'n': out += '\n'; break;
        case 't': out += '\t'; break;
        default: return std::unexpected("unsupported escape \\" + std::string(1, n));
        }
    }
    return out;
}

std::expected<Value, std::string> value(std::string_view s) {
    if (s.starts_with('[')) {
        if (!s.ends_with(']') || s.size() < 2) {
            return std::unexpected(std::string("arrays must close on the same line"));
        }
        const auto parts = split_top(s.substr(1, s.size() - 2));
        if (!parts) {
            return std::unexpected(parts.error());
        }
        Array items;
        for (const auto part : *parts) {
            const auto p = trim(part);
            if (p.empty()) {
                continue; // trailing comma
            }
            if (p.starts_with('[')) {
                return std::unexpected(std::string("nested arrays are not supported"));
            }
            auto v = value(p);
            if (!v) {
                return v;
            }
            items.push_back(std::move(*v));
        }
        return Value{std::move(items)};
    }
    if (s.starts_with('"')) {
        if (s.size() < 2 || !s.ends_with('"')) {
            return std::unexpected(std::string("unterminated string"));
        }
        auto u = unescape(s.substr(1, s.size() - 2));
        if (!u) {
            return std::unexpected(u.error());
        }
        return Value{std::move(*u)};
    }
    if (s == "true") {
        return Value{true};
    }
    if (s == "false") {
        return Value{false};
    }
    bool neg = false;
    std::string_view body0 = s;
    if (body0.starts_with('-')) {
        neg = true;
        body0.remove_prefix(1);
    }
    std::string body;
    for (const char c : body0) {
        if (c != '_') {
            body += c;
        }
    }
    const std::string quoted = "\"" + std::string(s) + "\"";
    if (body.starts_with("0x")) {
        std::int64_t v = 0;
        const char* b = body.data() + 2;
        const char* e = body.data() + body.size();
        const auto [p, ec] = std::from_chars(b, e, v, 16);
        if (b == e || ec != std::errc{} || p != e) {
            return std::unexpected("bad hex " + quoted);
        }
        return Value{neg ? -v : v};
    }
    // Rust's str::parse accepts one leading '+'; std::from_chars does not
    std::string_view num = body;
    if (num.starts_with('+')) {
        num.remove_prefix(1);
    }
    if (!num.empty() && !num.starts_with('+')) {
        std::int64_t v = 0;
        const char* b = num.data();
        const char* e = num.data() + num.size();
        if (const auto [p, ec] = std::from_chars(b, e, v, 10); ec == std::errc{} && p == e) {
            return Value{neg ? -v : v};
        }
        double d = 0;
        if (const auto [p, ec] = std::from_chars(b, e, d); ec == std::errc{} && p == e) {
            return Value{neg ? -d : d};
        }
    }
    return std::unexpected("unsupported value " + quoted);
}

} // namespace

const Value* Section::get(std::string_view key) const {
    for (const auto& [k, v] : entries) {
        if (k == key) {
            return &v;
        }
    }
    return nullptr;
}

std::optional<std::string> Section::str(std::string_view key) const {
    const Value* v = get(key);
    if (v && std::holds_alternative<std::string>(v->v)) {
        return std::get<std::string>(v->v);
    }
    return std::nullopt;
}

std::optional<std::int64_t> Section::integer(std::string_view key) const {
    const Value* v = get(key);
    if (v && std::holds_alternative<std::int64_t>(v->v)) {
        return std::get<std::int64_t>(v->v);
    }
    return std::nullopt;
}

std::optional<double> Section::number(std::string_view key) const {
    const Value* v = get(key);
    if (v && std::holds_alternative<double>(v->v)) {
        return std::get<double>(v->v);
    }
    if (v && std::holds_alternative<std::int64_t>(v->v)) {
        return static_cast<double>(std::get<std::int64_t>(v->v));
    }
    return std::nullopt;
}

std::optional<bool> Section::boolean(std::string_view key) const {
    const Value* v = get(key);
    if (v && std::holds_alternative<bool>(v->v)) {
        return std::get<bool>(v->v);
    }
    return std::nullopt;
}

std::vector<std::int64_t> Section::ints(std::string_view key) const {
    std::vector<std::int64_t> out;
    const Value* v = get(key);
    if (v && std::holds_alternative<Array>(v->v)) {
        for (const auto& x : std::get<Array>(v->v)) {
            if (std::holds_alternative<std::int64_t>(x.v)) {
                out.push_back(std::get<std::int64_t>(x.v));
            }
        }
    }
    return out;
}

std::vector<std::string> Section::strs(std::string_view key) const {
    std::vector<std::string> out;
    const Value* v = get(key);
    if (v && std::holds_alternative<Array>(v->v)) {
        for (const auto& x : std::get<Array>(v->v)) {
            if (std::holds_alternative<std::string>(x.v)) {
                out.push_back(std::get<std::string>(x.v));
            }
        }
    }
    return out;
}

std::expected<std::vector<Section>, ParseError> parse(std::string_view src) {
    std::vector<Section> out(1);
    std::size_t n = 0;
    while (!src.empty() || n == 0) {
        if (src.empty()) {
            break;
        }
        const auto nl = src.find('\n');
        std::string_view raw = src.substr(0, nl);
        src = nl == std::string_view::npos ? std::string_view{} : src.substr(nl + 1);
        if (raw.ends_with('\r')) {
            raw.remove_suffix(1); // str::lines() drops a CR before the LF
        }
        n++;
        const auto err = [&](std::string msg) { return std::unexpected(ParseError{n, std::move(msg)}); };
        const std::string_view line = trim(strip_comment(raw));
        if (line.empty()) {
            continue;
        }
        if (line.starts_with("[[")) {
            if (!line.ends_with("]]") || line.size() < 4) {
                return err("unclosed [[table]]");
            }
            const auto name = trim(line.substr(2, line.size() - 4));
            if (auto k = check_key(name); !k) {
                return err(k.error());
            }
            out.push_back(Section{std::string(name), true, {}, n});
        } else if (line.starts_with('[')) {
            if (!line.ends_with(']') || line.size() < 2) {
                return err("unclosed [table]");
            }
            const auto name = trim(line.substr(1, line.size() - 2));
            if (auto k = check_key(name); !k) {
                return err(k.error());
            }
            if (std::any_of(out.begin(), out.end(), [&](const Section& s) { return !s.array && s.name == name; })) {
                return err("table defined twice");
            }
            out.push_back(Section{std::string(name), false, {}, n});
        } else {
            const auto eq = line.find('=');
            if (eq == std::string_view::npos) {
                return err("expected key = value");
            }
            const auto k = trim(line.substr(0, eq));
            if (auto c = check_key(k); !c) {
                return err(c.error());
            }
            auto v = value(trim(line.substr(eq + 1)));
            if (!v) {
                return err(v.error());
            }
            Section& sec = out.back();
            if (sec.get(k)) {
                return err("key defined twice");
            }
            sec.entries.emplace_back(std::string(k), std::move(*v));
        }
    }
    return out;
}

} // namespace pibridge::hmd::toml
