// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/json.rs (MIT OR Apache-2.0).
#include "pibridge/cam/json.h"

#include "pibridge/cam/rustfmt.h"

#include <cmath>
#include <format>

namespace pibridge::cam::json {

const Value* Value::get(std::string_view key) const {
    if (type != Type::Obj) {
        return nullptr;
    }
    for (const auto& [k, v] : obj) {
        if (k == key) {
            return &v;
        }
    }
    return nullptr;
}

std::optional<double> Value::as_f64() const {
    if (type != Type::Num) {
        return std::nullopt;
    }
    return num;
}

const std::string* Value::as_str() const {
    return type == Type::Str ? &str : nullptr;
}

const std::vector<Value>* Value::as_array() const {
    return type == Type::Arr ? &arr : nullptr;
}

std::string JsonError::text() const {
    return std::format("JSON error at byte {}: {}", pos, msg);
}

namespace {

/// Nesting limit (protects the recursion against hostile input).
constexpr std::size_t MAX_DEPTH = 64;

struct Parser {
    std::string_view s;
    std::size_t pos = 0;

    std::unexpected<JsonError> err(const char* msg) const { return std::unexpected(JsonError{pos, msg}); }

    void ws() {
        while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t' || s[pos] == '\n' || s[pos] == '\r')) {
            pos++;
        }
    }

    bool eat(std::string_view lit) {
        if (s.substr(pos).starts_with(lit)) {
            pos += lit.size();
            return true;
        }
        return false;
    }

    std::expected<Value, JsonError> value(std::size_t depth) {
        if (depth > MAX_DEPTH) {
            return err("nesting too deep");
        }
        ws();
        if (pos >= s.size()) {
            return err("unexpected end");
        }
        const char c = s[pos];
        if (c == '{') {
            pos++;
            Value m;
            m.type = Value::Type::Obj;
            ws();
            if (eat("}")) {
                return m;
            }
            for (;;) {
                ws();
                if (pos >= s.size() || s[pos] != '"') {
                    return err("expected a string key");
                }
                auto k = string();
                if (!k) {
                    return std::unexpected(k.error());
                }
                ws();
                if (!eat(":")) {
                    return err("expected ':'");
                }
                auto v = value(depth + 1);
                if (!v) {
                    return v;
                }
                m.obj.emplace_back(std::move(*k), std::move(*v));
                ws();
                if (eat(",")) {
                    continue;
                }
                if (eat("}")) {
                    return m;
                }
                return err("expected ',' or '}'");
            }
        }
        if (c == '[') {
            pos++;
            Value a;
            a.type = Value::Type::Arr;
            ws();
            if (eat("]")) {
                return a;
            }
            for (;;) {
                auto v = value(depth + 1);
                if (!v) {
                    return v;
                }
                a.arr.push_back(std::move(*v));
                ws();
                if (eat(",")) {
                    continue;
                }
                if (eat("]")) {
                    return a;
                }
                return err("expected ',' or ']'");
            }
        }
        if (c == '"') {
            auto st = string();
            if (!st) {
                return std::unexpected(st.error());
            }
            return Value::string(std::move(*st));
        }
        if (c == 't' && eat("true")) {
            return Value::boolean(true);
        }
        if (c == 'f' && eat("false")) {
            return Value::boolean(false);
        }
        if (c == 'n' && eat("null")) {
            return Value::null();
        }
        if (c == '-' || (c >= '0' && c <= '9')) {
            return number();
        }
        return err("unexpected character");
    }

    bool digits() {
        const std::size_t st = pos;
        while (pos < s.size() && s[pos] >= '0' && s[pos] <= '9') {
            pos++;
        }
        return pos > st;
    }

    std::expected<Value, JsonError> number() {
        const std::size_t start = pos;
        eat("-");
        if (eat("0")) {
            // no leading zeros
        } else if (!digits()) {
            return err("bad number");
        }
        if (eat(".") && !digits()) {
            return err("bad fraction");
        }
        if (pos < s.size() && (s[pos] == 'e' || s[pos] == 'E')) {
            pos++;
            if (!eat("+")) {
                eat("-");
            }
            if (!digits()) {
                return err("bad exponent");
            }
        }
        double x = 0;
        if (!rfmt::parse_f64(s.substr(start, pos - start), x) || !std::isfinite(x)) {
            return std::unexpected(JsonError{start, "number out of range"});
        }
        return Value::number(x);
    }

    std::expected<std::uint32_t, JsonError> hex4() {
        if (pos + 4 <= s.size()) {
            std::uint32_t v = 0;
            bool ok = true;
            std::string_view t = s.substr(pos, 4);
            // u32::from_str_radix(t, 16) accepts a leading '+'
            std::size_t i = 0;
            if (t[0] == '+') {
                i = 1;
            }
            for (; i < 4; i++) {
                const char c = t[i];
                int d = -1;
                if (c >= '0' && c <= '9') {
                    d = c - '0';
                } else if (c >= 'a' && c <= 'f') {
                    d = c - 'a' + 10;
                } else if (c >= 'A' && c <= 'F') {
                    d = c - 'A' + 10;
                }
                if (d < 0) {
                    ok = false;
                    break;
                }
                v = v * 16 + static_cast<std::uint32_t>(d);
            }
            if (ok && rfmt::utf8_valid(t)) {
                pos += 4;
                return v;
            }
        }
        return std::unexpected(JsonError{pos, "bad \\u escape"});
    }

    static void push_utf8(std::string& o, std::uint32_t cp) {
        if (cp < 0x80) {
            o += static_cast<char>(cp);
        } else if (cp < 0x800) {
            o += static_cast<char>(0xc0 | (cp >> 6));
            o += static_cast<char>(0x80 | (cp & 0x3f));
        } else if (cp < 0x10000) {
            o += static_cast<char>(0xe0 | (cp >> 12));
            o += static_cast<char>(0x80 | ((cp >> 6) & 0x3f));
            o += static_cast<char>(0x80 | (cp & 0x3f));
        } else {
            o += static_cast<char>(0xf0 | (cp >> 18));
            o += static_cast<char>(0x80 | ((cp >> 12) & 0x3f));
            o += static_cast<char>(0x80 | ((cp >> 6) & 0x3f));
            o += static_cast<char>(0x80 | (cp & 0x3f));
        }
    }

    std::expected<std::string, JsonError> string() {
        pos++; // opening quote
        std::string out;
        for (;;) {
            if (pos >= s.size()) {
                return err("unterminated string");
            }
            const auto c = static_cast<unsigned char>(s[pos]);
            pos++;
            if (c == '"') {
                break;
            }
            if (c == '\\') {
                if (pos >= s.size()) {
                    return err("unterminated escape");
                }
                const char e = s[pos];
                pos++;
                std::uint32_t ch = 0;
                switch (e) {
                case '"': ch = '"'; break;
                case '\\': ch = '\\'; break;
                case '/': ch = '/'; break;
                case 'b': ch = 0x08; break;
                case 'f': ch = 0x0c; break;
                case 'n': ch = '\n'; break;
                case 'r': ch = '\r'; break;
                case 't': ch = '\t'; break;
                case 'u': {
                    auto hi = hex4();
                    if (!hi) {
                        return std::unexpected(hi.error());
                    }
                    std::uint32_t cp = *hi;
                    if (*hi >= 0xd800 && *hi < 0xdc00) {
                        if (!eat("\\u")) {
                            return err("lone surrogate");
                        }
                        auto lo = hex4();
                        if (!lo) {
                            return std::unexpected(lo.error());
                        }
                        if (!(*lo >= 0xdc00 && *lo < 0xe000)) {
                            return err("bad surrogate pair");
                        }
                        cp = 0x10000 + ((*hi - 0xd800) << 10) + (*lo - 0xdc00);
                    }
                    if (cp > 0x10ffff || (cp >= 0xd800 && cp <= 0xdfff)) {
                        return err("bad code point");
                    }
                    ch = cp;
                    break;
                }
                default: return err("bad escape");
                }
                push_utf8(out, ch);
                continue;
            }
            if (c <= 0x1f) {
                return err("control character in string");
            }
            out += static_cast<char>(c);
        }
        if (!rfmt::utf8_valid(out)) {
            return err("invalid UTF-8");
        }
        return out;
    }
};

} // namespace

std::expected<Value, JsonError> parse(std::string_view text) {
    Parser p{text, 0};
    auto v = p.value(0);
    if (!v) {
        return v;
    }
    p.ws();
    if (p.pos != p.s.size()) {
        return p.err("trailing characters");
    }
    return v;
}

std::string quote(std::string_view s) {
    std::string o = "\"";
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        switch (c) {
        case '"': o += "\\\""; break;
        case '\\': o += "\\\\"; break;
        case '\n': o += "\\n"; break;
        case '\r': o += "\\r"; break;
        case '\t': o += "\\t"; break;
        default:
            if (c < 0x20) {
                o += std::format("\\u{:04x}", c);
            } else {
                o += ch;
            }
        }
    }
    o += '"';
    return o;
}

std::string num(double x) {
    if (std::isfinite(x)) {
        return rfmt::debug_f64(x);
    }
    return "null";
}

std::string num_array(std::span<const double> v) {
    std::string o = "[";
    for (std::size_t i = 0; i < v.size(); i++) {
        if (i > 0) {
            o += ", ";
        }
        o += num(v[i]);
    }
    return o + "]";
}

} // namespace pibridge::cam::json
