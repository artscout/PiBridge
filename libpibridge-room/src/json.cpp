// SPDX-License-Identifier: MIT
#include "pibridge/room/json.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <format>

namespace pibridge::room {

Json Json::boolean(bool b) {
    Json j;
    j.type_ = Type::Bool;
    j.bool_ = b;
    return j;
}

Json Json::number(double v) { return numberText(formatNumber(v)); }

Json Json::numberText(std::string text) {
    Json j;
    j.type_ = Type::Number;
    j.text_ = std::move(text);
    return j;
}

Json Json::string(std::string s) {
    Json j;
    j.type_ = Type::String;
    j.text_ = std::move(s);
    return j;
}

Json Json::array(std::vector<Json> items) {
    Json j;
    j.type_ = Type::Array;
    j.items_ = std::move(items);
    return j;
}

Json Json::object() {
    Json j;
    j.type_ = Type::Object;
    return j;
}

double Json::num(double fallback) const {
    if (type_ != Type::Number && type_ != Type::String) {
        return fallback;
    }
    double v = 0;
    const char* b = text_.data();
    const char* e = b + text_.size();
    const auto r = std::from_chars(b, e, v);
    if (r.ec != std::errc() || r.ptr != e) {
        return fallback;
    }
    return v;
}

std::string Json::str() const {
    if (type_ == Type::String || type_ == Type::Number) {
        return text_;
    }
    return {};
}

std::size_t Json::size() const { return type_ == Type::Array ? items_.size() : type_ == Type::Object ? members_.size() : 0; }

const Json& Json::at(std::size_t i) const {
    static const Json none;
    return type_ == Type::Array && i < items_.size() ? items_[i] : none;
}

Json& Json::at(std::size_t i) { return items_.at(i); }

void Json::push(Json v) {
    if (type_ != Type::Array) {
        *this = array();
    }
    items_.push_back(std::move(v));
}

bool Json::has(std::string_view key) const {
    if (type_ != Type::Object) {
        return false;
    }
    for (const auto& [k, v] : members_) {
        if (k == key) {
            return true;
        }
    }
    return false;
}

const Json& Json::get(std::string_view key) const {
    static const Json none;
    if (type_ == Type::Object) {
        for (const auto& [k, v] : members_) {
            if (k == key) {
                return v;
            }
        }
    }
    return none;
}

Json& Json::operator[](std::string_view key) {
    if (type_ != Type::Object) {
        *this = object();
    }
    for (auto& [k, v] : members_) {
        if (k == key) {
            return v;
        }
    }
    members_.emplace_back(std::string(key), Json());
    return members_.back().second;
}

void Json::erase(std::string_view key) {
    std::erase_if(members_, [&](const Member& m) { return m.first == key; });
}

bool operator==(const Json& a, const Json& b) {
    if (a.type_ != b.type_) {
        return false;
    }
    switch (a.type_) {
    case Json::Type::Null: return true;
    case Json::Type::Bool: return a.bool_ == b.bool_;
    case Json::Type::Number: return a.num() == b.num();
    case Json::Type::String: return a.text_ == b.text_;
    case Json::Type::Array: return a.items_ == b.items_;
    case Json::Type::Object: return a.members_ == b.members_;
    }
    return false;
}

namespace {

void escapeTo(std::string& out, const std::string& s) {
    out += '"';
    for (const char ch : s) {
        const auto c = static_cast<unsigned char>(ch);
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        default:
            if (c < 0x20) {
                out += std::format("\\u{:04x}", c);
            } else {
                out += ch;
            }
        }
    }
    out += '"';
}

} // namespace

void Json::dumpTo(std::string& out, int indent, int depth) const {
    const auto nl = [&](int d) {
        out += '\n';
        out.append(static_cast<std::size_t>(indent * d), ' ');
    };
    switch (type_) {
    case Type::Null: out += "null"; break;
    case Type::Bool: out += bool_ ? "true" : "false"; break;
    case Type::Number: out += text_; break;
    case Type::String: escapeTo(out, text_); break;
    case Type::Array:
        if (items_.empty()) {
            out += "[]";
            break;
        }
        out += '[';
        for (std::size_t i = 0; i < items_.size(); i++) {
            if (i > 0) {
                out += ',';
            }
            nl(depth + 1);
            items_[i].dumpTo(out, indent, depth + 1);
        }
        nl(depth);
        out += ']';
        break;
    case Type::Object:
        if (members_.empty()) {
            out += "{}";
            break;
        }
        out += '{';
        for (std::size_t i = 0; i < members_.size(); i++) {
            if (i > 0) {
                out += ',';
            }
            nl(depth + 1);
            escapeTo(out, members_[i].first);
            out += " : ";
            members_[i].second.dumpTo(out, indent, depth + 1);
        }
        nl(depth);
        out += '}';
        break;
    }
}

std::string Json::dump(int indent) const {
    std::string out;
    dumpTo(out, indent, 0);
    return out;
}

namespace {

class Parser {
public:
    explicit Parser(std::string_view s) : s_(s) {}

    std::expected<Json, std::string> document() {
        auto v = value(0);
        if (!v) {
            return v;
        }
        ws();
        if (i_ != s_.size()) {
            return fail("text after the end");
        }
        return v;
    }

private:
    std::unexpected<std::string> fail(std::string_view what) const {
        return std::unexpected(std::format("JSON: {} at byte {}", what, i_));
    }
    void ws() {
        while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\n' || s_[i_] == '\r' || s_[i_] == '\t')) {
            i_++;
        }
    }
    bool lit(std::string_view w) {
        if (s_.substr(i_, w.size()) == w) {
            i_ += w.size();
            return true;
        }
        return false;
    }

    std::expected<Json, std::string> value(int depth) {
        if (depth > 64) {
            return fail("nested too deep");
        }
        ws();
        if (i_ >= s_.size()) {
            return fail("unexpected end");
        }
        const char c = s_[i_];
        if (c == '{') {
            return object(depth);
        }
        if (c == '[') {
            return array(depth);
        }
        if (c == '"') {
            auto s = string();
            if (!s) {
                return std::unexpected(s.error());
            }
            return Json::string(std::move(*s));
        }
        if (lit("true")) {
            return Json::boolean(true);
        }
        if (lit("false")) {
            return Json::boolean(false);
        }
        if (lit("null")) {
            return Json::null();
        }
        return number();
    }

    std::expected<Json, std::string> number() {
        const std::size_t b = i_;
        if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) {
            i_++;
        }
        while (i_ < s_.size() && ((s_[i_] >= '0' && s_[i_] <= '9') || s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E' ||
                                  s_[i_] == '-' || s_[i_] == '+')) {
            i_++;
        }
        std::string t(s_.substr(b, i_ - b));
        double v = 0;
        const auto r = std::from_chars(t.data(), t.data() + t.size(), v);
        if (t.empty() || r.ec != std::errc() || r.ptr != t.data() + t.size()) {
            i_ = b;
            return fail("bad value");
        }
        return Json::numberText(std::move(t));
    }

    std::expected<std::string, std::string> string() {
        i_++; // "
        std::string out;
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '"') {
                return out;
            }
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i_ >= s_.size()) {
                break;
            }
            const char e = s_[i_++];
            switch (e) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                if (i_ + 4 > s_.size()) {
                    return fail("bad escape");
                }
                unsigned cp = 0;
                const auto r = std::from_chars(s_.data() + i_, s_.data() + i_ + 4, cp, 16);
                if (r.ec != std::errc()) {
                    return fail("bad escape");
                }
                i_ += 4;
                // UTF-8 (surrogate pairs are kept as two code points; SteamVR's files are ASCII)
                if (cp < 0x80) {
                    out += static_cast<char>(cp);
                } else if (cp < 0x800) {
                    out += static_cast<char>(0xC0 | (cp >> 6));
                    out += static_cast<char>(0x80 | (cp & 0x3F));
                } else {
                    out += static_cast<char>(0xE0 | (cp >> 12));
                    out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
                    out += static_cast<char>(0x80 | (cp & 0x3F));
                }
                break;
            }
            default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }

    std::expected<Json, std::string> array(int depth) {
        i_++;
        Json a = Json::array();
        ws();
        if (i_ < s_.size() && s_[i_] == ']') {
            i_++;
            return a;
        }
        while (true) {
            auto v = value(depth + 1);
            if (!v) {
                return v;
            }
            a.push(std::move(*v));
            ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                i_++;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == ']') {
                i_++;
                return a;
            }
            return fail("expected , or ]");
        }
    }

    std::expected<Json, std::string> object(int depth) {
        i_++;
        Json o = Json::object();
        ws();
        if (i_ < s_.size() && s_[i_] == '}') {
            i_++;
            return o;
        }
        while (true) {
            ws();
            if (i_ >= s_.size() || s_[i_] != '"') {
                return fail("expected a key");
            }
            auto k = string();
            if (!k) {
                return std::unexpected(k.error());
            }
            ws();
            if (i_ >= s_.size() || s_[i_] != ':') {
                return fail("expected :");
            }
            i_++;
            auto v = value(depth + 1);
            if (!v) {
                return v;
            }
            o.members().emplace_back(std::move(*k), std::move(*v));
            ws();
            if (i_ < s_.size() && s_[i_] == ',') {
                i_++;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == '}') {
                i_++;
                return o;
            }
            return fail("expected , or }");
        }
    }

    std::string_view s_;
    std::size_t i_ = 0;
};

} // namespace

std::expected<Json, std::string> parseJson(std::string_view text) { return Parser(text).document(); }

std::string formatNumber(double v) {
    if (!std::isfinite(v)) {
        return "0";
    }
    if (v == 0) {
        return "0";
    }
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof buf, v); // shortest round-trip form
    return std::string(buf, r.ptr);
}

} // namespace pibridge::room
