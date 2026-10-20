// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/xml.rs (MIT OR Apache-2.0).
#include "pibridge/cam/xml.h"

#include "pibridge/cam/rustfmt.h"

#include <format>
#include <optional>

namespace pibridge::cam::xml {

const std::string* Element::attr(std::string_view n) const {
    for (const auto& [k, v] : attrs) {
        if (k == n) {
            return &v;
        }
    }
    return nullptr;
}

const Element* Element::child(std::string_view n) const {
    for (const auto& c : children) {
        if (c.name == n) {
            return &c;
        }
    }
    return nullptr;
}

std::vector<const Element*> Element::children_named(std::string_view n) const {
    std::vector<const Element*> v;
    for (const auto& c : children) {
        if (c.name == n) {
            v.push_back(&c);
        }
    }
    return v;
}

std::string XmlError::text() const {
    return std::format("xml: {} at byte {}", msg, at);
}

namespace {

using R = std::expected<void, XmlError>;

std::unexpected<XmlError> err(std::size_t at, std::string msg) {
    return std::unexpected(XmlError{at, std::move(msg)});
}

bool is_name_byte(unsigned char b) {
    return (b >= '0' && b <= '9') || (b >= 'a' && b <= 'z') || (b >= 'A' && b <= 'Z') || b == '_' || b == '-' ||
           b == '.' || b == ':' || b >= 0x80;
}

// Rust u8::is_ascii_whitespace: space, \t, \n, \x0C, \r (not \x0B).
bool is_ws(unsigned char b) {
    return b == ' ' || b == '\t' || b == '\n' || b == 0x0c || b == '\r';
}

void push_utf8(std::string& o, std::uint32_t cp) {
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

// u32::from_str_radix(s, 16): optional '+', at least one hex digit, no overflow.
bool parse_hex_u32(std::string_view s, std::uint32_t& out) {
    if (!s.empty() && s[0] == '+') {
        s.remove_prefix(1);
    }
    if (s.empty()) {
        return false;
    }
    std::uint64_t v = 0;
    for (const char c : s) {
        int d = 0;
        if (c >= '0' && c <= '9') {
            d = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            d = c - 'a' + 10;
        } else if (c >= 'A' && c <= 'F') {
            d = c - 'A' + 10;
        } else {
            return false;
        }
        v = v * 16 + static_cast<std::uint64_t>(d);
        if (v > 0xffffffffull) {
            return false;
        }
    }
    out = static_cast<std::uint32_t>(v);
    return true;
}

// char::from_u32
bool valid_char(std::uint32_t cp) {
    return cp <= 0x10ffff && !(cp >= 0xd800 && cp <= 0xdfff);
}

std::optional<std::string> decode_entities(const std::string& s) {
    if (s.find('&') == std::string::npos) {
        return s;
    }
    std::string out;
    out.reserve(s.size());
    std::string_view rest = s;
    for (;;) {
        const std::size_t p = rest.find('&');
        if (p == std::string_view::npos) {
            break;
        }
        out.append(rest.substr(0, p));
        const std::string_view after = rest.substr(p + 1);
        const std::size_t end = after.find(';');
        if (end == std::string_view::npos) {
            return std::nullopt;
        }
        const std::string_view ent = after.substr(0, end);
        if (ent == "lt") {
            out += '<';
        } else if (ent == "gt") {
            out += '>';
        } else if (ent == "amp") {
            out += '&';
        } else if (ent == "quot") {
            out += '"';
        } else if (ent == "apos") {
            out += '\'';
        } else if (ent.starts_with("#x") || ent.starts_with("#X")) {
            std::uint32_t cp = 0;
            if (!parse_hex_u32(ent.substr(2), cp) || !valid_char(cp)) {
                return std::nullopt;
            }
            push_utf8(out, cp);
        } else if (ent.starts_with("#")) {
            std::uint32_t cp = 0;
            if (!rfmt::parse_u32(ent.substr(1), cp) || !valid_char(cp)) {
                return std::nullopt;
            }
            push_utf8(out, cp);
        } else {
            return std::nullopt;
        }
        rest = after.substr(end + 1);
    }
    out.append(rest);
    return out;
}

struct P {
    std::string_view s;
    std::size_t i = 0;

    unsigned char at(std::size_t k) const { return static_cast<unsigned char>(s[k]); }
    bool has(std::size_t k) const { return k < s.size(); }

    bool starts(std::string_view pat) const { return s.substr(i).starts_with(pat); }

    void skip_ws() {
        while (i < s.size() && is_ws(at(i))) {
            i++;
        }
    }

    R skip_until(std::string_view pat) {
        const std::size_t start = i;
        const std::size_t p = s.find(pat, i);
        if (p == std::string_view::npos) {
            return err(start, std::format("unterminated construct, expected {}", rfmt::debug_str(pat)));
        }
        i = p + pat.size();
        return {};
    }

    // Skip text, comments, processing instructions and DOCTYPE until the next element tag (<x or </).
    R skip_misc() {
        for (;;) {
            while (i < s.size() && s[i] != '<') {
                i++;
            }
            if (i >= s.size()) {
                return {};
            }
            R r;
            if (starts("<!--")) {
                r = skip_until("-->");
            } else if (starts("<?")) {
                r = skip_until("?>");
            } else if (starts("<![CDATA[")) {
                r = skip_until("]]>");
            } else if (starts("<!")) {
                r = skip_until(">");
            } else {
                return {};
            }
            if (!r) {
                return r;
            }
        }
    }

    std::expected<std::string, XmlError> name() {
        const std::size_t start = i;
        while (i < s.size() && is_name_byte(at(i))) {
            i++;
        }
        if (i == start) {
            return err(start, "expected a name");
        }
        return rfmt::utf8_lossy(s.substr(start, i - start));
    }

    std::expected<std::string, XmlError> attr_value() {
        if (!has(i) || (s[i] != '"' && s[i] != '\'')) {
            return err(i, "expected a quoted attribute value");
        }
        const char q = s[i];
        i++;
        const std::size_t start = i;
        while (i < s.size() && s[i] != q) {
            i++;
        }
        if (i >= s.size()) {
            return err(start, "unterminated attribute value");
        }
        const std::string raw = rfmt::utf8_lossy(s.substr(start, i - start));
        i++;
        auto d = decode_entities(raw);
        if (!d) {
            return err(start, "bad entity");
        }
        return std::move(*d);
    }

    std::expected<Element, XmlError> element(std::size_t depth) {
        if (depth > 64) {
            return err(i, "nesting too deep");
        }
        if (!has(i) || s[i] != '<') {
            return err(i, "expected '<'");
        }
        i++;
        auto nm = name();
        if (!nm) {
            return std::unexpected(nm.error());
        }
        Element el;
        el.name = std::move(*nm);
        for (;;) {
            skip_ws();
            if (!has(i)) {
                return err(i, "unexpected end inside a tag");
            }
            if (s[i] == '/') {
                if (!has(i + 1) || s[i + 1] != '>') {
                    return err(i, "expected '/>'");
                }
                i += 2;
                return el;
            }
            if (s[i] == '>') {
                i++;
                break;
            }
            auto k = name();
            if (!k) {
                return std::unexpected(k.error());
            }
            skip_ws();
            if (!has(i) || s[i] != '=') {
                return err(i, std::format("expected '=' after attribute {}", *k));
            }
            i++;
            skip_ws();
            auto v = attr_value();
            if (!v) {
                return std::unexpected(v.error());
            }
            el.attrs.emplace_back(std::move(*k), std::move(*v));
        }
        // content
        for (;;) {
            if (auto r = skip_misc(); !r) {
                return std::unexpected(r.error());
            }
            if (i >= s.size()) {
                return err(i, std::format("missing </{}>", el.name));
            }
            if (starts("</")) {
                i += 2;
                auto close = name();
                if (!close) {
                    return std::unexpected(close.error());
                }
                if (*close != el.name) {
                    return err(i, std::format("</{}> closes <{}>", *close, el.name));
                }
                skip_ws();
                if (!has(i) || s[i] != '>') {
                    return err(i, "expected '>'");
                }
                i++;
                return el;
            }
            auto c = element(depth + 1);
            if (!c) {
                return c;
            }
            el.children.push_back(std::move(*c));
        }
    }
};

} // namespace

std::expected<Element, XmlError> parse(std::string_view doc) {
    P p{doc, 0};
    if (auto r = p.skip_misc(); !r) {
        return std::unexpected(r.error());
    }
    if (p.i >= p.s.size()) {
        return err(p.i, "no root element");
    }
    auto root = p.element(0);
    if (!root) {
        return root;
    }
    if (auto r = p.skip_misc(); !r) {
        return std::unexpected(r.error());
    }
    if (p.i < p.s.size()) {
        return err(p.i, "content after the root element");
    }
    return root;
}

} // namespace pibridge::cam::xml
