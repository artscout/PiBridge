// SPDX-License-Identifier: MIT
// A small JSON value for SteamVR's files (chaperone_info.vrchap, steamvr.vrsettings) and our own rooms. It keeps what
// we do not touch as it was: object keys stay in file order, and a number keeps its text until it is changed, so a
// round trip only changes the values the editor writes. No exceptions: parse() returns an error text.
#pragma once

#include <expected>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::room {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };
    using Member = std::pair<std::string, Json>;

    Json() = default;
    static Json null() { return {}; }
    static Json boolean(bool b);
    static Json number(double v);
    /// A number written as `text` (no reformatting).
    static Json numberText(std::string text);
    static Json string(std::string s);
    static Json array(std::vector<Json> items = {});
    static Json object();

    Type type() const { return type_; }
    bool isNull() const { return type_ == Type::Null; }
    bool isNumber() const { return type_ == Type::Number; }
    bool isString() const { return type_ == Type::String; }
    bool isArray() const { return type_ == Type::Array; }
    bool isObject() const { return type_ == Type::Object; }
    bool isBool() const { return type_ == Type::Bool; }

    /// Numbers as double (0 for anything else); strings that hold a number are read too.
    double num(double fallback = 0) const;
    bool boolean() const { return type_ == Type::Bool && bool_; }
    /// The string value, or the number's text, or "" (universeID is a string in SteamVR's files, a number elsewhere).
    std::string str() const;
    const std::string& numberTextRef() const { return text_; }

    // arrays
    std::size_t size() const;
    const Json& at(std::size_t i) const;
    Json& at(std::size_t i);
    void push(Json v);
    std::vector<Json>& items() { return items_; }
    const std::vector<Json>& items() const { return items_; }

    // objects (keys in insertion order)
    bool has(std::string_view key) const;
    /// The member, or a shared null when missing.
    const Json& get(std::string_view key) const;
    /// The member, created (null) when missing.
    Json& operator[](std::string_view key);
    void erase(std::string_view key);
    std::vector<Member>& members() { return members_; }
    const std::vector<Member>& members() const { return members_; }

    /// Indented text (3 spaces like SteamVR's own files), ending without a newline.
    std::string dump(int indent = 3) const;
    friend bool operator==(const Json& a, const Json& b);

private:
    void dumpTo(std::string& out, int indent, int depth) const;

    Type type_ = Type::Null;
    bool bool_ = false;
    std::string text_; // number text or string value
    std::vector<Json> items_;
    std::vector<Member> members_;
};

std::expected<Json, std::string> parseJson(std::string_view text);
/// Shortest text that reads back as the same double ("0.5", "2.43", "1e-07").
std::string formatNumber(double v);

} // namespace pibridge::room
