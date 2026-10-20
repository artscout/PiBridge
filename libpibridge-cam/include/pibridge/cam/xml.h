// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/xml.rs (MIT OR Apache-2.0).
//
// Minimal XML element/attribute reader, enough for the factory calibration document (<?xml?> prolog, comments,
// nested elements, quoted attributes, the five predefined entities and numeric character references). Text content
// is ignored. Error messages and byte offsets are the Rust ones (the C ABI reports them).
#pragma once

#include <cstddef>
#include <expected>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pibridge::cam::xml {

/// One element with its attributes and child elements.
struct Element {
    std::string name;
    /// Attributes in document order.
    std::vector<std::pair<std::string, std::string>> attrs;
    /// Child elements in document order.
    std::vector<Element> children;

    /// Attribute value (nullptr when absent).
    const std::string* attr(std::string_view name) const;
    /// First child with this tag name (nullptr when absent).
    const Element* child(std::string_view name) const;
    /// All children with this tag name.
    std::vector<const Element*> children_named(std::string_view name) const;
};

/// Parse error with byte offset.
struct XmlError {
    std::size_t at = 0;
    std::string msg;
    /// "xml: {msg} at byte {at}".
    std::string text() const;
};

/// Parse a document and return its root element.
std::expected<Element, XmlError> parse(std::string_view doc);

} // namespace pibridge::cam::xml
