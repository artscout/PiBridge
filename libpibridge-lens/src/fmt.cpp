// SPDX-License-Identifier: MIT
// Rust's `{}` of an f32 (see fmt.h).
#include "pibridge/lens/fmt.h"

#include <charconv>
#include <cstdlib>
#include <string_view>

namespace pibridge::lens::fmt {

std::string display_f32(float x) {
    if (std::isnan(x)) {
        return "NaN";
    }
    if (std::isinf(x)) {
        return x < 0 ? "-inf" : "inf";
    }
    const std::string sign = std::signbit(x) ? "-" : "";
    if (x == 0.0f) {
        return sign + "0";
    }
    // shortest digits that round-trip as f32: d1.d2d3...e<exp>
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof buf, std::fabs(x), std::chars_format::scientific);
    const std::string_view s(buf, static_cast<std::size_t>(r.ptr - buf));
    const std::size_t e = s.find('e');
    std::string d;
    for (std::size_t i = 0; i < e; i++) {
        if (s[i] != '.') {
            d.push_back(s[i]);
        }
    }
    const int exp10 = std::atoi(std::string(s.substr(e + 1)).c_str());
    const int n = static_cast<int>(d.size());
    // value = 0.d * 10^(exp10 + 1): the decimal point goes after `point` digits
    const int point = exp10 + 1;
    std::string out;
    if (point <= 0) {
        out = "0." + std::string(static_cast<std::size_t>(-point), '0') + d;
    } else if (point >= n) {
        out = d + std::string(static_cast<std::size_t>(point - n), '0');
    } else {
        out = d.substr(0, static_cast<std::size_t>(point)) + "." + d.substr(static_cast<std::size_t>(point));
    }
    return sign + out;
}

} // namespace pibridge::lens::fmt
