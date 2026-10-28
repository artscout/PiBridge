// SPDX-License-Identifier: MIT
// The VR dashboard drawn with QPainter into a BGRA picture (dash_view.h's layout; xrkit's panel pass takes the bytes as
// they are). Everything is drawn in code: rounded cards, text, chips; no picture files. CPU only (an offscreen
// QGuiApplication for the fonts, as tools/roomsetup's panelpaint); repainted only when the view changes.
#pragma once

#include "dash_view.h"

#include <cstdint>
#include <vector>

namespace pibridge::home::dash {

class Painter {
public:
    Painter();
    ~Painter();
    Painter(const Painter&) = delete;
    Painter& operator=(const Painter&) = delete;
    /// Paint `v` into a kW x kH BGRA picture; the bytes stay valid until the next paint (rows `stride` bytes apart).
    const std::uint8_t* paint(const View& v, int& stride);
    /// The same as a tightly packed copy (kW * kH * 4 bytes): the self test's pictures.
    std::vector<std::uint8_t> picture(const View& v);

private:
    struct Impl;
    Impl* d_;
};

} // namespace pibridge::home::dash
