// SPDX-License-Identifier: MIT
// panelpaint: the room-setup panel, painted with QPainter into a BGRA image for the VR panel texture (xrkit TexPass).
//
// Everything is drawn procedurally (rounded card, header, text, rounded chips, the hex "scanning" accent) so the panel
// needs NO external assets and works out of the box. Pimax's own PVRGuardian images are an OPTIONAL skin layered on
// top when they have been copied into the kit (setAssetDir) -- never required, never in the repo.
//
// The paint is CPU-only (an offscreen QGuiApplication for the font database); the VR side only uploads the pixels.
// A sample can be dumped to a PNG without a headset (pibridge-panelpaint), which is how the look is tuned.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pibridge::roomsetup::panel {

/// A button chip. The rect is in panel UV: x,y the top-left, w,h the size, all 0..1 with y DOWN (image space).
struct Button {
    std::string label;
    float x = 0, y = 0, w = 0, h = 0;
    bool hover = false;   // the pointer is on it
    bool primary = false; // the main action (filled accent), e.g. NEXT / SAVE / FINISH
};

/// What to draw on the panel this frame.
struct Content {
    std::string title;              // the header strip
    std::vector<std::string> lines; // body text, one per line, centred
    std::vector<Button> buttons;    // a row of chips near the bottom
    bool scanning = false;          // the look-around / mapping step: a green hex accent
};

/// Paints Content into a BGRA8888 image (QImage::Format_ARGB32 = B,G,R,A bytes on little-endian, what TexPass wants).
class Painter {
public:
    Painter();
    ~Painter();
    Painter(const Painter&) = delete;
    Painter& operator=(const Painter&) = delete;

    /// Optional Pimax skin: a folder of PVRGuardian PNGs (btn_normal.png, ...). Empty = procedural only.
    void setAssetDir(const std::string& dir);

    /// Paint into a `pxW` x `pxH` BGRA image; returns the pixels (valid until the next paint) and the row stride.
    const std::uint8_t* paint(const Content& c, int pxW, int pxH, int& strideBytes);

    /// Paint and write a PNG (for the headless look check). true on success.
    bool savePng(const Content& c, int pxW, int pxH, const std::string& path);

private:
    struct Impl;
    Impl* d_;
};

} // namespace pibridge::roomsetup::panel
