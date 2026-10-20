// SPDX-License-Identifier: MIT
// Passthrough for PiBridge's OpenXR tools (pibridge-roomsetup --map and pibridge-home): the headset's four cameras (factory ids
// 0..3, 640x480 grey fisheye, KB4: the lower "tracking" pair looking 25-30 deg down and the upper side
// "Pi-controller" pair looking about 21 deg up) drawn as the background of each eye, so the room is seen around you,
// ceiling included.
//
// Per eye pixel (one fragment shader pass, tools/xrkit/shaders/passthrough.frag, same arithmetic as sample() here):
//   the eye ray at the display time (the eye pose of xrLocateViews) -> a point in the stage: the first of the saved
//   room's walls and ceiling and the floor plane it meets (the room prism, when a room is known), else the floor
//   within 8 m, else a sphere of `radius` around the eye -> that point in each camera at the frame's capture time (the
//   head pose of the frame's own timestamp, so the head's motion between capture and display is taken out: the time
//   warp) -> KB4 -> the camera image.
// Camera choice per pixel (omarchyvr's omarchyvr-passthrough lut.rs, ported): the eye's preference order (the lower
// camera on the eye's side, the other lower one, then the upper camera on the eye's side and the other upper one).
// A camera's weight is 1 inside its usable image and falls to 0 over `feather` px towards the tile border or the
// vignetting circle; sample A is the first camera with weight > 0, and where its weight wa < 1 the other camera with
// the largest weight wb is blended in with alpha = wa / (wa + (1 - wa) wb) (continuous at every edge: no hard seam).
// Each camera has its own gain (balanceGains: the tiles' mean grey brought together) so the seams do not show as a
// step in brightness.
// The GPU part (one 4-layer image, upload, pipeline) is xrkit's PassthroughGpu; this file has no OpenXR or Vulkan:
// the calibration, the uniform block, the CPU reference (tests and --snapshot) and the camera feed thread.
#pragma once

#include "pibridge/cam/calib.h"
#include "pibridge/cam/kb4.h"
#include "pibridge/room/geom.h"
#include "pibridge/room/raster.h"
#include "pibridge/room/vmath.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace pibridge::xrkit::pt {

inline constexpr int kW = 640;
inline constexpr int kH = 480;
inline constexpr std::size_t kCams = 4;   // at most; the rig may have only the lower pair
inline constexpr std::size_t kWalls = 16; // wall segments of the room prism in the uniform block

/// One passthrough camera.
struct Camera {
    std::size_t tile = 0;     // factory id = tile of the camera frame
    std::string name;         // trackingA, trackingB, Pi-controllerA, Pi-controllerB
    cam::Kb4 kb;
    room::Mat34 viewFromCam;  // OpenCV camera axes (x right, y down, z forward) -> OpenXR view axes, metres
    bool upper = false;       // looks up (the controller cameras)
};

/// The cameras in slots 0..n-1 (the frame's images and the GPU layers use the same slots) and the order each eye
/// tries them in.
struct Rig {
    std::array<Camera, kCams> cam{};
    std::size_t n = 0;
    std::array<std::array<std::uint8_t, kCams>, 2> order{}; // per eye (0 left, 1 right): slots, best first
    std::vector<std::size_t> tiles() const;
};

/// Every 640x480 camera of the calibration that has its body pose (ids 0..3), as a rig. `headInBody`: the view
/// origin in the factory body (IMU) frame, whose axes are the OpenXR head axes (calib.h); default_head_in_imu is the
/// usual estimate. `upper` false: the lower pair only (the first version's choice). Fails without the lower pair.
std::expected<Rig, std::string> makeRig(const cam::DeviceCalib& cal, room::Vec3 headInBody, bool upper = true);
/// The preference order of an eye: lower cameras first, the eye's side first (by the camera's x in the view).
std::array<std::uint8_t, kCams> preference(const Rig& r, int eye);

/// The newest frame of the rig: img[slot] (640x480 each, row 0 at the top).
struct Frame {
    std::array<std::vector<std::uint8_t>, kCams> img;
    std::array<float, kCams> means{};
    std::uint64_t monoNs = 0; // CLOCK_MONOTONIC of the exposure (the device time mapped), else of the arrival
    std::uint64_t seq = 0;    // +1 per frame of the feed
    float mean = 0;           // mean grey (subsampled) of the lower pair (the auto gain follows it)
    std::size_t n = 0;        // images filled
};

/// The per-eye uniform block, std140 (the shader's `Eye`): 848 bytes of floats.
struct Uniform {
    std::array<float, 16> stageFromEye{};                   // column major
    std::array<float, 4> tan{};                             // tan of the fov: left, right, up, down
    std::array<float, 4> misc{};                            // floor y (stage), radius (m), gain, 1 = sRGB swapchain
    std::array<float, 4> info{};                            // cameras, feather px, vignette radius px, max distance m
    std::array<float, 4> order{};                           // the eye's camera slots, best first
    std::array<float, 4> roomInfo{};                        // walls, ceiling y, 1 = room prism on, border px
    std::array<std::array<float, 4>, kWalls> walls{};       // x0 z0 x1 z1 (stage) per wall segment
    std::array<std::array<float, 16>, kCams> camFromStage{}; // column major, per slot, at the frame's capture time
    std::array<std::array<float, 4>, kCams> kbF{};          // fx fy cx cy
    std::array<std::array<float, 4>, kCams> kbK{};          // k1..k4
    std::array<std::array<float, 4>, kCams> kbL{};          // theta limit, valid (1/0), the camera's gain, 0
};
static_assert(sizeof(Uniform) == 848, "std140 layout of the passthrough uniform block");

struct EyeView {
    room::Mat34 stageFromEye;
    double tanL = -1, tanR = 1, tanU = 1, tanD = -1;
};
/// From OpenXR's fov angles (radians, left and down negative).
EyeView eyeView(const room::Mat34& stageFromEye, double angleLeft, double angleRight, double angleUp, double angleDown);

inline constexpr double kRadius = 2.5;     // the distance of everything that is not floor (without a room)
inline constexpr double kMaxFloor = 8.0;   // a hit further than this counts as this distance
inline constexpr double kFeatherPx = 24.0; // the blend band at a camera's edge
inline constexpr double kVignettePx = 330.0;
inline constexpr double kBorderPx = 2.0;

/// The saved room as the reprojection surface: its floor outline (stage x, z) extruded to the ceiling.
struct RoomShape {
    room::Poly outline; // at least 3 points; more than kWalls are thinned
    double ceilingY = 2.43;
};

struct Look {
    double floorY = 0;      // the floor in the stage
    double radius = kRadius;
    double gain = 1;
    bool srgb = false;
    std::optional<RoomShape> room;
    std::array<double, kCams> camGain{1, 1, 1, 1}; // per slot (balanceGains)
    double feather = kFeatherPx;
    double vignette = kVignettePx;
};
/// The block for `eye` (0 left, 1 right): `stageFromViewAtCapture` is the head (view) pose at the frame's timestamp.
Uniform makeUniform(int eye, const EyeView& e, const Rig& rig, const room::Mat34& stageFromViewAtCapture, const Look& look);

/// Which camera(s) a pixel came from (coverage pictures and tests).
struct Sample {
    float grey = 0;   // 0..1, after the gains, before sRGB
    int a = -1;       // slot of sample A (-1: none)
    int b = -1;       // slot of sample B (-1: none)
    float alpha = 1;  // weight of A
};
/// The shader's arithmetic on the CPU at (u, v) (0..1, v down) of the eye image; nullopt where no camera sees.
std::optional<Sample> sampleFull(const Uniform& u, const Frame& f, double uu, double vv);
/// The grey only.
std::optional<float> sample(const Uniform& u, const Frame& f, double uu, double vv);
/// The distance along the eye ray the shader uses (the room prism, the floor, else the radius); for tests.
double surfaceDistance(const Uniform& u, room::Vec3 origin, room::Vec3 dir);
/// The eye image (grey as RGB; dark grey where no camera sees), the background of --snapshot.
room::Image render(const Uniform& u, const Frame& f, int w, int h);
/// Which camera feeds each pixel: slot 0 red, 1 blue, 2 yellow, 3 green, blended where they cross-fade; dark: none.
room::Image coverage(const Uniform& u, const Frame& f, int w, int h);
/// A gain that lifts a dark room (mean grey) towards mid grey: 1..4.
double autoGain(double mean);
/// Per slot: the lower pair's mean over the camera's own mean, 0.6..1.6 (1 for a camera without a picture).
std::array<double, kCams> balanceGains(const Frame& f);
/// Mean grey of an image, every 8th pixel of every 8th row.
float meanOf(const std::vector<std::uint8_t>& img);

/// The camera feed: a thread reading the camera broker ("broker") or a .pxcam, keeping the newest tracking frame's
/// tiles of the rig; reopens every 2 s while the source is not there. The render thread only takes a finished frame
/// (a swap under a mutex): it never waits for the camera.
class Feed {
public:
    Feed(std::string spec, std::vector<std::size_t> tiles, bool loopReplay = true);
    ~Feed();
    Feed(const Feed&) = delete;
    Feed& operator=(const Feed&) = delete;
    void start();
    void stop();
    /// A frame newer than `afterSeq` into `out` (its buffers are swapped with the feed's); false when none.
    bool take(Frame& out, std::uint64_t afterSeq);
    /// The newest seq (0: none yet).
    std::uint64_t seq() const { return seq_.load(); }
    /// Plain words when the source cannot be opened (empty while frames come).
    std::string error() const;

private:
    void run();
    std::string spec_;
    std::vector<std::size_t> tiles_;
    bool loop_;
    std::atomic<bool> stop_{false};
    std::atomic<std::uint64_t> seq_{0};
    mutable std::mutex m_;
    Frame back_;
    bool fresh_ = false;
    std::string error_;
    std::thread th_;
};

} // namespace pibridge::xrkit::pt
