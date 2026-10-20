// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/ae.rs (MIT OR Apache-2.0).
//
// Auto-exposure (AE) for the tracking cameras (6DoF phase 2; evidence and frame layout in omarchyvr
// `analysis/slam-exposure.md`, the Crystal Super in `analysis/crystal-super-ae.md`).
//
// On Windows the host pushes exposure and gain for the four cameras with HID command 0x51 on the headset, about 12.5
// times a second:
//
//   58 19 51 00 | 01 32 | e0 e1 e2 e3 | g0 g1 g2 g3 | led_e led_g      (u16 little endian)
//
// e/g are the tracking-exposure values of cameras 0..3 (the lower pair 0,1 and the upper pair 2,3 always carry equal
// values), led_e/led_g the short controller-LED exposure (constant 40/62). The camera frame metadata echoes the values
// in effect, so the loop can be closed on what each image was really taken with. On Linux nobody sends it and the
// cameras stay at the power-on default (Dream Air 10410/449, Crystal Super 3500/60).
//
// This module is pure logic: statistics of the tracking tiles -> a new setting per camera pair -> the command
// payload. Sending is the driver's job and is OFF unless PIMAX_CAM_AE=1. Every report still passes PiBridge's allowlist
// gate (`camera_ae` in devices/*.toml); for the Crystal Super this module itself never hands out a setting outside
// Windows' per-pair ranges (AeModel::allows, AeParams::pair_caps).
//
// Control law (per pair, both tiles pooled): masked mean of the unclipped 16x16 cells (law V2) or the trimmed mean
// (law V1, PIMAX_CAM_AE_LAW=old), error ln(target / m) with a deadband, damped multiplicative step r = exp(k * err)
// clamped to max_step, applied to the exposure*gain product of the analysed frame (from its metadata: no wind-up
// from the 1-3 frame latency). The product is split exposure first up to the soft cap, then gain, quantised to the
// values Windows sent.
#pragma once

#include "pibridge/cam/camera.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace pibridge::cam::ae {

/// HID command id of the exposure push.
inline constexpr std::uint8_t AE_CMD = 0x51;
/// Option byte of the exposure push.
inline constexpr std::uint8_t AE_OPT = 0x00;
/// Payload length (01 32 + 10 u16): 58 19 51 00 + 22 bytes, LL = 22 + 3 = 0x19.
inline constexpr std::size_t AE_PAYLOAD_LEN = 22;
/// The two constant leading payload bytes Windows always sends.
inline constexpr std::array<std::uint8_t, 2> AE_HEAD{0x01, 0x32};
/// Linux power-on default of the tracking exposure (no AE).
inline constexpr std::uint16_t POWER_ON_EXPOSURE = 10410;
/// Windows' exposure ladder (gain 13): geometric, ratio ~1.0302 per step, anchored at 1682.
inline constexpr double LADDER_ANCHOR = 1682.0;
/// Ladder ratio.
inline constexpr double LADDER_RATIO = 1.030'212;
/// Gain codes Windows used (both pairs), ascending.
inline constexpr std::array<std::uint16_t, 29> GAIN_CODES{13, 15, 17, 19, 21, 23, 26, 29, 32, 35, 39, 43, 46, 53, 59,
                                                          71, 78, 79, 87, 95, 105, 116, 128, 141, 155, 171, 188, 207, 215};
/// Windows' exposure values at gain 13, ascending (observed 1682..4108), extended below 1682 with the same ratio down
/// to Windows' observed minimum ~1140.
inline constexpr std::array<std::uint16_t, 44> EXPOSURE_LADDER{
    1142, 1177, 1212, 1249, 1287, 1326, 1366, 1407, 1449, 1493, 1538, 1585, 1633, 1682, 1733, 1785,
    1839, 1895, 1952, 2011, 2072, 2135, 2200, 2266, 2334, 2405, 2478, 2553, 2630, 2709, 2791, 2875,
    2962, 3051, 3143, 3238, 3336, 3437, 3541, 3648, 3758, 3871, 3988, 4108};
/// Linux power-on default of the tracking gain (no AE).
inline constexpr std::uint16_t POWER_ON_GAIN = 449;

/// Nearest ladder value (geometric extension outside the table).
double ladder(double e);
/// Largest ladder value <= e (geometric extension below the table).
double ladder_down(double e);
/// Nearest value of an ascending table (geometric extension with Windows' ratio outside it).
double ladder_in(std::span<const std::uint16_t> table, double e);
/// Largest value of an ascending table <= e (geometric extension below it).
double ladder_down_in(std::span<const std::uint16_t> table, double e);

// ---- Crystal Super: what Pimax Play sent on Windows (capture 2026-10-17, 16751 commands in 14 takes). Same frame,
// same 80 ms period, same pairs (0,1 / 2,3 always equal), same LED pair.

/// Crystal Super exposure ladder at the low gains: the Dream Air ladder from 2200 up to the soft cap 4108.
inline constexpr std::array<std::uint16_t, 22> CRYSTAL_EXPOSURE_LADDER{
    2200, 2266, 2334, 2405, 2478, 2553, 2630, 2709, 2791, 2875, 2962, 3051, 3143, 3238, 3336, 3437, 3541,
    3648, 3758, 3871, 3988, 4108};
/// Crystal Super gain codes below the top gain (215), as Windows sent them on the Crystal.
inline constexpr std::array<std::uint16_t, 28> CRYSTAL_GAIN_CODES{13, 17, 19, 21, 23, 29, 32, 35, 43, 46, 49, 53, 58, 65,
                                                                  71, 78, 79, 87, 95, 105, 116, 128, 141, 155, 171, 188,
                                                                  207, 215};
/// Crystal Super settings above the soft exposure cap, ascending exposure x gain (every pair was sent by Windows).
inline constexpr std::array<std::pair<std::uint16_t, std::uint16_t>, 41> CRYSTAL_HIGH{{
    {4194, 215}, {4320, 215}, {4450, 215}, {4584, 215}, {4722, 215}, {4864, 215}, {5051, 215}, {5161, 215},
    {5316, 215}, {5476, 215}, {5641, 215}, {5720, 220}, {5811, 226}, {5986, 237}, {6072, 244}, {6166, 249},
    {6351, 261}, {6440, 270}, {6542, 275}, {6632, 280}, {6739, 288}, {6942, 303}, {7042, 309}, {7151, 318},
    {7251, 326}, {7366, 334}, {7462, 334}, {7587, 351}, {7662, 358}, {7815, 368}, {7880, 370}, {7980, 375},
    {8050, 387}, {8292, 406}, {8342, 421}, {8541, 426}, {8741, 431}, {8941, 439}, {9541, 439}, {9841, 446},
    {10410, 449},
}};
/// Crystal Super firmware default of the tracking cameras before the first push (frame metadata, 2026-10-18).
inline constexpr std::pair<std::uint16_t, std::uint16_t> CRYSTAL_POWER_ON{3500, 60};

/// Windows' range of one camera pair: (exposure min, exposure max, gain min, gain max).
struct PairRange {
    std::uint16_t e_min, e_max, g_min, g_max;
    bool operator==(const PairRange&) const = default;
};
/// Crystal Super ranges per pair, exactly what Windows sent: lower pair (cameras 0, 1) 2200..10410 us, gain 13..449;
/// upper pair (2, 3) 2200..8941 us, gain 13..439. The same ranges are the allowlist of `camera_ae` in
/// devices/crystal-super.toml.
inline constexpr std::array<PairRange, 2> CRYSTAL_PAIR_RANGES{{{2200, 10410, 13, 449}, {2200, 8941, 13, 439}}};

/// Exposure and gain of the four cameras plus the LED-frame pair (raw units, as on the wire).
struct AeSetting {
    std::array<std::uint16_t, 4> exposure{};
    std::array<std::uint16_t, 4> gain{};
    std::uint16_t led_exposure = 0;
    std::uint16_t led_gain = 0;
    bool operator==(const AeSetting&) const = default;
    /// The setting a frame's metadata reports (LED pair: the given defaults, the metadata does not carry it).
    static AeSetting from_info(const FrameInfo& fi, std::uint16_t led_exposure, std::uint16_t led_gain);
};

/// The headset whose Windows auto-exposure the controller follows. Only the model constants differ.
enum class AeModel {
    /// Pimax Dream Air (34a4:0044). The default, unchanged.
    DreamAir,
    /// Pimax Crystal Super (34a4:0040).
    CrystalSuper,
};

/// Device module id ("dream-air", "crystal-super").
const char* model_id(AeModel m);
/// From a module id (dream-air, crystal-super; a few spellings).
std::optional<AeModel> model_parse(std::string_view s);
/// From the product id of the headset's HID chip.
std::optional<AeModel> model_by_pid(std::uint16_t pid);
/// Exposure ladder at the low gains.
std::span<const std::uint16_t> model_ladder(AeModel m);
/// Gain codes used with the soft exposure cap.
std::span<const std::uint16_t> model_gain_codes(AeModel m);
/// (exposure, gain) settings above the soft cap at the top gain code (empty: the Dream Air's split).
std::span<const std::pair<std::uint16_t, std::uint16_t>> model_high(AeModel m);
/// The cameras' setting before the first push.
std::pair<std::uint16_t, std::uint16_t> model_power_on(AeModel m);
/// Windows' range of each pair, enforced on every setting (nullopt: not enforced, the Dream Air as before).
std::optional<std::array<PairRange, 2>> model_pair_ranges(AeModel m);
/// The setting is inside Windows' ranges of this model (always true without enforced ranges).
bool model_allows(AeModel m, const AeSetting& s);
/// The model of the headset on USB (PIMAX_CAM_AE_MODEL first, then the HID chip in sysfs; Dream Air when none or
/// both are found) and where that came from.
std::pair<AeModel, std::string> model_detect();

/// Product ids of the vendor 34a4 devices in a sysfs USB directory.
std::vector<std::uint16_t> usb_pids_sysfs(const std::string& dir);
/// The model from the Pimax product ids on USB: the Crystal Super only when its HID chip (0040) is there and the
/// Dream Air's (0044) is not; everything else keeps the Dream Air.
std::pair<AeModel, const char*> model_from_pids(std::span<const std::uint16_t> pids);

/// Command payload (22 bytes after 58 19 51 00).
std::array<std::uint8_t, AE_PAYLOAD_LEN> payload(const AeSetting& s);
/// Parse a payload (or a whole 58 19 51 00 .. report). nullopt if it is not an exposure push.
std::optional<AeSetting> parse_payload(std::span<const std::uint8_t> b);

/// Brightness statistics of one tile (or a pooled pair).
struct TileStats {
    /// Mean of all sampled pixels.
    double mean = 0.0;
    /// Mean of the pixels at or below the trim percentile (the AE input; law V2: the masked mean of the unclipped
    /// cells).
    double trimmed_mean = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    /// Fraction of samples at or above the saturation level.
    double sat_frac = 0.0;
    /// Fraction of samples at or below 8 (black).
    double dark_frac = 0.0;
    /// Samples used.
    std::uint32_t n = 0;
    bool operator==(const TileStats&) const = default;
};

/// 256-bin histogram.
struct Histogram {
    std::array<std::uint32_t, 256> bins{};
    std::uint64_t total() const;
    void merge(const Histogram& o);
    /// Value at quantile q (0..1).
    double quantile(double q) const;
    /// Statistics; trim = percentile (0..1) above which pixels are left out of the trimmed mean.
    TileStats stats(double trim, std::uint8_t sat_level) const;
};

/// Sampling mask: pixels inside an ellipse centred on the tile (the fisheye image circle).
struct Mask {
    double cx = 319.5;
    double cy = 239.5;
    double rx = 320.0 * 0.95;
    double ry = 240.0 * 0.97;
    bool inside(double x, double y) const {
        const double dx = (x - cx) / rx;
        const double dy = (y - cy) / ry;
        return dx * dx + dy * dy <= 1.0;
    }
    bool operator==(const Mask&) const = default;
};

/// Pixel (x, y) of a tile.
inline std::uint8_t px(const TileView& t, std::size_t x, std::size_t y) {
    return t.data[(t.y0 + y) * t.stride + t.x0 + x];
}

/// Histogram of a tile on a step-pixel grid inside mask.
Histogram tile_histogram(const TileView& t, std::size_t step, const Mask& mask);

/// Side of a metering cell in tile pixels.
inline constexpr std::size_t METER_CELL = 16;
/// A cell with more than this fraction of clipped samples is left out of the mean.
inline constexpr double METER_CELL_CLIP = 0.5;

/// The ONE brightness metric (auto exposure, room setup, the "good frame" bit).
struct SceneMetric {
    /// Masked mean of the unclipped cells (0..255). All masked samples when every cell is clipped.
    double mean = 0.0;
    /// Fraction of masked samples at or above the saturation level.
    double sat = 0.0;
    /// Fraction of masked samples at or below 8.
    double dark = 0.0;
    /// Masked samples.
    std::uint32_t n = 0;
    bool operator==(const SceneMetric&) const = default;
};

/// Metric plus the histograms it was made from (all = every masked sample, kept = the unclipped cells).
struct SceneMetricHist {
    SceneMetric metric;
    Histogram all;
    Histogram kept;
};
SceneMetricHist scene_metric_hist(std::span<const TileView> tiles, std::size_t step, const Mask& mask,
                                  std::uint8_t sat_level);
/// The shared brightness metric of some tiles (one camera pair, or one frame set).
SceneMetric scene_metric(std::span<const TileView> tiles, std::size_t step, const Mask& mask, std::uint8_t sat_level);
/// scene_metric of a tightly packed 8-bit grey image: default mask scaled to w x h, saturation 250, every 2nd pixel.
SceneMetric scene_metric_grey(std::span<const std::uint8_t> data, std::size_t w, std::size_t h);

/// "Good frame": masked mean at least this ...
inline constexpr double GOOD_MEAN_MIN = 30.0;
/// ... and at most this ...
inline constexpr double GOOD_MEAN_MAX = 190.0;
/// ... clipped fraction at most this ...
inline constexpr double GOOD_SAT_MAX = 0.20;
/// ... and quiet for this long after a big exposure step (ns).
inline constexpr std::uint64_t GOOD_QUIET_NS = 1'000'000'000;
/// A step of the exposure x gain product by more than this ratio (either way) is a big step.
inline constexpr double BIG_STEP_RATIO = 1.25;
/// Exposure or gain that differs from the previous frame by more than this fraction means a step landed.
inline constexpr double STEP_LANDED_FRAC = 0.03;

/// The one definition of a good tracking frame for a pair.
bool is_good_frame(const SceneMetric& m, bool step_landed, std::optional<std::uint64_t> since_big_step_ns,
                   bool settling);

/// Follows a pair's own frame metadata (exposure, gain) to find landed and big steps for is_good_frame.
class StepWatch {
public:
    /// One tracking frame of the pair. Returns (a step landed in this frame, time since the last big step).
    std::pair<bool, std::optional<std::uint64_t>> push(std::uint16_t exposure, std::uint16_t gain, std::uint64_t now_ns);
    /// push and is_good_frame in one call.
    bool good(const SceneMetric& m, std::uint16_t exposure, std::uint16_t gain, std::uint64_t now_ns, bool settling);

private:
    std::optional<std::pair<std::uint16_t, std::uint16_t>> prev_;
    std::optional<std::uint64_t> big_at_;
};

/// Which control law runs. PIMAX_CAM_AE_LAW=old (also v1, off, 0) selects V1, the kill switch.
enum class AeLaw {
    /// The law before 2026-10-17: trimmed mean, floor 100 us, rule at 5 % clipped, deadband 0.08.
    V1,
    /// Masked mean over unclipped cells, floor 1140 us (300 us low stage), rule above 20 %, deadband 0.15, 5 changes/s.
    V2,
};

/// Exposure floor of the normal range (Windows' lowest observed value).
inline constexpr std::uint16_t EXPOSURE_FLOOR = 1140;
/// Exposure floor of the low stage (only after 3 s of more than 30 % clipped).
inline constexpr std::uint16_t EXPOSURE_LOW_STAGE = 300;

/// Thresholds of the low-light governor (extended range and the "tracking light low" warning).
struct LightParams {
    bool enabled = true;
    double extend_mean = 20.0;
    std::uint64_t extend_ns = 5'000'000'000;
    double retract_mean = 80.0;
    double settled_mean = 40.0;
    std::uint64_t retract_ns = 8'000'000'000;
    double low_mean = 15.0;
    std::uint64_t low_ns = 10'000'000'000;
    double clear_mean = 25.0;
    std::uint64_t clear_ns = 3'000'000'000;
    std::uint16_t ext_exposure = POWER_ON_EXPOSURE;
    std::uint16_t ext_gain = POWER_ON_GAIN;
    bool clip_enabled = true;
    double clip_frac = 0.30;
    std::uint64_t clip_ns = 3'000'000'000;
    std::uint64_t clip_back_ns = 8'000'000'000;
    bool operator==(const LightParams&) const = default;
};

/// Controller parameters (raw units as on the wire). The defaults are the Dream Air's (Windows' behaviour from the
/// captures).
struct AeParams {
    double target = 64.0;
    double deadband = 0.15;
    double k = 0.5;
    double max_step = LADDER_RATIO * LADDER_RATIO * LADDER_RATIO;
    double max_step_far = 2.0;
    std::uint16_t exposure_min = EXPOSURE_FLOOR;
    std::uint16_t exposure_max = 4108;
    std::uint16_t gain_min = 13;
    std::uint16_t gain_max = 215;
    std::uint8_t sat_level = 250;
    double sat_max_frac = 0.20;
    double sat_pull = 0.15;
    double trim = 0.98;
    std::size_t step = 4;
    std::uint64_t period_ns = 80'000'000;
    std::uint16_t led_exposure = 40;
    std::uint16_t led_gain = 62;
    std::array<std::array<std::size_t, 2>, 2> pairs{{{0, 1}, {2, 3}}};
    bool upper = true;
    bool quantize = true;
    std::uint16_t exposure_soft = 4108;
    LightParams light;
    AeLaw law = AeLaw::V2;
    std::uint64_t min_change_ns = 200'000'000;
    double sat_release = 0.12;
    AeModel model = AeModel::DreamAir;

    bool operator==(const AeParams&) const = default;

    /// The defaults for a headset model (Dream Air: the defaults, unchanged).
    static AeParams for_model(AeModel model);
    /// The old law (kill switch).
    static AeParams old_law();
    /// old_law for a model (the Crystal keeps its Windows floor).
    static AeParams old_law_for(AeModel model);
    /// Defaults with the PIMAX_CAM_AE_* overrides (Dream Air).
    static AeParams from_env();
    /// from_env for a model; for the Crystal Super the overrides stay inside Windows' Crystal range. Throws (as the
    /// Rust code panics) where a NaN override reaches a clamp with crossed bounds.
    static AeParams from_env_model(AeModel model);
    /// Exposure and gain caps of pair k (0 lower, 1 upper), tightened to the model's Windows range of that pair.
    std::array<std::uint16_t, 4> pair_caps(std::size_t k) const;
    /// Split a brightness product e * g into (exposure, gain) within the lower pair's caps.
    std::pair<std::uint16_t, std::uint16_t> split(double product) const;
    /// split within the caps of pair k.
    std::pair<std::uint16_t, std::uint16_t> split_pair(std::size_t k, double product) const;
    /// The pair's setting inside its caps: unchanged when it already is, otherwise the split of its product.
    std::pair<std::uint16_t, std::uint16_t> fit_pair(std::size_t k, std::uint16_t e, std::uint16_t g) const;
};

/// A change of the low-light governor, for the driver to log and show.
struct LightEvent {
    enum Kind { Extended, Retracted, LowRaised, LowCleared } kind;
    /// Extended: the new stage (1 = gain up to the extended cap, 2 = exposure too).
    std::uint8_t stage = 0;
    bool operator==(const LightEvent&) const = default;
};

/// Low-light governor state (pure, driven by host time).
struct LightGov {
    /// 0 normal, 1 gain extended, 2 gain and exposure extended.
    std::uint8_t stage = 0;
    /// "Tracking light low" warning raised.
    bool low = false;
    /// The exposure floor is lowered to EXPOSURE_LOW_STAGE (clipping window in view).
    bool clip_low = false;

    /// One tracking frame.
    std::vector<LightEvent> step(const LightParams& lp, std::uint64_t now_ns, double mean, bool at_ceiling, bool settled,
                                 std::uint8_t top);
    /// The clip-driven low stage. Returns true when clip_low changed.
    bool step_clip(const LightParams& lp, std::uint64_t now_ns, bool at_floor, bool free);

private:
    std::optional<std::uint64_t> starved_, easy_, dark_, clear_, last_ns_, clipping_, unneeded_;
};

/// What one update decided.
struct AeDecision {
    std::array<TileStats, 2> stats{};
    /// Log error per pair (ln(target / m), after the saturation rule), 0 inside the deadband.
    std::array<double, 2> err{};
    bool changed = false;
    /// A command should go out now (period elapsed).
    bool send = false;
};

/// The controller.
class AutoExposure {
public:
    explicit AutoExposure(AeParams p);

    AeParams p;
    /// Commands handed out for sending.
    std::uint64_t sends = 0;
    /// Updates that changed the target.
    std::uint64_t changes = 0;
    /// Tracking frames analysed.
    std::uint64_t frames = 0;
    LightGov light;

    /// Highest range stage that exists (0: the normal ceiling is already the extended one, or the feature is off).
    std::uint8_t top_stage() const;
    /// (exposure cap, gain cap) of a range stage.
    std::pair<std::uint16_t, std::uint16_t> caps(std::uint8_t stage) const;
    /// Light events since the last call.
    std::vector<LightEvent> take_events();
    /// Current target (nullopt before the first frame).
    std::optional<AeSetting> target() const { return target_; }
    /// Log-domain error of one pair (0 inside the deadband).
    double error(const TileStats& s) const;
    /// New (exposure, gain, err) for the lower pair.
    std::tuple<std::uint16_t, std::uint16_t, double> step_pair(std::uint16_t e, std::uint16_t g, const TileStats& s) const;
    /// step_pair for pair k.
    std::tuple<std::uint16_t, std::uint16_t, double> step_pair_k(std::size_t k, std::uint16_t e, std::uint16_t g,
                                                                 const TileStats& s) const;
    /// One tracking frame: setting = what the frame was taken with, stats = per pair, now_ns = host time.
    AeDecision update(const AeSetting& setting, std::array<TileStats, 2> stats, std::uint64_t now_ns);
    /// Pool the tiles of each pair of a tracking frame.
    std::array<TileStats, 2> frame_stats(const std::array<TileView, 4>& tiles, const Mask& mask) const;

private:
    std::tuple<std::uint16_t, std::uint16_t, double> step_pair_with(std::size_t k, std::uint16_t e, std::uint16_t g,
                                                                    const TileStats& s, bool no_brighter) const;
    void govern(const AeSetting& setting, const std::array<TileStats, 2>& stats, std::uint64_t now_ns);

    std::optional<AeSetting> target_;
    std::optional<std::uint64_t> last_send_ns_;
    std::uint16_t base_exposure_max_;
    std::uint16_t base_gain_max_;
    std::uint16_t base_exposure_min_;
    std::vector<LightEvent> events_;
    std::array<std::optional<std::uint64_t>, 2> last_change_ns_{};
    std::array<bool, 2> sat_latch_{};
};

// ---- the driver-facing texts (capi_ae.rs) ----

/// The headset text of the low-light warning.
inline constexpr const char* LOW_LIGHT_TEXT =
    "Tracking cameras see too little infrared light — daylight or an IR lamp helps";
/// Start-up marker line of the headset model (PIMAX_AE_MODEL).
std::string model_line(const AutoExposure& ae, std::string_view why);
/// Start-up marker line (PIMAX_AE_EXTENDED).
std::string start_line(const AutoExposure& ae);
/// One log line per governor event.
std::string event_line(const LightEvent& ev, const AutoExposure& ae);
/// XSOverlay-format message for WayVR.
std::string toast_message(std::string_view text);
/// Send the toast to 127.0.0.1:42069 (UDP, best effort).
void toast(std::string_view text);

} // namespace pibridge::cam::ae
