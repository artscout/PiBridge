// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/calib.rs (MIT OR Apache-2.0).
//
// Per-eye host-side gaze calibration (fitted by the eye calibration tool, applied by the live pipeline).
//
// The EyeChip's own calibration is never touched (no calibration opcode is sent). Each eye's raw head-frame gaze
// direction (after the device-to-head conversion of FrameConv) is mapped on its own, then the two corrected rays are
// combined.
//
// Per-eye map: on the tangent `(u, v) = (x / -z, y / -z)` of a head-frame direction (OpenXR axes, -z forward) a
// polynomial `u' = cu . f(u, v)`, `v' = cv . f(u, v)`, `f = (1, u, v, u^2, u v, v^2)` (the Correction of the old
// combined correction). Three model sizes:
//
// * `offset`: identity plus a constant (2 free numbers per eye): what a kappa angle / headset slip looks like,
// * `affine`: plus gain and cross terms (6 per eye): the ~0.6 gain of the uncalibrated chip,
// * `quadratic`: plus the second-order terms (12 per eye), ridge-regularised towards affine: the compressed upper
//   field seen in the captures.
//
// The fit (fit_eye) is weighted least squares on per-target central values (median of the fixation samples), with
// the identity map as the prior (ridge on the departure from it: free offset, small penalty on the linear terms, a
// selectable one on the quadratic terms) and Huber reweighting (IRLS) so that one bad target cannot drag the map.
// select_model picks the model by leave-one-target-out error and prefers the simpler one unless the richer one is
// clearly better.
//
// Origin terms: the chip's direction is "from its estimate of the eye centre to the pupil", so an error of 1 mm in the
// reported eye-centre origin is a direction error of about 1 mm / 10 mm = 6 degrees (measured on the Dream Air: 0.10
// tangent per mm). A map that sees only the direction cannot remove that; OriginTerms adds `u' += ou . d,
// v' += ov . d` with `d` the reported origin minus a reference origin (millimetres), and a gate that drops (hard) or
// down-weights (soft) an eye whose origin is far from the reference (the chip's bad state: origin jumps of 8 mm and
// more with a garbage direction). Fitted on the individual fixation samples (omarchyvr calfit.rs, not ported here).
//
// The truth for an eye is the direction from that eye's rendering position (the OpenXR view position) to the target:
// that is the direction in which the eye saw the target's pixels (collimated optics).
//
// Combination: each valid eye's corrected ray is followed to a fixation distance (Combine::depth_m, default 1.5 m)
// from its eye position; the hit points are averaged with weights (inverse error variance per eye, from the
// validation) and the combined ray goes from the midpoint of the eyes to that point. With one eye the same
// construction keeps the direction consistent (no jump of half the vergence angle when the other eye blinks). When
// both eyes are valid but disagree by more than Combine::max_disagree_deg, the eye with the smaller expected error is
// used alone. An eye that comes back after being invalid is ignored for Combine::holdoff samples (eyelid artefacts at
// blink end).
//
// File (`eye-calibration.toml`): written by the calibration tool, read at every tracker start
// (`PIBRIDGE_EYE_CALIBRATION=<path>|none`; unset: the default file when it exists, `~/.config/pibridge/` first, then
// `~/.config/pimax-linux/` (PiBridge's earlier name), then `~/.config/omarchyvr/`). Byte-compatible with the Rust to_toml/parse (versions 1 and 2), see docs/libpibridge-eyemath.md.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "pibridge/eyemath/correction.h"
#include "pibridge/eyemath/filter.h"
#include "pibridge/eyemath/track.h"
#include "pibridge/eyemath/types.h"

namespace pibridge::eyemath {

/// Model size.
enum class ModelKind { Offset, Affine, Quadratic };

/// Number of features used.
std::size_t model_features(ModelKind k);
/// File name of the model.
const char* model_name(ModelKind k);
/// Parse a file name.
std::optional<ModelKind> parse_model(std::string_view s);
/// Minimum targets for a fit.
std::size_t model_min_points(ModelKind k);

/// One calibration target seen by one eye: central raw tangent, truth tangent, weight.
struct EyePoint {
    /// Robust central raw gaze tangent (u, v).
    Tangent raw;
    /// Tangent of the direction from the eye's rendering position to the target.
    Tangent truth;
    /// Relative weight (number of good samples, dispersion); 1 is normal.
    double weight = 1.0;

    friend bool operator==(const EyePoint&, const EyePoint&) = default;
};

/// Angle in degrees between two tangents (as directions).
double tangent_angle_deg(Tangent a, Tangent b);

/// Mean, median, 95th percentile and maximum.
struct Stats {
    /// Count.
    std::size_t n = 0;
    double mean = 0.0;
    double median = 0.0;
    /// 95th percentile.
    double p95 = 0.0;
    double max = 0.0;

    friend bool operator==(const Stats&, const Stats&) = default;
};

/// Statistics of a list (non-finite values are left out).
Stats stats(std::span<const double> v);

/// Ridge strengths of a fit, relative to each feature's own information (`sum w f^2`): 1 halves a coefficient that
/// the data alone would set, 0.01 barely touches it.
struct Ridge {
    /// On the linear terms' departure from identity.
    double linear = 1e-3;
    /// On the quadratic terms.
    double quadratic = 0.2;

    friend bool operator==(const Ridge&, const Ridge&) = default;
};

/// Huber threshold in tangent units (about 1.1 degrees).
inline constexpr double HUBER_K = 0.02;

/// Robust fit: weighted ridge least squares with Huber reweighting (5 iterations).
std::optional<Correction> fit_eye(std::span<const EyePoint> points, ModelKind kind, Ridge ridge);

/// Error in degrees of map `c` at point `p`.
double point_error_deg(const Correction& c, const EyePoint& p);

/// One evaluated model.
struct ModelFit {
    /// Model size.
    ModelKind kind = ModelKind::Affine;
    /// Ridge used.
    Ridge ridge;
    /// The map fitted on all points.
    Correction map;
    /// Leave-one-target-out error (degrees).
    Stats loo;
    /// Error at the fit points.
    Stats train;
    /// Origin coefficients `(u, v)` per millimetre when the model has them (sample-level fits).
    std::optional<std::pair<Vec3, Vec3>> org;
    /// Leave-one-out error of every target, in the order of the fit points.
    std::vector<double> loo_each;

    friend bool operator==(const ModelFit&, const ModelFit&) = default;
};

/// Fit `kind` on all points and score it by leave-one-target-out.
std::optional<ModelFit> evaluate(std::span<const EyePoint> points, ModelKind kind, Ridge ridge);

/// All candidate models and the chosen one.
struct Selection {
    /// Error of the raw gaze at the points.
    Stats raw;
    /// Candidates in increasing complexity.
    std::vector<ModelFit> candidates;
    /// Index of the chosen candidate.
    std::optional<std::size_t> chosen;

    /// The chosen fit (nullptr when none).
    const ModelFit* best() const { return chosen ? &candidates[*chosen] : nullptr; }
};

/// Fit offset, affine and quadratic (three ridge strengths) and keep the one with the lowest leave-one-out mean
/// error, unless a simpler one is within 7 % (or 0.05 degrees) of it.
Selection select_model(std::span<const EyePoint> points);

/// Largest pupil deviation the pupil terms are applied for, millimetres (the terms are fitted inside the range the
/// calibration scene produced; content of another brightness must not extrapolate them).
inline constexpr double PUPIL_CLAMP_MM = 1.0;

/// Origin-dependent part of an eye's correction and the reliability gate (all zero: not used).
struct OriginTerms {
    /// Reference origin (the median reported origin of the calibration fixations), metres.
    Vec3 reference_m{0.0, 0.0, 0.0};
    /// Tangent added to `u` per millimetre of origin deviation (x, y, z).
    Vec3 u{0.0, 0.0, 0.0};
    /// Tangent added to `v` per millimetre of origin deviation (x, y, z).
    Vec3 v{0.0, 0.0, 0.0};
    /// Origin farther than this from the reference (millimetres) is the chip's bad state: the eye is dropped.
    /// 0: no gate.
    double gate_mm = 0.0;
    /// Reference pupil diameter (the median of the calibration fixations), millimetres.
    double pupil_ref_mm = 0.0;
    /// Tangent added to `u` per millimetre of pupil diameter deviation (clamped to +-PUPIL_CLAMP_MM).
    double pupil_u = 0.0;
    /// Same for `v`.
    double pupil_v = 0.0;

    /// Whether anything is set (a file with any active eye is written as version 2).
    bool is_active() const;
    /// Tangent shift of the pupil terms for a reported diameter (zero when unknown or the terms are not set).
    Tangent pupil_shift(std::optional<double> pupil_mm) const;
    /// Deviation of `origin` (metres) from the reference, millimetres.
    Vec3 deviation_mm(const Vec3& origin) const;
    /// Distance of `origin` from the reference, millimetres.
    double distance_mm(const Vec3& origin) const;
    /// Tangent shift for `origin` (zero when the terms are not set).
    Tangent shift(const Vec3& origin) const;
    /// False for a non-finite origin or one beyond the gate.
    bool usable(const Vec3& origin) const;
    /// Soft reliability in (0, 1]: 1 at the reference, Gaussian in the distance with sigma = gate / 2 (1 without a
    /// gate).
    double reliability(const Vec3& origin) const;

    friend bool operator==(const OriginTerms&, const OriginTerms&) = default;
};

/// One eye of a calibration.
struct EyeCal {
    /// The map (its `model` names the size).
    Correction map;
    /// Rendering position of the eye in the head frame, metres (OpenXR VIEW space).
    Vec3 origin_m{0.0, 0.0, 0.0};
    /// Expected angular error of this eye after calibration (validation), degrees.
    double accuracy_deg = 1.0;
    /// Sample-to-sample noise (RMS), degrees.
    double precision_deg = 0.5;
    /// Drift offset added to the corrected tangent (recentre).
    Tangent drift;
    /// Origin terms and gate (version 2 files).
    OriginTerms org;

    /// Calibrated head-frame direction of a raw head-frame direction.
    std::optional<Vec3> apply(const Vec3& raw) const;
    /// apply() with an extra (running) drift offset.
    std::optional<Vec3> apply_with(const Vec3& raw, Tangent extra) const;
    /// apply_with() with the origin the tracker reported for this sample (metres) and its pupil diameter (mm): the
    /// origin and pupil terms are added, and nothing is returned for an origin beyond the gate.
    std::optional<Vec3> apply_full(const Vec3& raw, std::optional<Vec3> origin, std::optional<double> pupil_mm,
                                   Tangent extra) const;
    /// Inverse error variance used for the combination.
    double weight() const;

    friend bool operator==(const EyeCal&, const EyeCal&) = default;
};

/// How the two eyes become one ray.
struct Combine {
    /// Fixation distance the rays are followed to, metres.
    double depth_m = 1.5;
    /// Larger disagreement between the eyes means one of them is wrong (degrees).
    double max_disagree_deg = 4.0;
    /// Samples an eye is ignored after it was invalid.
    std::uint32_t holdoff = 2;

    friend bool operator==(const Combine&, const Combine&) = default;
};

/// Running state of the combination (blink hold-off).
struct CombineState {
    std::array<std::uint32_t, 2> invalid_run_end{0, 0};
    std::array<std::uint32_t, 2> since_valid{0, 0};
};

struct LoadedCalibration;

/// A complete calibration.
struct Calibration {
    /// Device-to-head conversion the calibration was measured with (the tracker uses it).
    FrameConv conv;
    /// `[left, right]`.
    std::array<EyeCal, 2> eyes{};
    /// Combination.
    Combine combine;
    /// Output filter.
    FilterParams filter;
    /// Unix time of the measurement.
    std::uint64_t created_unix = 0;
    /// Free text lines written as comments (validation summary, ...).
    std::vector<std::string> notes;

    /// Per-eye identity maps (for tests and as a start), eyes at -+ipd/2 on x.
    static Calibration identity(FrameConv conv, double ipd_m);

    /// The file text (version 1, or 2 when an eye has active origin terms).
    std::string to_toml() const;
    /// Parse the file text.
    static std::expected<Calibration, Error> parse(std::string_view text);
    /// Load a file.
    static std::expected<Calibration, Error> load(const std::filesystem::path& path);
    /// Write to_toml() to `path` (temporary file + rename; no backup of an existing file: the caller keeps one).
    std::expected<void, Error> save(const std::filesystem::path& path) const;
    /// `~/.config/pibridge/eye-calibration.toml` (where a new file goes); empty without `$HOME`.
    static std::optional<std::filesystem::path> default_path();
    /// The default file that exists under `home`: the pibridge one, else the pimax-linux one, else the omarchyvr one.
    static std::optional<std::filesystem::path> find_default(const std::optional<std::filesystem::path>& home);
    /// Selection by `PIBRIDGE_EYE_CALIBRATION` (`none`/`off`/`0`: no calibration; a path: that file, which must load;
    /// unset or empty: the default file when it exists).
    static std::expected<std::optional<LoadedCalibration>, Error> select(
        std::optional<std::string_view> var, const std::optional<std::filesystem::path>& home);
    /// select() with the real environment (`PIBRIDGE_EYE_CALIBRATION`, `HOME`).
    static std::expected<std::optional<LoadedCalibration>, Error> from_env();

    /// Calibrate a raw sample in place: per-eye directions are replaced by the calibrated ones (origins by the
    /// calibration's eye positions), and the combined ray is rebuilt as described above.
    void apply_sample(Sample& s, CombineState& st) const;
    /// apply_sample() with running drift offsets per eye (Drift::offset).
    void apply_sample_with(Sample& s, CombineState& st, const std::array<Tangent, 2>& extra) const;

    friend bool operator==(const Calibration&, const Calibration&) = default;
};

/// A calibration and the file it came from.
struct LoadedCalibration {
    Calibration cal;
    std::filesystem::path path;
};

/// A raw eye ray of a sample as (origin m, direction) in double when valid.
std::optional<std::pair<Vec3, Vec3>> raw_ray(const EyeRay& e);

namespace detail {
/// Gaussian elimination with partial pivoting; empty for a singular system (Rust `pub(crate)`, for the tests).
std::optional<std::vector<double>> solve(std::vector<std::vector<double>> a, std::vector<double> b);
/// Weighted ridge least squares of the departure from identity; `w` overrides the point weights (Rust private,
/// for the tests).
std::optional<Correction> fit_weighted(std::span<const EyePoint> points, std::span<const double> w, ModelKind kind,
                                       Ridge ridge);
} // namespace detail

} // namespace pibridge::eyemath
