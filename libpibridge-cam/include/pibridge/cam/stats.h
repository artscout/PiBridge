// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-cam/src/stats.rs (MIT OR Apache-2.0).
//
// Capture statistics for the tool and live tests: frame intervals, rates per frame kind, transfer latency (host
// arrival minus device time, relative to its minimum) and the exposure/gain/row-offset sets seen. The report text is
// the Rust one, character for character.
#pragma once

#include "pibridge/cam/camera.h"

#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace pibridge::cam {

/// Simple running summary of a series.
class Series {
public:
    void push(double x) { v_.push_back(x); }
    std::size_t len() const { return v_.size(); }
    bool is_empty() const { return v_.empty(); }
    double mean() const;
    double std() const;
    double min() const;
    double max() const;
    /// Quantile 0..1 (NaN when empty).
    double quantile(double q) const;
    /// "mean ± std [min .. max] unit (n=N)".
    std::string fmt(const char* unit) const;

private:
    std::vector<double> v_;
};

/// (kind, exposure, gain, tile rows) of a frame.
using SettingsKey =
    std::tuple<std::uint32_t, std::array<std::uint16_t, 4>, std::array<std::uint16_t, 4>, std::array<std::uint16_t, 4>>;

/// Accumulates FrameInfos.
class CaptureStats {
public:
    void add(const FrameInfo& i);
    /// Finalise derived series (call once before printing).
    void finish();
    /// Wall duration covered (s).
    double duration_s() const;
    /// Average frame rate.
    double fps() const;
    /// Multi-line report.
    std::string report() const;

    Series host_dt_ms;
    Series dev_dt_ms;
    Series tracking_dt_ms;
    Series latency_ms;
    Series mono_minus_host_ms;
    std::map<SettingsKey, std::uint64_t> settings;
    std::uint64_t frames = 0;
    std::uint64_t dropped = 0;

private:
    std::optional<std::uint64_t> first_host_;
    std::uint64_t last_host_ = 0;
    std::optional<FrameInfo> last_;
    std::optional<std::uint64_t> last_tracking_dev_;
    std::vector<double> raw_lat_;
};

} // namespace pibridge::cam
