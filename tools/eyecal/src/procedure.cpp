// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/procedure.rs (MIT OR Apache-2.0); the on-chip mode (Mode::Chip) is new.
#include "procedure.h"

#include "refit.h"
#include "report.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>

namespace pibridge::eyecal {

namespace cal = pibridge::eye::calibration;

namespace {

constexpr Rgb WHITE = {1.0F, 1.0F, 1.0F};
constexpr Rgb GREEN = {0.2F, 1.0F, 0.35F};
constexpr Rgb AMBER = {1.0F, 0.7F, 0.1F};
constexpr Rgb RED = {1.0F, 0.25F, 0.2F};
constexpr Rgb GREY = {0.55F, 0.55F, 0.58F};
/// The green dot on white (the dark-background green is too light there).
constexpr Rgb PIMAX_GREEN_ON_WHITE = {0.0F, 0.55F, 0.15F};
/// The dimmed dot of an accepted target.
constexpr Rgb DIM = {0.8F, 0.8F, 0.8F};

/// A sample older than this is no longer "the current gaze".
constexpr double RECENT_S = 1.0;

constexpr double deg(double r) { return r * 180.0 / std::numbers::pi; }

double quat_angle_deg(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    const double d = std::min(std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]), 1.0);
    return 2.0 * deg(std::acos(d));
}

double smoothstep(double f) { return f * f * (3.0 - 2.0 * f); }

/// The fixation target recommended by Thaler et al. (2013) on another background (the cross is cut in the background
/// colour): a disc with a cross cut out and a dot in the middle.
void abc_on(std::vector<Mark>& marks, const Vec3& p, Rgb color, Rgb background) {
    marks.push_back(Mark{p, MarkKind::Disc, 0.45, 0.0, color});
    marks.push_back(Mark{p, MarkKind::Cross, 0.47, 0.09, background});
    marks.push_back(Mark{p, MarkKind::Disc, 0.08, 0.0, color == GREEN ? GREEN : RED});
}

void abc(std::vector<Mark>& marks, const Vec3& p, Rgb color) { abc_on(marks, p, color, BACKGROUND); }

void ring(std::vector<Mark>& marks, const Vec3& p, double r, Rgb color) {
    marks.push_back(Mark{p, MarkKind::Ring, r, 0.14, color});
}

double angle(const Vec3& a, const Vec3& b) {
    const Vec3 c = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    const double s = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    return deg(std::atan2(s, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]));
}

/// Median, 0 for an empty list (procedure.rs).
double median0(std::vector<double> v) {
    if (v.empty()) {
        return 0.0;
    }
    return median_nan(std::move(v));
}

std::string fmt_opt(std::optional<double> x) { return x ? std::format("{:.2f}", *x) : std::string("-"); }

Vec3 mid(const std::array<Vec3, 2>& p) {
    return {0.5 * (p[0][0] + p[1][0]), 0.5 * (p[0][1] + p[1][1]), 0.5 * (p[0][2] + p[1][2])};
}

Vec3 scaled(const Vec3& d, double k) { return {d[0] * k, d[1] * k, d[2] * k}; }

std::optional<Vec3> gaze_dir(const em::Sample& s) {
    if (!s.gaze) {
        return std::nullopt;
    }
    return Vec3{s.gaze->direction[0], s.gaze->direction[1], s.gaze->direction[2]};
}

std::string upper(std::string s) {
    for (auto& c : s) {
        if (c >= 'a' && c <= 'z') {
            c = static_cast<char>(c - 'a' + 'A');
        }
    }
    return s;
}

/// (u, v) of a tangent offset in degrees (right, up).
std::pair<double, double> tangent_deg(Tangent t) { return {deg(std::atan(t.u)), deg(std::atan(t.v))}; }

} // namespace

const char* eye_name(std::size_t e) { return e == 0 ? "left" : "right"; }

const char* set_name(Set s) {
    switch (s) {
    case Set::Calibration: return "Calibration";
    case Set::Validation: return "Validation";
    case Set::Recenter: return "Recenter";
    case Set::Service: return "Service";
    case Set::Chip: return "Chip";
    }
    return "?";
}

const char* stage_name(Stage s) {
    switch (s) {
    case Stage::Attract: return "Attract";
    case Stage::Hold: return "Hold";
    case Stage::Collect: return "Collect";
    case Stage::Accepted: return "Accepted";
    case Stage::HeadMoved: return "HeadMoved";
    }
    return "?";
}

const char* phase_name(Phase::Kind k) {
    switch (k) {
    case Phase::Kind::Intro: return "Intro";
    case Phase::Kind::Point: return "Point";
    case Phase::Kind::Pursuit: return "Pursuit";
    case Phase::Kind::Results: return "Results";
    case Phase::Kind::Cursor: return "Cursor";
    case Phase::Kind::ServiceWait: return "ServiceWait";
    case Phase::Kind::ServiceCursor: return "ServiceCursor";
    case Phase::Kind::ChipBusy: return "ChipBusy";
    case Phase::Kind::Grid: return "Grid";
    case Phase::Kind::GridSaved: return "GridSaved";
    }
    return "?";
}

Settings Settings::pimax() const {
    Settings x = *this;
    x.pattern = Pattern::Pimax;
    x.attract_s = 1.0;
    x.collect_s = 2.0;
    return x;
}

Settings Settings::chip() const {
    Settings x = pimax();
    x.mode = Mode::Chip;
    return x;
}

Procedure::Procedure(Settings settings) : s(std::move(settings)) {
    cal_t_ = s.pattern == Pattern::Grid ? eyecal::calibration_targets(s.cal_points) : pimax_targets();
    val_t_ = eyecal::validation_targets();
    chip_t_ = eyecal::chip_targets(ChipLayout{s.conv, s.chip_mirror_x, s.chip_mirror_y});
    if (grid_flow()) {
        phase = Phase::of(Phase::Kind::Grid);
    }
}

bool Procedure::grid_flow() const {
    return s.mode == Mode::Chip || (s.mode == Mode::Recenter && s.pattern == Pattern::Pimax);
}

void Procedure::set_chip_ready(bool ready) {
    chip_ready_ = ready;
    if (ready && status_ == "WAITING FOR THE EYE TRACKER") {
        status_.clear();
    }
}

Target Procedure::target(Set set, std::size_t i) const {
    switch (set) {
    case Set::Calibration: return cal_t_[i];
    case Set::Validation: return val_t_[i];
    case Set::Recenter: return recenter_targets()[i];
    case Set::Service: return service_targets()[i];
    case Set::Chip: return chip_t_[i];
    }
    return Target{};
}

std::size_t Procedure::set_len(Set set) const {
    switch (set) {
    case Set::Calibration: return cal_t_.size();
    case Set::Validation: return val_t_.size();
    case Set::Recenter: return recenter_targets().size();
    case Set::Service: return service_targets().size();
    case Set::Chip: return chip_t_.size();
    }
    return 0;
}

std::array<double, 2> Procedure::eye_ratio() const {
    const double n = static_cast<double>(std::max<std::size_t>(recent_.size(), 1));
    std::array<double, 2> r{};
    for (std::size_t e = 0; e < 2; e++) {
        std::size_t c = 0;
        for (const auto& x : recent_) {
            c += x.eyes[e] ? 1 : 0;
        }
        r[e] = static_cast<double>(c) / n;
    }
    return r;
}

std::string Procedure::tag() const {
    if (phase.is(Phase::Kind::Point)) {
        const Target t = target(phase.set, phase.i);
        return std::format("{}:{}:{}:{}", set_name(phase.set), phase.i, t.name(), stage_name(phase.stage));
    }
    if (phase.is(Phase::Kind::Pursuit)) {
        const Target p = pursuit_at(std::max(now_ - phase.t0 - 1.0, 0.0));
        return std::format("Pursuit:{:.2f},{:.2f}", p.az, p.el);
    }
    return phase_name(phase.kind);
}

std::vector<Effect> Procedure::command(Cmd c) {
    if (s.mode == Mode::Chip) {
        return chip_command(c);
    }
    if (grid_flow()) {
        return recenter_command(c);
    }
    std::vector<Effect> fx;
    // The Pimax path has two buttons, as Pimax Play: at the start A calibrate, B exit; at the end A again, B save and
    // exit. Nothing else.
    const bool results = phase.is(Phase::Kind::Results) || phase.is(Phase::Kind::Cursor);
    if (s.pattern == Pattern::Pimax) {
        if (c == Cmd::Next && results) {
            *this = Procedure(s);
            fx.push_back(Effect::print("calibrating again"));
            c = Cmd::Next; // from the fresh intro: start at once
        } else if (c == Cmd::Redo && results) {
            auto a = accept();
            fx.insert(fx.end(), a.begin(), a.end());
            fx.push_back(Effect::of(Effect::Kind::Quit));
            return fx;
        } else if ((c == Cmd::Next || c == Cmd::Start) && phase.is(Phase::Kind::Intro)) {
            c = Cmd::Next;
        } else if (c == Cmd::Redo && phase.is(Phase::Kind::Intro)) {
            fx.push_back(Effect::of(Effect::Kind::Quit));
            return fx;
        } else if (c != Cmd::Accept) {
            return fx;
        }
    } else if (c == Cmd::Start) {
        c = Cmd::Next;
    }
    const auto k = phase.kind;
    using K = Phase::Kind;
    if (c == Cmd::Next && k == K::Intro) {
        if (s.mode == Mode::Full) {
            fx.push_back(Effect::print(std::format("calibration: {} targets, then {} validation targets; keep the head still",
                                                   cal_t_.size(), val_t_.size())));
        } else {
            fx.push_back(Effect::print("recentre: 5 targets"));
        }
        start_point(s.mode == Mode::Recenter ? Set::Recenter : Set::Calibration, 0, 1);
    } else if (c == Cmd::Redo) {
        *this = Procedure(s);
        fx.push_back(Effect::print("starting over"));
    } else if (c == Cmd::Accept && (k == K::Results || k == K::Cursor || k == K::ServiceCursor)) {
        auto a = accept();
        fx.insert(fx.end(), a.begin(), a.end());
    } else if ((c == Cmd::Next || c == Cmd::Cursor) && k == K::Results && calibration) {
        phase = Phase::of(K::Cursor);
    } else if ((c == Cmd::Next || c == Cmd::Cursor) && k == K::Cursor) {
        phase = Phase::of(K::Results);
    } else if (c == Cmd::Pursuit && (k == K::Results || k == K::Cursor) && calibration) {
        pursuit_samples_.clear();
        phase = Phase::of(K::Pursuit, now_);
    } else if (c == Cmd::Service && (k == K::Results || k == K::Cursor)) {
        if (saved) {
            phase = Phase::of(K::ServiceWait, now_);
            service_err.clear();
            fx.push_back(Effect::of(Effect::Kind::StartService));
            fx.push_back(Effect::print("end-to-end check: the chip is handed to the runtime, which applies the saved calibration"));
        } else {
            status_ = "SAVE FIRST (MENU / Y), THEN S";
        }
    } else if ((c == Cmd::Next || c == Cmd::Service || c == Cmd::Cursor) && k == K::ServiceCursor) {
        phase = Phase::of(K::Results);
        fx.push_back(Effect::of(Effect::Kind::StopService));
    }
    return fx;
}

std::vector<Effect> Procedure::accept() {
    if (!calibration) {
        status_ = "NOTHING TO SAVE";
        return {};
    }
    saved = true;
    status_ = "SAVED: NEW GAZE APPS USE IT. S: CHECK THROUGH THE RUNTIME";
    return {Effect::with_text(Effect::Kind::SaveCalibration, calibration->to_toml())};
}

std::size_t Procedure::target_index(std::size_t k) const {
    for (std::size_t i = 0; i < cal_t_.size(); i++) {
        if (cal_t_[i] == cal_res[k].target) {
            return i;
        }
    }
    return k;
}

void Procedure::start_point(Set set, std::size_t i, std::uint32_t attempt) {
    window_.clear();
    head_ref_.reset();
    head_max_ = 0.0;
    head_speed_ = 0.0;
    phase = Phase::point(set, i, Stage::Attract, now_, attempt);
}

std::vector<Effect> Procedure::frame(double t, std::optional<std::array<double, 4>> head,
                                     const std::array<Vec3, 2>& eye_pos, const std::vector<RawSample>& samples,
                                     std::optional<Vec3> service) {
    now_ = t;
    bool finite = true;
    for (const auto& p : eye_pos) {
        for (const double c : p) {
            finite = finite && std::isfinite(c);
        }
    }
    if (finite && std::fabs(eye_pos[1][0] - eye_pos[0][0]) > 0.01) {
        eye_pos_ = eye_pos;
    }
    for (const auto& x : samples) {
        recent_.push_back(x);
    }
    while (!recent_.empty() && recent_.front().t < t - RECENT_S) {
        recent_.pop_front();
    }
    update_cursor(samples);
    service_cursor_ = service;
    // head motion
    if (head) {
        if (head_prev_) {
            const double dt = t - head_prev_->first;
            if (dt > 1e-3) {
                head_speed_ = std::max(head_speed_, quat_angle_deg(*head, head_prev_->second) / dt);
            }
        }
        head_prev_ = std::pair{t, *head};
        if (head_ref_) {
            head_max_ = std::max(head_max_, quat_angle_deg(*head, *head_ref_));
        }
    }
    std::vector<Effect> fx;
    const auto append = [&fx](std::vector<Effect> more) { fx.insert(fx.end(), more.begin(), more.end()); };
    if (phase.is(Phase::Kind::Point)) {
        const Set set = phase.set;
        const std::size_t i = phase.i;
        const Stage stage = phase.stage;
        const double t0 = phase.t0;
        const std::uint32_t attempt = phase.attempt;
        if (!head_ref_) {
            head_ref_ = head;
            head_speed_ = 0.0;
        }
        const bool moved = head_max_ > s.head_limit_deg || head_speed_ > s.head_speed_dps;
        const double dt = t - t0;
        // a chip target whose request is out cannot start over: the chip collects it now
        const bool restartable = (stage == Stage::Attract || stage == Stage::Hold || stage == Stage::Collect) &&
                                 set != Set::Service && !(set == Set::Chip && stage == Stage::Collect);
        if (restartable && moved) {
            head_restarts++;
            fx.push_back(Effect::print(std::format("target {} ({}): head moved {:.1f} deg ({:.0f} deg/s), starting the target again",
                                                   i + 1, target(set, i).name(), head_max_, head_speed_)));
            phase = Phase::point(set, i, Stage::HeadMoved, t, attempt);
        } else if (stage == Stage::Attract && dt >= s.attract_s) {
            // a chip target waits from the moment the dot stopped: the chip must get its request while the eyes are on
            // the target, not while they still rest on the previous one (the host fit records a window and finds the
            // fixation in it afterwards, so its hold counts from the start, as in the Rust tool)
            if (set == Set::Chip) {
                window_.clear();
            }
            phase = Phase::point(set, i, Stage::Hold, set == Set::Chip ? t : t0, attempt);
        } else if (stage == Stage::Hold) {
            if (set == Set::Chip) {
                // the chip's targets keep the hold too: the fixation the chip collects starts in it
                window_.insert(window_.end(), samples.begin(), samples.end());
            }
            if ((dt >= s.min_hold_s && steady(t)) || dt >= s.max_hold_s) {
                if (set != Set::Chip) {
                    window_.clear();
                }
                if (set == Set::Chip) {
                    // the wearer looks at the target now: the chip collects it while its request is answered
                    const cal::Step* cur = chip_run_.current();
                    if (cur != nullptr && cur->op == cal::OP_TARGET && cur->target == i) {
                        chip_inflight_ = *cur;
                        chip_sent_at_ = t;
                        chip_last_target_ = i;
                        fx.push_back(send(*cur));
                        phase = Phase::point(set, i, Stage::Collect, t, attempt);
                    }
                } else {
                    phase = Phase::point(set, i, Stage::Collect, t, attempt);
                }
            }
        } else if (stage == Stage::Collect) {
            if (set == Set::Service) {
                if (service) {
                    service_samples_.emplace_back(t, *service);
                }
            } else {
                window_.insert(window_.end(), samples.begin(), samples.end());
            }
            if (set != Set::Chip && dt >= s.collect_s) {
                append(finish_point(set, i, attempt));
            }
        } else if (stage == Stage::Accepted && dt >= 0.25) {
            append(next_point(set, i));
        } else if (stage == Stage::HeadMoved && dt >= 1.2) {
            start_point(set, i, attempt);
        }
    } else if (phase.is(Phase::Kind::Pursuit)) {
        const double t0 = phase.t0;
        if (t - t0 > 1.0) {
            pursuit_samples_.insert(pursuit_samples_.end(), samples.begin(), samples.end());
        }
        if (t - t0 >= 1.0 + PURSUIT_PERIOD_S) {
            append(finish_pursuit(t0));
        }
    } else if (phase.is(Phase::Kind::ServiceWait)) {
        if (service) {
            service_samples_.clear();
            start_point(Set::Service, 0, 1);
        } else if (t - phase.t0 > 20.0) {
            status_ = "NO GAZE FROM THE RUNTIME (EYE TRACKING SERVICE?)";
            phase = Phase::of(Phase::Kind::ServiceCursor);
        }
    }
    return fx;
}

bool Procedure::steady(double t) const {
    std::vector<const RawSample*> last;
    for (const auto& x : recent_) {
        if (x.t >= t - 0.25) {
            last.push_back(&x);
        }
    }
    if (last.size() < 15) {
        return false;
    }
    int ok_eyes = 0;
    const double lim = std::tan(1.5 * std::numbers::pi / 180.0);
    for (std::size_t e = 0; e < 2; e++) {
        std::vector<OptTan> tan;
        std::size_t valid = 0;
        for (const auto* x : last) {
            tan.push_back(x->eyes[e] ? em::tangent_of(x->eyes[e]->dir) : std::nullopt);
            valid += tan.back() ? 1 : 0;
        }
        if (valid * 10 < tan.size() * 7) {
            continue;
        }
        double lo_u = std::numeric_limits<double>::max();
        double hi_u = std::numeric_limits<double>::lowest();
        double lo_v = lo_u;
        double hi_v = hi_u;
        for (const auto& x : smooth(tan, 3)) {
            if (x) {
                lo_u = std::min(lo_u, x->u);
                hi_u = std::max(hi_u, x->u);
                lo_v = std::min(lo_v, x->v);
                hi_v = std::max(hi_v, x->v);
            }
        }
        if (hi_u - lo_u <= lim && hi_v - lo_v <= lim) {
            ok_eyes++;
        }
    }
    return ok_eyes > 0;
}

std::vector<Effect> Procedure::finish_point(Set set, std::size_t i, std::uint32_t attempt) {
    const Target tg = target(set, i);
    std::vector<Effect> fx;
    if (set == Set::Service) {
        const Vec3 truth = tg.dir();
        std::vector<double> errs;
        for (const auto& [_, d] : service_samples_) {
            errs.push_back(angle(d, truth));
        }
        const std::optional<double> e = errs.empty() ? std::nullopt : std::optional<double>(median0(errs));
        fx.push_back(Effect::print(std::format(
            "runtime check {}/{} ({}): {}", i + 1, service_targets().size(), tg.name(),
            e ? std::format("error {:.2f} deg (median of {} frames)", *e, errs.size()) : std::string("no gaze"))));
        service_err.push_back(e);
        service_samples_.clear();
        phase = Phase::point(set, i, Stage::Accepted, now_, attempt);
        return fx;
    }
    std::vector<RawSample> w = std::move(window_);
    window_.clear();
    PointRes res;
    res.target = tg;
    res.eyes = {analyze_eye(w, 0, s.fix), analyze_eye(w, 1, s.fix)};
    res.ok = std::ranges::any_of(res.eyes, [](const auto& e) { return e && e->used >= 30; });
    res.samples = std::move(w);
    res.head_deg = head_max_;
    res.attempts = attempt;
    const auto eye = [](const std::optional<EyeWindow>& e) {
        return e ? std::format("{} of {} used, noise {:.2f} deg", e->used, e->total, e->rms_s2s_deg)
                 : std::string("no fixation");
    };
    fx.push_back(Effect::print(std::format("{} {}/{} ({}): left {}; right {}; head {:.1f} deg{}", set_name(set), i + 1,
                                           set_len(set), res.target.name(), eye(res.eyes[0]), eye(res.eyes[1]),
                                           res.head_deg, res.ok ? "" : "  [not usable]")));
    if (!res.ok && attempt < 2) {
        fx.push_back(Effect::print("  again (blink, no steady gaze?)"));
        start_point(set, i, attempt + 1);
        return fx;
    }
    if (set == Set::Calibration) {
        auto slot = std::ranges::find_if(cal_res, [&](const PointRes& r) { return r.target == tg; });
        if (slot != cal_res.end()) {
            *slot = std::move(res);
        } else {
            cal_res.push_back(std::move(res));
        }
    } else if (set == Set::Validation) {
        em::Calibration raw_cal = em::Calibration::identity(s.conv, eye_pos_[1][0] - eye_pos_[0][0]);
        for (std::size_t e = 0; e < 2; e++) {
            raw_cal.eyes[e].origin_m = eye_pos_[e];
        }
        if (calibration) {
            const auto ev = evaluate_window(*calibration, res.samples, tg, s.fix);
            const auto raw_ev = evaluate_window(raw_cal, res.samples, tg, s.fix);
            fx.push_back(Effect::print(std::format("  validation error: combined {} (raw {}), left {}, right {}",
                                                   fmt_opt(ev.comb_err_deg), fmt_opt(raw_ev.comb_err_deg),
                                                   fmt_opt(ev.eye_err_deg[0]), fmt_opt(ev.eye_err_deg[1]))));
            val_res.push_back(ValRes{std::move(res), ev, raw_ev});
        }
    } else if (set == Set::Recenter) {
        rec_res.push_back(std::move(res));
    }
    phase = Phase::point(set, i, Stage::Accepted, now_, attempt);
    return fx;
}

std::vector<Effect> Procedure::next_point(Set set, std::size_t i) {
    if (set == Set::Chip) {
        return chip_advance();
    }
    if (set == Set::Calibration && in_redo_) {
        if (!redo_queue_.empty()) {
            const std::size_t j = redo_queue_.front();
            redo_queue_.erase(redo_queue_.begin());
            start_point(set, j, 2);
            return {};
        }
        in_redo_ = false;
        return finish_calibration();
    }
    if (i + 1 < set_len(set)) {
        start_point(set, i + 1, 1);
        return {};
    }
    switch (set) {
    case Set::Calibration: return finish_calibration();
    case Set::Validation: return finish_validation();
    case Set::Recenter: return s.mode == Mode::Chip ? finish_chip_validation() : finish_recenter();
    case Set::Service: return finish_service();
    case Set::Chip: break;
    }
    return {};
}

std::vector<std::pair<std::size_t, em::EyePoint>> Procedure::eye_points(std::size_t e) const {
    std::vector<std::pair<std::size_t, em::EyePoint>> out;
    for (std::size_t k = 0; k < cal_res.size(); k++) {
        const auto& r = cal_res[k];
        if (!r.eyes[e]) {
            continue;
        }
        const auto truth = truth_tangent(r.target, eye_pos_[e]);
        if (!truth) {
            continue;
        }
        out.emplace_back(k, em::EyePoint{r.eyes[e]->center, *truth,
                                         std::clamp(static_cast<double>(r.eyes[e]->used) / 60.0, 0.3, 1.0)});
    }
    return out;
}

std::vector<Effect> Procedure::finish_calibration() {
    std::vector<Effect> fx;
    std::array<std::optional<em::EyeCal>, 2> eyes{};
    std::array<std::optional<std::pair<double, double>>, 2> gains{};
    const auto redo_has = [&](std::size_t idx) { return std::ranges::find(redo_queue_, idx) != redo_queue_.end(); };
    for (std::size_t e = 0; e < 2; e++) {
        // the sample-level fit (direction + reported origin); the point fit below is the fallback
        ExtractOpts opts;
        opts.pupil = s.pattern == Pattern::Pimax;
        std::vector<Window> wins;
        for (const auto& r : cal_res) {
            wins.emplace_back(r.target, std::span<const RawSample>(r.samples));
        }
        const auto data = eye_data(wins, e, eye_pos_[e], opts, s.fix);
        if (auto fit = fit_eye(data, eye_pos_[e], opts)) {
            const em::ModelFit* bp = fit->selection.best();
            if (bp == nullptr) {
                continue;
            }
            const em::ModelFit best = *bp;
            for (const auto& m : fit->selection.candidates) {
                if (m.kind == em::ModelKind::Affine) {
                    gains[e] = std::pair{m.map.cx[1], m.map.cy[2]};
                    break;
                }
            }
            const double loo_med = std::max(best.loo.median, 0.5);
            for (const auto& [k, err] : fit->loo_by_window) {
                if (err > std::max(3.0 * loo_med, 2.5) && cal_res[k].attempts < 2 && !redo_has(target_index(k))) {
                    fx.push_back(Effect::print(std::format("{} eye: target {} is {:.1f} deg off the fit of the others: measuring it again",
                                                           eye_name(e), cal_res[k].target.name(), err)));
                    redo_queue_.push_back(target_index(k));
                }
            }
            fx.push_back(Effect::print(std::format(
                "{} eye: raw error {:.2f} deg -> {} model, leave-one-out {:.2f} deg (median {:.2f}, max {:.2f})", eye_name(e),
                fit->selection.raw.mean, model_name(*fit), best.loo.mean, best.loo.median, best.loo.max)));
            fits[e] = EyeFit{fit->selection, fit->points};
            eyes[e] = fit->cal;
            continue;
        }
        const auto pts = eye_points(e);
        std::vector<em::EyePoint> only;
        for (const auto& p : pts) {
            only.push_back(p.second);
        }
        if (only.size() < 4) {
            fx.push_back(Effect::print(std::format("{} eye: only {} usable targets, not calibrated", eye_name(e), only.size())));
            continue;
        }
        const auto sel = em::select_model(only);
        const em::ModelFit* bp = sel.best();
        if (bp == nullptr) {
            continue;
        }
        const em::ModelFit best = *bp;
        // the affine candidate tells the axis signs whatever model won
        for (const auto& m : sel.candidates) {
            if (m.kind == em::ModelKind::Affine) {
                gains[e] = std::pair{m.map.cx[1], m.map.cy[2]};
                break;
            }
        }
        // a target far off the fit of the others is re-measured once (the wearer looked elsewhere)
        const double loo_med = std::max(best.loo.median, 0.5);
        for (std::size_t j = 0; j < pts.size(); j++) {
            std::vector<em::EyePoint> rest;
            for (std::size_t m = 0; m < only.size(); m++) {
                if (m != j) {
                    rest.push_back(only[m]);
                }
            }
            const auto mm = em::fit_eye(rest, best.kind, best.ridge);
            if (!mm) {
                continue;
            }
            const double err = em::point_error_deg(*mm, pts[j].second);
            const std::size_t k = pts[j].first;
            if (err > std::max(3.0 * loo_med, 2.5) && cal_res[k].attempts < 2 && !redo_has(target_index(k))) {
                fx.push_back(Effect::print(std::format("{} eye: target {} is {:.1f} deg off the fit of the others: measuring it again",
                                                       eye_name(e), cal_res[k].target.name(), err)));
                redo_queue_.push_back(target_index(k));
            }
        }
        fx.push_back(Effect::print(std::format(
            "{} eye: raw error {:.2f} deg -> {} model, leave-one-out {:.2f} deg (median {:.2f}, max {:.2f})", eye_name(e),
            sel.raw.mean, em::model_name(best.kind), best.loo.mean, best.loo.median, best.loo.max)));
        std::vector<double> prec;
        for (const auto& r : cal_res) {
            if (r.eyes[e]) {
                prec.push_back(r.eyes[e]->rms_s2s_deg);
            }
        }
        em::EyeCal ec;
        ec.map = best.map;
        ec.origin_m = eye_pos_[e];
        ec.accuracy_deg = best.loo.mean;
        ec.precision_deg = prec.empty() ? 1.0 : median0(prec) / std::numbers::sqrt2;
        eyes[e] = ec;
        fits[e] = EyeFit{sel, only.size()};
    }
    // at most two rounds of re-measuring, whatever the fit says (no loop if a target never fits)
    if (redo_rounds_ >= 2) {
        redo_queue_.clear();
    }
    if (!redo_queue_.empty()) {
        redo_rounds_++;
        const std::size_t j = redo_queue_.front();
        redo_queue_.erase(redo_queue_.begin());
        in_redo_ = true;
        start_point(Set::Calibration, j, 2);
        return fx;
    }
    // mirrored axes: both eyes' gains negative
    const auto neg = [&](bool first) {
        return gains[0] && gains[1] && (first ? gains[0]->first : gains[0]->second) < 0.0 &&
               (first ? gains[1]->first : gains[1]->second) < 0.0;
    };
    const bool fx_neg = neg(true);
    const bool fy_neg = neg(false);
    if ((fx_neg || fy_neg) && axis_restarts_ == 0) {
        axis_restarts_++;
        em::FrameConv c = s.conv;
        if (fx_neg) {
            c.flip_x = !c.flip_x;
        }
        if (fy_neg) {
            c.flip_y = !c.flip_y;
        }
        fx.push_back(Effect::print(std::format(
            "the gaze moves opposite to the targets on {}: restarting the tracker with PIBRIDGE_EYE_FLIP_X={} FLIP_Y={} and calibrating again",
            fx_neg && fy_neg ? "both axes" : (fx_neg ? "the horizontal axis" : "the vertical axis"), c.flip_x ? 1 : 0,
            c.flip_y ? 1 : 0)));
        s.conv = c;
        fx.push_back(Effect::restart(c));
        const std::uint32_t restarts = axis_restarts_;
        const bool ready = chip_ready_;
        *this = Procedure(s);
        axis_restarts_ = restarts;
        chip_ready_ = ready;
        status_ = "AXIS FIXED: PRESS A TO CALIBRATE AGAIN";
        return fx;
    }
    if (!eyes[0] || !eyes[1]) {
        // one eye only: both slots get that eye's map so the file is complete, the missing one never validates
        const auto& one = eyes[0] ? eyes[0] : eyes[1];
        if (one) {
            em::EyeCal l = *one;
            em::EyeCal r = *one;
            l.origin_m = eye_pos_[0];
            r.origin_m = eye_pos_[1];
            calibration = make_calibration(l, r);
            fx.push_back(Effect::print("only one eye calibrated: check the headset fit"));
            start_point(Set::Validation, 0, 1);
        } else {
            status_ = "CALIBRATION FAILED: NO EYE SEEN. CHECK THE FIT, B TO RETRY";
            fx.push_back(Effect::print("calibration failed: no eye had enough usable targets"));
            phase = Phase::of(Phase::Kind::Results);
        }
        return fx;
    }
    calibration = make_calibration(*eyes[0], *eyes[1]);
    start_point(Set::Validation, 0, 1);
    return fx;
}

em::Calibration Procedure::make_calibration(em::EyeCal l, em::EyeCal r) const {
    em::Calibration c;
    c.conv = s.conv;
    c.eyes = {std::move(l), std::move(r)};
    c.combine = em::Combine{};
    c.filter = s.filter;
    c.created_unix = unix_now();
    return c;
}

std::vector<Effect> Procedure::finish_validation() {
    std::vector<Effect> fx;
    // per-eye validation errors become the combination weights
    if (calibration) {
        for (std::size_t e = 0; e < 2; e++) {
            std::vector<double> errs;
            for (const auto& v : val_res) {
                if (v.eval.eye_err_deg[e]) {
                    errs.push_back(*v.eval.eye_err_deg[e]);
                }
            }
            if (errs.size() >= 3) {
                calibration->eyes[e].accuracy_deg = em::stats(errs).mean;
            }
        }
    }
    const auto sum = summary(*this);
    if (calibration) {
        calibration->notes = sum.notes;
    }
    fx.push_back(Effect::print(sum.text));
    fx.push_back(Effect::with_text(Effect::Kind::SaveResults, json(*this)));
    if (s.pursuit) {
        pursuit_samples_.clear();
        phase = Phase::of(Phase::Kind::Pursuit, now_);
    } else {
        phase = Phase::of(Phase::Kind::Results);
    }
    return fx;
}

std::vector<Effect> Procedure::finish_recenter() {
    std::vector<Effect> fx;
    if (!s.existing) {
        status_ = "NO CALIBRATION TO RECENTRE";
        phase = Phase::of(Phase::Kind::Results);
        return fx;
    }
    em::Calibration c = *s.existing;
    for (std::size_t e = 0; e < 2; e++) {
        std::vector<double> du;
        std::vector<double> dv;
        for (const auto& r : rec_res) {
            if (!r.eyes[e]) {
                continue;
            }
            const auto truth = truth_tangent(r.target, c.eyes[e].origin_m);
            if (!truth) {
                continue;
            }
            const auto g = eye_center(c.eyes[e], *r.eyes[e]);
            if (!g) {
                continue;
            }
            du.push_back(truth->u - g->u);
            dv.push_back(truth->v - g->v);
        }
        if (du.size() >= 3) {
            const std::size_t n = du.size();
            const double mu = median0(du);
            const double mv = median0(dv);
            const double before = deg(std::atan(std::sqrt(mu * mu + mv * mv)));
            c.eyes[e].drift.u += mu;
            c.eyes[e].drift.v += mv;
            fx.push_back(Effect::print(std::format("{} eye: offset {:.2f} deg removed ({} targets)", eye_name(e), before, n)));
        } else {
            fx.push_back(Effect::print(std::format("{} eye: too few targets, unchanged", eye_name(e))));
        }
    }
    c.created_unix = unix_now();
    calibration = c;
    // the Pimax path's end screen says A / B itself
    status_ = grid_flow() ? std::string() : std::string("MENU / Y: SAVE THE RECENTRED CALIBRATION");
    phase = Phase::of(Phase::Kind::Results);
    return fx;
}

std::vector<Effect> Procedure::finish_service() {
    std::vector<double> errs;
    for (const auto& e : service_err) {
        if (e) {
            errs.push_back(*e);
        }
    }
    const auto st = em::stats(errs);
    status_ = errs.empty() ? std::string("RUNTIME CHECK: NO GAZE")
                           : std::format("RUNTIME CHECK: MEAN {:.2f} MAX {:.2f} DEG", st.mean, st.max);
    phase = Phase::of(Phase::Kind::ServiceCursor);
    return {Effect::print(std::format(
        "end-to-end through the runtime: {} of {} targets, error mean {:.2f} median {:.2f} max {:.2f} deg", errs.size(),
        service_err.size(), st.mean, st.median, st.max))};
}

std::vector<Effect> Procedure::finish_pursuit(double t0) {
    phase = Phase::of(Phase::Kind::Results);
    if (!calibration) {
        return {};
    }
    const em::Calibration c = *calibration;
    em::CombineState st;
    std::vector<std::pair<double, Tangent>> gaze;
    for (std::size_t i = 0; i < pursuit_samples_.size(); i++) {
        auto x = pursuit_samples_[i].to_sample(i);
        c.apply_sample(x, st);
        const auto d = gaze_dir(x);
        if (!d) {
            continue;
        }
        if (const auto t = em::tangent_of(*d)) {
            gaze.emplace_back(pursuit_samples_[i].t, *t);
        }
    }
    const double start = t0 + 1.0;
    const Vec3 m = mid(eye_pos_);
    const auto truth_at = [&](double t) { return truth_tangent(pursuit_at(std::max(t - start, 0.0)), m); };
    // drop catch-up saccades: gaze speed far above the target's 4.5 deg/s
    std::vector<bool> keep(gaze.size(), true);
    for (std::size_t i = 0; i < gaze.size(); i++) {
        const std::size_t a = i >= 2 ? i - 2 : 0;
        const std::size_t b = std::min(i + 2, gaze.size() - 1);
        const double dt = gaze[b].first - gaze[a].first;
        keep[i] = dt <= 0.0 || em::tangent_angle_deg(gaze[a].second, gaze[b].second) / dt < 40.0;
    }
    std::optional<std::pair<double, em::Stats>> best;
    for (int k = 0; k <= 25; k++) {
        const double lag = k * 0.01;
        std::vector<double> errs;
        for (std::size_t i = 0; i < gaze.size(); i++) {
            if (!keep[i]) {
                continue;
            }
            if (const auto tr = truth_at(gaze[i].first - lag)) {
                errs.push_back(em::tangent_angle_deg(gaze[i].second, *tr));
            }
        }
        const auto sst = em::stats(errs);
        if (!best || sst.mean < best->second.mean) {
            best = std::pair{lag, sst};
        }
    }
    double used = 0.0;
    if (!pursuit_samples_.empty()) {
        used = static_cast<double>(std::ranges::count(keep, true)) / static_cast<double>(pursuit_samples_.size());
    }
    if (!best) {
        return {};
    }
    pursuit_res = PursuitRes{best->first, best->second, used};
    const auto& err = best->second;
    return {Effect::print(std::format(
                "smooth pursuit: error mean {:.2f} median {:.2f} p95 {:.2f} deg at lag {:.0f} ms ({:.0f}% of samples)",
                err.mean, err.median, err.p95, best->first * 1000.0, 100.0 * used)),
            Effect::with_text(Effect::Kind::SaveResults, json(*this))};
}

void Procedure::update_cursor(const std::vector<RawSample>& samples) {
    if (samples.empty()) {
        if (recent_.empty() || recent_.back().t < now_ - 0.3) {
            cursor_.reset();
            raw_cursor_.reset();
            gaze_.reset();
        }
        return;
    }
    // the check grid's gaze: what pibridge-eyed publishes, the raw samples through its host map (identity without one)
    // and its filter (live.cpp Pipeline::push)
    {
        em::Calibration ident = em::Calibration::identity(s.conv, 0.064);
        for (std::size_t e = 0; e < 2; e++) {
            ident.eyes[e].origin_m = eye_pos_[e];
        }
        const em::Calibration& vc = s.view ? *s.view : ident;
        em::FilterParams f;
        f.enabled = false;
        if (s.view_filter) {
            f = *s.view_filter;
        } else if (s.view) {
            f = s.view->filter;
        }
        for (const auto& r : samples) {
            auto y = r.to_sample(0);
            vc.apply_sample(y, gaze_combine_);
            em::filter_sample(f, gaze_filter_, y);
            gaze_ = gaze_dir(y);
        }
    }
    // raw combined (identity maps), for the comparison
    em::Calibration raw = em::Calibration::identity(s.conv, 0.064);
    for (std::size_t e = 0; e < 2; e++) {
        raw.eyes[e].origin_m = eye_pos_[e];
    }
    em::CombineState rs;
    auto x = samples.back().to_sample(0);
    raw.apply_sample(x, rs);
    raw_cursor_ = gaze_dir(x);
    if (!calibration) {
        cursor_.reset();
        return;
    }
    for (const auto& r : samples) {
        auto y = r.to_sample(0);
        calibration->apply_sample(y, cursor_combine_);
        em::filter_sample(calibration->filter, cursor_filter_, y);
        cursor_ = gaze_dir(y);
    }
}

// ------------------------------------------------------------------------------------------------ on-chip mode

Effect Procedure::send(const cal::Step& st) {
    chip_pending_++;
    return Effect::chip(st);
}

void Procedure::to_grid(std::string status) {
    window_.clear();
    status_ = std::move(status);
    phase = Phase::of(grid_saved_ ? Phase::Kind::GridSaved : Phase::Kind::Grid, now_);
}

std::vector<Effect> Procedure::save_and_grid() {
    std::vector<Effect> fx = accept();
    if (s.mode == Mode::Chip) {
        fx.push_back(Effect::of(Effect::Kind::KeepProfile));
    } else {
        s.existing = *calibration; // a recentre again builds on this one
    }
    // the grid's gaze goes through the new host map from now on (the chip has used its new calibration since its 0x42e)
    s.view = *calibration;
    gaze_combine_ = em::CombineState{};
    gaze_filter_ = em::OneEuro{};
    grid_saved_ = true;
    fx.push_back(Effect::print("saved: back to the check grid, the gaze dot uses the new calibration"));
    to_grid("SAVED: THE DOT USES THE NEW CALIBRATION");
    return fx;
}

std::vector<Effect> Procedure::grid_command(Cmd c) {
    std::vector<Effect> fx;
    if (c == Cmd::Redo) {
        fx.push_back(Effect::print("exit from the check grid"));
        fx.push_back(Effect::of(Effect::Kind::Quit));
        return fx;
    }
    if (c != Cmd::Next && c != Cmd::Start) {
        return fx;
    }
    if (s.mode == Mode::Recenter) {
        reset_keep_session();
        fx.push_back(Effect::print("recentre: 5 targets"));
        start_point(Set::Recenter, 0, 1);
        return fx;
    }
    if (!chip_ready_) {
        status_ = "WAITING FOR THE EYE TRACKER";
        return fx;
    }
    if (chip_pending_ > 0) {
        // the answers of a stopped run are still out: a new run's answers must not meet them
        status_ = "PLEASE WAIT";
        return fx;
    }
    reset_keep_session();
    fx.push_back(Effect::print(std::format(
        "on-chip calibration: {} targets in two passes, then {} validation targets; keep the head still", chip_t_.size(),
        recenter_targets().size())));
    auto more = chip_advance();
    fx.insert(fx.end(), more.begin(), more.end());
    return fx;
}

std::vector<Effect> Procedure::recenter_command(Cmd c) {
    using K = Phase::Kind;
    std::vector<Effect> fx;
    if (phase.is(K::Grid) || phase.is(K::GridSaved)) {
        return grid_command(c);
    }
    if (phase.is(K::Results) || phase.is(K::Cursor)) {
        if (c == Cmd::Next) {
            fx.push_back(Effect::print("recentring again"));
            auto more = grid_command(Cmd::Next);
            fx.insert(fx.end(), more.begin(), more.end());
        } else if (c == Cmd::Redo || c == Cmd::Accept) {
            if (calibration) {
                fx = save_and_grid();
            } else {
                to_grid("NOT SAVED");
            }
        }
        return fx;
    }
    // the targets: B goes back to the grid, nothing else
    if (c == Cmd::Redo) {
        fx.push_back(Effect::print("recentre stopped: nothing saved"));
        to_grid("STOPPED: NOT SAVED");
    }
    return fx;
}

std::vector<Effect> Procedure::chip_command(Cmd c) {
    std::vector<Effect> fx;
    using K = Phase::Kind;
    const auto k = phase.kind;
    if (k == K::Grid || k == K::GridSaved || k == K::Intro) {
        return grid_command(c);
    }
    if (k == K::Results || k == K::Cursor) {
        if (c == Cmd::Next) {
            fx.push_back(Effect::print("calibrating again"));
            auto more = grid_command(Cmd::Next);
            fx.insert(fx.end(), more.begin(), more.end());
        } else if (c == Cmd::Redo || c == Cmd::Accept) {
            if (chip_profile && calibration) {
                fx = save_and_grid();
            } else {
                fx.push_back(Effect::print("nothing to save: back to the check grid"));
                to_grid("NOT SAVED");
            }
        }
        return fx;
    }
    // running: only B, which stops (the open pass is finished) and goes back to the grid
    if (c == Cmd::Redo) {
        fx = shutdown();
        fx.push_back(Effect::print("calibration stopped: nothing saved, back to the check grid"));
        to_grid("STOPPED: NOT SAVED");
    }
    return fx;
}

void Procedure::reset_keep_session() {
    const bool ready = chip_ready_;
    const bool grid_saved = grid_saved_;
    const std::uint32_t pending = chip_pending_;
    const double now = now_;
    *this = Procedure(s);
    chip_ready_ = ready;
    grid_saved_ = grid_saved;
    chip_pending_ = pending;
    now_ = now;
    if (grid_saved_) {
        phase = Phase::of(Phase::Kind::GridSaved, now_);
    }
}

std::vector<Effect> Procedure::chip_advance() {
    std::vector<Effect> fx;
    if (const cal::Step* st = chip_run_.current()) {
        if (st->op == cal::OP_TARGET && st->target) {
            start_point(Set::Chip, *st->target, 1);
            return fx;
        }
        if (st->op == cal::OP_START) {
            fx.push_back(Effect::print(std::format("pass {}: {} targets", st->pass,
                                                   st->pass == 1 ? cal::PASS1_TARGETS.size() : cal::PASS2_TARGETS.size())));
        }
        chip_inflight_ = *st;
        chip_sent_at_ = now_;
        fx.push_back(send(*st));
        phase = Phase::of(Phase::Kind::ChipBusy, now_);
        return fx;
    }
    if (!chip_run_.done() || !chip_run_.profile()) {
        return chip_fail(chip_run_.failed() ? chip_run_.error() : std::string("the calibration ended without a profile"));
    }
    chip_profile = *chip_run_.profile();
    std::string sizes;
    for (const auto n : chip_run_.download_sizes()) {
        sizes += (sizes.empty() ? "" : ", ") + std::to_string(n);
    }
    fx.push_back(Effect::print(std::format("on-chip calibration done: the new profile has {} bytes (downloads: {} bytes)",
                                           chip_profile->size(), sizes)));
    fx.push_back(Effect::save_profile(*chip_profile));
    fx.push_back(Effect::print("validation: 5 targets with the new chip calibration"));
    start_point(Set::Recenter, 0, 1);
    return fx;
}

std::vector<Effect> Procedure::chip_answer(std::uint32_t seq, const std::expected<cal::Answer, std::string>& r) {
    std::vector<Effect> fx;
    if (chip_pending_ > 0) {
        chip_pending_--;
    }
    if (chip_pending_ == 0 && status_ == "PLEASE WAIT") {
        status_.clear();
    }
    if (!chip_inflight_) {
        return fx;
    }
    const cal::Step st = *chip_inflight_;
    chip_inflight_.reset();
    if (chip_aborting_) {
        chip_aborting_ = false;
        fx.push_back(Effect::print(r ? std::string("the open calibration pass was finished (0x3fc)")
                                     : "finishing the open calibration pass: " + r.error()));
        return fx;
    }
    if (!r) {
        return chip_fail(r.error());
    }
    if (auto a = chip_run_.answer(seq, *r); !a) {
        return chip_fail(a.error());
    }
    const double ms = (now_ - chip_sent_at_) * 1000.0;
    if (st.op == cal::OP_TARGET && phase.is(Phase::Kind::Point) && phase.set == Set::Chip) {
        const std::size_t i = phase.i;
        PointRes res;
        res.target = chip_t_[i];
        res.eyes = {analyze_eye(window_, 0, s.fix), analyze_eye(window_, 1, s.fix)};
        res.ok = std::ranges::any_of(res.eyes, [](const auto& e) { return e.has_value(); });
        res.samples = window_;
        res.head_deg = head_max_;
        res.attempts = phase.attempt;
        const auto eye = [&](std::size_t e) {
            const auto truth = truth_tangent(res.target, eye_pos_[e]);
            if (!res.eyes[e] || !truth) {
                return std::string("no fixation");
            }
            return std::format("raw gaze {:.1f} deg off", em::tangent_angle_deg(res.eyes[e]->center, *truth));
        };
        fx.push_back(Effect::print(std::format("chip target {}/{} ({}): collected after {:.0f} ms; left {}, right {}", i + 1,
                                               chip_t_.size(), res.target.name(), ms, eye(0), eye(1))));
        chip_res.push_back(std::move(res));
        phase = Phase::point(Set::Chip, i, Stage::Accepted, now_, phase.attempt);
        return fx;
    }
    switch (st.op) {
    case cal::OP_START: break;
    case cal::OP_COMPUTE:
        fx.push_back(Effect::print(std::format("pass {}: computed after {:.0f} ms (the chip uses the new calibration now)",
                                               st.pass, ms)));
        break;
    case cal::OP_DOWNLOAD:
        fx.push_back(Effect::print(std::format("pass {}: profile downloaded ({} bytes)", st.pass, r->payload.size())));
        break;
    case cal::OP_FINISH: fx.push_back(Effect::print(std::format("pass {}: finished", st.pass))); break;
    default: break;
    }
    auto more = chip_advance();
    fx.insert(fx.end(), more.begin(), more.end());
    return fx;
}

std::vector<Effect> Procedure::chip_fail(const std::string& why) {
    std::vector<Effect> fx;
    chip_failed = true;
    status_ = "CALIBRATION FAILED: " + upper(why);
    fx.push_back(Effect::print("on-chip calibration failed: " + why + " (nothing saved, the previous profile stays)"));
    if (const auto ab = chip_run_.abort_step(); ab && !chip_inflight_) {
        chip_inflight_ = *ab;
        chip_aborting_ = true;
        chip_sent_at_ = now_;
        fx.push_back(send(*ab));
    }
    window_.clear();
    phase = Phase::of(Phase::Kind::Results);
    return fx;
}

std::vector<Effect> Procedure::shutdown() {
    if (s.mode != Mode::Chip || chip_aborting_) {
        return {};
    }
    std::optional<cal::Step> ab = chip_run_.abort_step();
    if (!ab && chip_inflight_ && chip_inflight_->op == cal::OP_START) {
        ab = cal::Step{cal::OP_FINISH, std::nullopt, chip_inflight_->pass};
    }
    if (!ab) {
        return {};
    }
    chip_inflight_ = *ab;
    chip_aborting_ = true;
    chip_sent_at_ = now_;
    return {send(*ab)};
}

ChipCheck check_chip_validation(const std::vector<PointRes>& windows, const std::array<Vec3, 2>& eye_pos) {
    ChipCheck out;
    for (std::size_t e = 0; e < 2; e++) {
        em::EyeCal ident;
        ident.origin_m = eye_pos[e];
        std::vector<std::pair<Tangent, Tangent>> pts; // (gaze, truth)
        for (const auto& r : windows) {
            if (!r.eyes[e]) {
                continue;
            }
            const auto truth = truth_tangent(r.target, eye_pos[e]);
            const auto g = eye_center(ident, *r.eyes[e]);
            if (truth && g) {
                pts.emplace_back(*g, *truth);
            }
        }
        if (pts.size() < 3) {
            continue;
        }
        ChipEyeCheck c;
        c.n = pts.size();
        std::vector<double> du;
        std::vector<double> dv;
        double before = 0.0;
        for (const auto& [g, t] : pts) {
            du.push_back(t.u - g.u);
            dv.push_back(t.v - g.v);
            before += em::tangent_angle_deg(g, t);
        }
        c.offset = Tangent{median0(du), median0(dv)};
        c.offset_deg = deg(std::atan(std::hypot(c.offset.u, c.offset.v)));
        double after = 0.0;
        for (const auto& [g, t] : pts) {
            after += em::tangent_angle_deg(Tangent{g.u + c.offset.u, g.v + c.offset.v}, t);
        }
        c.before_deg = before / static_cast<double>(pts.size());
        c.after_deg = after / static_cast<double>(pts.size());
        // the gaze against the targets, per axis: a negative slope is a mirrored axis
        const auto slope = [&](bool x) -> std::optional<double> {
            double mt = 0.0;
            double mg = 0.0;
            for (const auto& [g, t] : pts) {
                mt += x ? t.u : t.v;
                mg += x ? g.u : g.v;
            }
            mt /= static_cast<double>(pts.size());
            mg /= static_cast<double>(pts.size());
            double cov = 0.0;
            double var = 0.0;
            for (const auto& [g, t] : pts) {
                const double a = (x ? t.u : t.v) - mt;
                cov += a * ((x ? g.u : g.v) - mg);
                var += a * a;
            }
            if (var < 1e-4) {
                return std::nullopt;
            }
            return cov / var;
        };
        c.mirrored_x = slope(true).value_or(1.0) < 0.0;
        c.mirrored_y = slope(false).value_or(1.0) < 0.0;
        c.apply = !c.mirrored_x && !c.mirrored_y && c.offset_deg >= 0.5 &&
                  (c.after_deg <= 0.5 * c.before_deg || c.after_deg <= 1.0);
        out.eyes[e] = c;
    }
    return out;
}

std::vector<Effect> Procedure::finish_chip_validation() {
    std::vector<Effect> fx;
    const ChipCheck chk = check_chip_validation(rec_res, eye_pos_);
    em::Calibration c = em::Calibration::identity(s.conv, eye_pos_[1][0] - eye_pos_[0][0]);
    c.filter = s.filter;
    c.created_unix = unix_now();
    chip_lines_.clear();
    std::vector<std::string> notes;
    notes.push_back(std::format("on-chip calibration {}: the chip's own calibration (profile {} bytes); host map = identity per eye",
                                stamp(unix_now()), chip_profile ? chip_profile->size() : 0));
    for (std::size_t e = 0; e < 2; e++) {
        c.eyes[e].origin_m = eye_pos_[e];
        const auto& x = chk.eyes[e];
        if (!x) {
            const std::string l = std::format("{} eye: too few validation targets, no offset", eye_name(e));
            fx.push_back(Effect::print(l));
            notes.push_back(l);
            continue;
        }
        const auto [ou, ov] = tangent_deg(x->offset);
        std::string l = std::format("{} eye: {:.2f} deg off the targets ({} targets)", eye_name(e), x->before_deg, x->n);
        if (x->apply) {
            c.eyes[e].drift = x->offset;
            c.eyes[e].accuracy_deg = x->after_deg;
            l += std::format("; constant offset ({:+.2f}, {:+.2f}) deg removed: {:.2f} deg left", ou, ov, x->after_deg);
        } else {
            c.eyes[e].accuracy_deg = x->before_deg;
            l += std::format("; no constant offset to remove (offset {:.2f} deg, {:.2f} deg without it)", x->offset_deg,
                             x->after_deg);
        }
        fx.push_back(Effect::print(l));
        notes.push_back(l);
        if (x->mirrored_x || x->mirrored_y) {
            fx.push_back(Effect::print(std::format(
                "{} eye: the gaze moves against the targets {}: the chip's targets may have been shown mirrored "
                "(try --chip-mirror {})",
                eye_name(e), x->mirrored_x && x->mirrored_y ? "on both axes" : (x->mirrored_x ? "horizontally" : "vertically"),
                x->mirrored_x && x->mirrored_y ? "xy" : (x->mirrored_x ? "x" : "y"))));
        }
    }
    c.notes = notes;
    calibration = c;
    chip_check = chk;
    // the end screen
    chip_lines_.push_back("CHIP CALIBRATION DONE");
    const auto eye_words = [&](std::size_t e) {
        const auto& x = chk.eyes[e];
        return x ? std::format("{:.1f} DEG", x->apply ? x->after_deg : x->before_deg) : std::string("-");
    };
    chip_lines_.push_back(std::format("ACCURACY LEFT {}  RIGHT {}", eye_words(0), eye_words(1)));
    const auto off_words = [&](std::size_t e) {
        const auto& x = chk.eyes[e];
        return x && x->apply ? std::format("{:.1f}", x->offset_deg) : std::string("-");
    };
    chip_lines_.push_back(std::format("OFFSET REMOVED LEFT {}  RIGHT {}", off_words(0), off_words(1)));
    if (std::ranges::any_of(chk.eyes, [](const auto& x) { return x && (x->mirrored_x || x->mirrored_y); })) {
        chip_lines_.push_back("THE DOTS LOOK MIRRORED: SEE THE TERMINAL");
    }
    fx.push_back(Effect::with_text(Effect::Kind::SaveResults, json(*this)));
    phase = Phase::of(Phase::Kind::Results);
    return fx;
}

// ------------------------------------------------------------------------------------------------ scene

Scene Procedure::scene() const {
    Scene sc;
    auto& marks = sc.marks;
    auto& hud = sc.hud;
    sc.background = BACKGROUND;
    const auto at = [](const Vec3& d) { return scaled(d, TARGET_DIST_M); };
    const auto ratio = eye_ratio();
    const std::string eye_status = std::format("EYES SEEN: LEFT {:.0f}%  RIGHT {:.0f}%", 100.0 * ratio[0], 100.0 * ratio[1]);
    using K = Phase::Kind;
    const bool pimax_look = s.pattern == Pattern::Pimax;
    switch (phase.kind) {
    case K::Grid:
    case K::GridSaved: {
        // the check grid: what pibridge-eyed's gaze looks like now (after a save: with the new calibration)
        sc.grid = true;
        if (gaze_) {
            sc.gaze = at(*gaze_);
        }
        if (!status_.empty()) {
            hud.push_back(status_);
        } else if (s.mode == Mode::Chip && !chip_ready_) {
            hud.push_back("WAITING FOR THE EYE TRACKER");
        } else if (!gaze_) {
            hud.push_back("NO GAZE");
        }
        const bool saved_grid = phase.is(K::GridSaved);
        if (s.mode == Mode::Chip) {
            hud.push_back(saved_grid ? "A: CALIBRATE AGAIN    B: EXIT" : "A: CALIBRATE    B: EXIT");
        } else {
            hud.push_back(saved_grid ? "A: RECENTER AGAIN    B: EXIT" : "A: RECENTER    B: EXIT");
        }
        break;
    }
    case K::Intro:
        abc(marks, at({0.0, 0.0, -1.0}), WHITE);
        hud.push_back(s.mode == Mode::Recenter ? "EYE RECENTRE" : "EYE CALIBRATION");
        hud.push_back("KEEP YOUR HEAD STILL");
        hud.push_back("FOLLOW THE DOTS WITH YOUR EYES ONLY");
        hud.push_back(eye_status);
        if (!status_.empty()) {
            hud.push_back(status_);
        } else if (s.mode == Mode::Chip && !chip_ready_) {
            hud.push_back("WAITING FOR THE EYE TRACKER");
        }
        hud.push_back(pimax_look ? "A: START   B: EXIT" : "A / SPACE: START");
        break;
    case K::Point: {
        const Set set = phase.set;
        const std::size_t i = phase.i;
        const Stage stage = phase.stage;
        const double dt = now_ - phase.t0;
        if (pimax_look && (set == Set::Calibration || set == Set::Chip)) {
            // Pimax Play's look: a green dot that travels to the next place, black or white background
            const Target tg = target(set, i);
            if (tg.bright) {
                sc.background = PIMAX_WHITE;
            }
            const Rgb dot = tg.bright ? PIMAX_GREEN_ON_WHITE : GREEN;
            Vec3 p = tg.point();
            if (stage == Stage::Attract && i >= 1) {
                const Target from_t = target(set, i - 1);
                const Vec3 from = from_t.dir();
                const double f = smoothstep(std::clamp(dt / s.attract_s, 0.0, 1.0));
                const Vec3 to = tg.dir();
                Vec3 d{};
                for (std::size_t k = 0; k < 3; k++) {
                    d[k] = from[k] + (to[k] - from[k]) * f;
                }
                const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                const double dist = from_t.dist_m + (tg.dist_m - from_t.dist_m) * f;
                p = scaled(d, dist / n);
            }
            if (stage == Stage::HeadMoved) {
                abc_on(marks, p, AMBER, sc.background);
                hud.push_back("HEAD MOVED: KEEP IT STILL");
            } else if (stage == Stage::Accepted) {
                abc_on(marks, p, DIM, sc.background);
            } else {
                abc_on(marks, p, dot, sc.background);
            }
            break;
        }
        const Target tg = target(set, i);
        const Vec3 p = tg.point();
        switch (stage) {
        case Stage::Attract: {
            const double f = std::clamp(dt / s.attract_s, 0.0, 1.0);
            marks.push_back(Mark{p, MarkKind::Ring, 3.0 + (0.75 - 3.0) * f, 0.15, GREY});
            abc(marks, p, WHITE);
            break;
        }
        case Stage::Hold:
        case Stage::Collect: abc(marks, p, WHITE); break;
        case Stage::Accepted: abc(marks, p, GREEN); break;
        case Stage::HeadMoved:
            abc(marks, p, AMBER);
            hud.push_back("HEAD MOVED: KEEP IT STILL");
            hud.push_back("ONLY YOUR EYES FOLLOW THE DOT");
            break;
        }
        break;
    }
    case K::ChipBusy: {
        // the dot stays where the eyes are while the chip starts, computes or sends the profile
        const Target tg = chip_last_target_ ? chip_t_[*chip_last_target_] : (chip_t_.empty() ? Target{} : chip_t_[0]);
        if (chip_last_target_ && tg.bright) {
            sc.background = PIMAX_WHITE;
        }
        abc_on(marks, tg.point(), chip_last_target_ ? DIM : GREEN, sc.background);
        if (now_ - phase.t0 > 0.6) {
            hud.push_back("PLEASE WAIT");
        }
        break;
    }
    case K::Pursuit: {
        const Target tg = pursuit_at(std::max(now_ - phase.t0 - 1.0, 0.0));
        abc(marks, tg.point(), WHITE);
        if (now_ - phase.t0 < 1.0) {
            hud.push_back("FOLLOW THE MOVING DOT");
        }
        break;
    }
    case K::Results:
    case K::Cursor:
        if (s.mode == Mode::Chip) {
            hud.insert(hud.end(), chip_lines_.begin(), chip_lines_.end());
            if (!status_.empty()) {
                hud.push_back(status_);
            }
            hud.push_back("");
            hud.push_back(chip_profile && calibration ? "A: AGAIN    B: SAVE" : "A: AGAIN    B: BACK");
            break;
        }
        if (grid_flow()) {
            const auto sum = summary(*this);
            hud.insert(hud.end(), sum.hud.begin(), sum.hud.end());
            if (!status_.empty()) {
                hud.push_back(status_);
            }
            hud.push_back("");
            hud.push_back(calibration ? "A: AGAIN    B: SAVE" : "A: AGAIN    B: BACK");
            break;
        }
        if (pimax_look) {
            const auto sum = summary(*this);
            hud.insert(hud.end(), sum.hud.begin(), sum.hud.end());
            hud.push_back("");
            hud.push_back("A: CALIBRATE AGAIN");
            hud.push_back(calibration ? "B: SAVE AND EXIT" : "B: EXIT");
            break;
        }
        {
            const auto sum = summary(*this);
            hud.insert(hud.end(), sum.hud.begin(), sum.hud.end());
            if (!status_.empty()) {
                hud.push_back(status_);
            }
            if (phase.is(K::Cursor)) {
                auto ts = val_t_;
                ts.push_back(Target::at(0.0, 0.0));
                for (const auto& t : ts) {
                    marks.push_back(Mark{t.point(), MarkKind::Disc, 0.25, 0.0, GREY});
                }
                if (raw_cursor_) {
                    ring(marks, at(*raw_cursor_), 0.6, {0.4F, 0.4F, 0.45F});
                }
                if (cursor_) {
                    ring(marks, at(*cursor_), 0.8, GREEN);
                }
                hud.push_back("GREEN: CALIBRATED  GREY: RAW");
                hud.push_back("A: RESULTS  MENU/Y: SAVE  S: RUNTIME CHECK");
            } else {
                hud.push_back(calibration ? "A: LIVE CURSOR  MENU/Y: SAVE  P: PURSUIT  B: REDO" : "B: START OVER");
            }
        }
        break;
    case K::ServiceWait:
        abc(marks, at({0.0, 0.0, -1.0}), WHITE);
        hud.push_back("STARTING THE RUNTIME'S EYE TRACKING");
        break;
    case K::ServiceCursor:
        for (const auto& t : service_targets()) {
            marks.push_back(Mark{t.point(), MarkKind::Disc, 0.25, 0.0, GREY});
        }
        if (service_cursor_) {
            ring(marks, at(*service_cursor_), 0.8, {0.3F, 0.8F, 1.0F});
        }
        hud.push_back("BLUE: GAZE FROM THE RUNTIME (WHAT APPS GET)");
        if (!status_.empty()) {
            hud.push_back(status_);
        }
        hud.push_back("A: BACK");
        break;
    }
    return sc;
}

bool Procedure::measuring() const {
    return phase.is(Phase::Kind::Point) || phase.is(Phase::Kind::Pursuit) || phase.is(Phase::Kind::ChipBusy);
}

std::vector<std::pair<Region, std::vector<double>>> region_errors(const Procedure& p) {
    std::vector<std::pair<Region, std::vector<double>>> out;
    for (const auto r : ALL_REGIONS) {
        std::vector<double> v;
        for (const auto& x : p.val_res) {
            if (region_of(x.point.target) == r && x.eval.comb_err_deg) {
                v.push_back(*x.eval.comb_err_deg);
            }
        }
        out.emplace_back(r, std::move(v));
    }
    return out;
}

} // namespace pibridge::eyecal
