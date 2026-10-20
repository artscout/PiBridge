// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eyecal/src/report.rs (MIT OR Apache-2.0); stamp() and write_with_backup() from
// pimax-lenstest/src/gaze.rs.
#include "report.h"

#include "procedure.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <format>
#include <fstream>
#include <system_error>

namespace pibridge::eyecal {

namespace fs = std::filesystem;

std::uint64_t unix_now() {
    const auto d = std::chrono::system_clock::now().time_since_epoch();
    const auto s = std::chrono::duration_cast<std::chrono::seconds>(d).count();
    return s < 0 ? 0 : static_cast<std::uint64_t>(s);
}

std::string stamp(std::uint64_t unix) {
    const auto days = static_cast<std::int64_t>(unix / 86400);
    const std::uint64_t rem = unix % 86400;
    // civil from days (Howard Hinnant)
    const std::int64_t z = days + 719'468;
    const std::int64_t era = (z >= 0 ? z : z - 146'096) / 146'097;
    const std::int64_t doe = z - era * 146'097;
    const std::int64_t yoe = (doe - doe / 1460 + doe / 36'524 - doe / 146'096) / 365;
    const std::int64_t y0 = yoe + era * 400;
    const std::int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const std::int64_t mp = (5 * doy + 2) / 153;
    const std::int64_t d = doy - (153 * mp + 2) / 5 + 1;
    const std::int64_t m = mp < 10 ? mp + 3 : mp - 9;
    const std::int64_t y = m <= 2 ? y0 + 1 : y0;
    return std::format("{:04}{:02}{:02}-{:02}{:02}{:02}", y, m, d, rem / 3600, rem % 3600 / 60, rem % 60);
}

std::expected<std::optional<fs::path>, std::string> write_with_backup(const fs::path& path, std::string_view text,
                                                                      std::string_view st) {
    std::error_code ec;
    if (path.has_parent_path()) {
        fs::create_directories(path.parent_path(), ec);
        if (ec) {
            return std::unexpected(std::format("{}: {}", path.parent_path().string(), ec.message()));
        }
    }
    std::optional<fs::path> backup;
    if (fs::exists(path, ec)) {
        const std::string name = path.filename().string();
        fs::path b = path.parent_path() / std::format("{}.bak-{}", name, st);
        for (int n = 1; fs::exists(b, ec); n++) {
            b = path.parent_path() / std::format("{}.bak-{}-{}", name, st, n);
        }
        fs::copy_file(path, b, ec);
        if (ec) {
            return std::unexpected(std::format("backup {}: {}", b.string(), ec.message()));
        }
        backup = b;
    }
    fs::path tmp = path;
    tmp.replace_extension("tmp-write");
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        f.write(text.data(), static_cast<std::streamsize>(text.size()));
        if (!f) {
            return std::unexpected(std::format("{}: cannot be written", tmp.string()));
        }
    }
    fs::rename(tmp, path, ec);
    if (ec) {
        return std::unexpected(std::format("{}: {}", path.string(), ec.message()));
    }
    return backup;
}

std::expected<std::optional<fs::path>, std::string> write_with_backup(const fs::path& path, std::string_view text) {
    return write_with_backup(path, text, stamp(unix_now()));
}

fs::path results_dir() {
    const char* h = std::getenv("HOME");
    const fs::path home = h != nullptr && *h != 0 ? fs::path(h) : fs::path(".");
    return home / ".local/share/pibridge/eye-calibration";
}

namespace {

/// Median of the upper middle element, 0 for an empty list (report.rs).
double median_hi(std::vector<double> v) {
    std::ranges::sort(v, [](double a, double b) { return std::strong_order(a, b) < 0; });
    return v.empty() ? 0.0 : v[v.size() / 2];
}

std::string jnum(double x) { return std::isfinite(x) ? std::format("{:.4f}", x) : std::string("null"); }

std::string jopt(std::optional<double> x) { return x ? jnum(*x) : std::string("null"); }

std::string jstats(const pibridge::eyemath::Stats& s) {
    return std::format("{{\"n\": {}, \"mean\": {}, \"median\": {}, \"p95\": {}, \"max\": {}}}", s.n, jnum(s.mean),
                       jnum(s.median), jnum(s.p95), jnum(s.max));
}

std::string jtarget(const Target& t) { return std::format("\"az\": {}, \"el\": {}", jnum(t.az), jnum(t.el)); }

std::string jstr(std::string_view s) {
    std::string o = "\"";
    for (const char c : s) {
        if (c == '"') {
            o += "\\\"";
        } else if (c == '\\') {
            o += "\\\\";
        } else if (c == '\n') {
            o += "\\n";
        } else if (static_cast<unsigned char>(c) < 0x20) {
            o += std::format("\\u{:04x}", static_cast<unsigned>(c));
        } else {
            o.push_back(c);
        }
    }
    o.push_back('"');
    return o;
}

std::string jeye(const std::optional<EyeWindow>& w) {
    if (!w) {
        return "null";
    }
    return std::format("{{\"total\": {}, \"valid\": {}, \"used\": {}, \"fix_s\": {}, \"raw_u\": {:.6f}, \"raw_v\": {:.6f}, "
                       "\"mad_deg\": {}, \"rms_s2s_deg\": {}, \"sd_deg\": {}}}",
                       w->total, w->valid, w->used, jnum(w->fix_s), w->center.u, w->center.v, jnum(w->mad_deg),
                       jnum(w->rms_s2s_deg), jnum(w->sd_deg));
}

std::string join(const std::vector<std::string>& v, std::string_view sep) {
    std::string o;
    for (std::size_t i = 0; i < v.size(); i++) {
        if (i > 0) {
            o += sep;
        }
        o += v[i];
    }
    return o;
}

std::string six(const std::array<double, 6>& a) {
    std::vector<std::string> v;
    for (const double x : a) {
        v.push_back(std::format("{:.6f}", x));
    }
    return join(v, ", ");
}

std::string three(const pibridge::eyemath::Vec3& a) {
    std::vector<std::string> v;
    for (const double x : a) {
        v.push_back(std::format("{:.6f}", x));
    }
    return join(v, ", ");
}

} // namespace

Summary summary(const Procedure& p) {
    namespace em = pibridge::eyemath;
    Summary s;
    if (p.calibration && p.val_res.empty() && p.rec_res.empty()) {
        s.hud.push_back("CALIBRATED, NOT VALIDATED");
    }
    if (p.val_res.empty()) {
        if (!p.rec_res.empty()) {
            s.hud.push_back("RECENTRE MEASURED");
            s.text = "recentre measured";
        } else if (!p.calibration) {
            s.hud.push_back("NO CALIBRATION");
        }
        return s;
    }
    std::vector<double> comb;
    std::vector<double> raw;
    std::vector<double> pooled;
    std::vector<double> prec;
    std::vector<double> sd;
    double loss = 0.0;
    for (const auto& v : p.val_res) {
        if (v.eval.comb_err_deg) {
            comb.push_back(*v.eval.comb_err_deg);
            prec.push_back(v.eval.comb_rms_s2s_deg);
            sd.push_back(v.eval.comb_sd_deg);
        }
        if (v.raw_eval.comb_err_deg) {
            raw.push_back(*v.raw_eval.comb_err_deg);
        }
        pooled.insert(pooled.end(), v.eval.sample_errs.begin(), v.eval.sample_errs.end());
        loss += v.eval.loss;
    }
    loss /= static_cast<double>(p.val_res.size());
    s.accuracy = em::stats(comb);
    s.raw = em::stats(raw);
    const auto ps = em::stats(pooled);
    const auto regions = region_errors(p);
    const auto reg = [&](Region r) {
        for (const auto& x : regions) {
            if (x.first == r && !x.second.empty()) {
                return std::format("{:.2f}", em::stats(x.second).mean);
            }
        }
        return std::string("-");
    };
    const auto per_eye = [&](std::size_t e) {
        std::vector<double> v;
        for (const auto& x : p.val_res) {
            if (x.eval.eye_err_deg[e]) {
                v.push_back(*x.eval.eye_err_deg[e]);
            }
        }
        return v.empty() ? std::string("-") : std::format("{:.2f}", em::stats(v).mean);
    };
    const auto& a = s.accuracy;
    s.text += std::format("validation ({} of {} targets): accuracy mean {:.2f} median {:.2f} p95 {:.2f} max {:.2f} deg (raw "
                          "gaze on the same targets: mean {:.2f} deg)\n",
                          comb.size(), p.val_res.size(), a.mean, a.median, a.p95, a.max, s.raw.mean);
    s.text += std::format("  per sample (accuracy + noise): mean {:.2f} median {:.2f} deg; precision RMS-S2S {:.2f} deg, SD "
                          "{:.2f} deg; data loss {:.0f}%\n",
                          ps.mean, ps.median, median_hi(prec), median_hi(sd), 100.0 * loss);
    s.text += std::format("  regions: centre (<=7 deg) {} | mid (7-12) {} | outer (>12) {} deg; per eye: left {} right {} deg\n",
                          reg(Region::Centre), reg(Region::Mid), reg(Region::Outer), per_eye(0), per_eye(1));
    for (std::size_t e = 0; e < 2; e++) {
        if (const auto& f = p.fits[e]) {
            if (const auto* b = f->selection.best()) {
                s.text += std::format("  {} eye: {}{} model on {} targets, calibration leave-one-out {:.2f} deg (raw {:.2f})\n",
                                      eye_name(e), em::model_name(b->kind), b->org ? " + origin" : "", f->points,
                                      b->loo.mean, f->selection.raw.mean);
            }
        }
    }
    s.text += std::format("  head-motion restarts: {}\n", p.head_restarts);
    if (const auto& pr = p.pursuit_res) {
        s.text += std::format("  smooth pursuit: mean {:.2f} median {:.2f} deg, lag {:.0f} ms\n", pr->err.mean,
                              pr->err.median, pr->lag_s * 1000.0);
    }
    s.hud.push_back(std::format("ACCURACY {:.2f} DEG (MEDIAN {:.2f}, 95% {:.2f})", a.mean, a.median, a.p95));
    s.hud.push_back(std::format("RAW WAS {:.1f} DEG   PRECISION {:.2f} DEG", s.raw.mean, median_hi(prec)));
    s.hud.push_back(std::format("CENTRE {}  MID {}  OUTER {}", reg(Region::Centre), reg(Region::Mid), reg(Region::Outer)));
    s.hud.push_back(std::format("LEFT {}  RIGHT {}  LOST {:.0f}%", per_eye(0), per_eye(1), 100.0 * loss));
    if (const auto& pr = p.pursuit_res) {
        s.hud.push_back(std::format("PURSUIT {:.2f} DEG", pr->err.mean));
    }
    s.notes.push_back(std::format("validated {}: accuracy mean {:.2f} median {:.2f} p95 {:.2f} deg on {} targets (raw "
                                  "{:.2f}), precision {:.2f} deg",
                                  stamp(unix_now()), a.mean, a.median, a.p95, comb.size(), s.raw.mean, median_hi(sd)));
    return s;
}

std::string json(const Procedure& p) {
    namespace em = pibridge::eyemath;
    std::string s = "{\n";
    s += "  \"tool\": \"pibridge-eyecal\",\n";
    s += std::format("  \"time_unix\": {},\n", unix_now());
    s += std::format("  \"mode\": {},\n", jstr(p.s.mode == Mode::Chip ? "chip" : (p.s.mode == Mode::Recenter ? "recenter" : "host")));
    const auto& c = p.s.conv;
    s += std::format("  \"flip_xyz\": [{}, {}, {}],\n", c.flip_x, c.flip_y, c.flip_z);
    const auto& ep = p.eye_positions();
    s += std::format("  \"eye_pos_m\": [[{}, {}, {}], [{}, {}, {}]],\n", jnum(ep[0][0]), jnum(ep[0][1]), jnum(ep[0][2]),
                     jnum(ep[1][0]), jnum(ep[1][1]), jnum(ep[1][2]));
    const auto sum = summary(p);
    std::string text = sum.text;
    while (!text.empty() && (text.back() == '\n' || text.back() == ' ')) {
        text.pop_back();
    }
    s += std::format("  \"summary\": {},\n", jstr(text));
    s += std::format("  \"validation_accuracy_deg\": {},\n", jstats(sum.accuracy));
    s += std::format("  \"validation_raw_deg\": {},\n", jstats(sum.raw));
    s += std::format("  \"head_restarts\": {},\n", p.head_restarts);
    std::vector<std::string> fits;
    for (std::size_t e = 0; e < 2; e++) {
        const auto& f = p.fits[e];
        if (!f) {
            fits.push_back(std::format("{}: null", jstr(eye_name(e))));
            continue;
        }
        std::vector<std::string> cands;
        for (const auto& m : f->selection.candidates) {
            cands.push_back(std::format(
                "{{\"model\": {}, \"origin\": {}, \"origin_u\": [{}], \"origin_v\": [{}], \"ridge_quadratic\": {}, \"loo_deg\": "
                "{}, \"train_deg\": {}, \"u\": [{}], \"v\": [{}]}}",
                jstr(em::model_name(m.kind)), m.org.has_value(), m.org ? three(m.org->first) : std::string(),
                m.org ? three(m.org->second) : std::string(), jnum(m.ridge.quadratic), jstats(m.loo), jstats(m.train),
                six(m.map.cx), six(m.map.cy)));
        }
        const auto* b = f->selection.best();
        fits.push_back(std::format("{}: {{\"points\": {}, \"raw_deg\": {}, \"chosen\": {}, \"candidates\": [{}]}}",
                                   jstr(eye_name(e)), f->points, jstats(f->selection.raw),
                                   b ? jstr(em::model_name(b->kind)) : std::string("null"), join(cands, ", ")));
    }
    s += std::format("  \"fits\": {{{}}},\n", join(fits, ", "));
    std::vector<std::string> cal;
    for (const auto& r : p.cal_res) {
        cal.push_back(std::format("    {{{}, \"ok\": {}, \"attempts\": {}, \"head_deg\": {}, \"left\": {}, \"right\": {}}}",
                                  jtarget(r.target), r.ok, r.attempts, jnum(r.head_deg), jeye(r.eyes[0]), jeye(r.eyes[1])));
    }
    s += std::format("  \"calibration_targets\": [\n{}\n  ],\n", join(cal, ",\n"));
    std::vector<std::string> val;
    for (const auto& v : p.val_res) {
        val.push_back(std::format(
            "    {{{}, \"region\": {}, \"err_deg\": {}, \"raw_err_deg\": {}, \"left_err_deg\": {}, \"right_err_deg\": {}, "
            "\"sample_err_deg\": {}, \"rms_s2s_deg\": {}, \"sd_deg\": {}, \"loss\": {}, \"head_deg\": {}, \"left\": {}, "
            "\"right\": {}}}",
            jtarget(v.point.target), jstr(region_name(region_of(v.point.target))), jopt(v.eval.comb_err_deg),
            jopt(v.raw_eval.comb_err_deg), jopt(v.eval.eye_err_deg[0]), jopt(v.eval.eye_err_deg[1]),
            jopt(v.eval.comb_sample_err_deg), jnum(v.eval.comb_rms_s2s_deg), jnum(v.eval.comb_sd_deg), jnum(v.eval.loss),
            jnum(v.point.head_deg), jeye(v.point.eyes[0]), jeye(v.point.eyes[1])));
    }
    s += std::format("  \"validation_targets\": [\n{}\n  ],\n", join(val, ",\n"));
    // the on-chip calibration: the chip's targets as shown, the gaze at each request, the validation per eye
    std::vector<std::string> chip;
    for (const auto& r : p.chip_res) {
        chip.push_back(std::format("    {{{}, \"chip_target\": {}, \"left\": {}, \"right\": {}}}", jtarget(r.target),
                                   r.target.chip, jeye(r.eyes[0]), jeye(r.eyes[1])));
    }
    s += std::format("  \"chip_targets\": [\n{}\n  ],\n", join(chip, ",\n"));
    std::string check = "null";
    if (p.chip_check) {
        std::vector<std::string> eyes;
        for (std::size_t e = 0; e < 2; e++) {
            const auto& x = p.chip_check->eyes[e];
            eyes.push_back(x ? std::format("{}: {{\"n\": {}, \"before_deg\": {}, \"after_deg\": {}, \"offset_u\": {:.6f}, "
                                           "\"offset_v\": {:.6f}, \"applied\": {}, \"mirrored_x\": {}, \"mirrored_y\": {}}}",
                                           jstr(eye_name(e)), x->n, jnum(x->before_deg), jnum(x->after_deg), x->offset.u,
                                           x->offset.v, x->apply, x->mirrored_x, x->mirrored_y)
                             : std::format("{}: null", jstr(eye_name(e))));
        }
        check = "{" + join(eyes, ", ") + "}";
    }
    s += std::format("  \"chip_validation\": {},\n", check);
    s += std::format("  \"chip_profile_bytes\": {},\n", p.chip_profile ? std::to_string(p.chip_profile->size()) : std::string("null"));
    s += std::format("  \"pursuit\": {},\n",
                     p.pursuit_res ? std::format("{{\"lag_s\": {}, \"err_deg\": {}, \"used\": {}}}", jnum(p.pursuit_res->lag_s),
                                                 jstats(p.pursuit_res->err), jnum(p.pursuit_res->used))
                                   : std::string("null"));
    std::vector<std::string> se;
    for (const auto& e : p.service_err) {
        se.push_back(jopt(e));
    }
    s += std::format("  \"service_err_deg\": [{}],\n", join(se, ", "));
    s += std::format("  \"calibration\": {}\n", p.calibration ? jstr(p.calibration->to_toml()) : std::string("null"));
    s += "}\n";
    return s;
}

} // namespace pibridge::eyecal
