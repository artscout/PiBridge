// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/model.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/model.h"

#include "pibridge/cam/calib.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/json.h"
#include "pibridge/cam/rustfmt.h"

#include <algorithm>
#include <cctype>
#include <dirent.h>
#include <format>

namespace pibridge::ctrl::track {

namespace g = cam::geom;
namespace rf = cam::rfmt;

Env process_env() {
    return [](std::string_view k) { return cam::env_utf8(std::string(k).c_str()); };
}

std::string ModelSource::describe() const {
    if (files) {
        return std::format("your own controller models ({} / {})", rf::utf8_lossy((*files)[0]),
                           rf::utf8_lossy((*files)[1]));
    }
    return "the shipped Pimax Dream Air reference controller models (no own copy found; the app imports yours from a "
           "Pimax Windows install)";
}

std::expected<LedModel, std::string> LedModel::parse(std::string_view text) {
    auto v = cam::json::parse(text);
    if (!v) {
        return std::unexpected(std::format("json: JsonError {{ pos: {}, msg: {} }}", v.error().pos,
                                           rf::debug_str(v.error().msg)));
    }
    const cam::json::Value* mp = v->get("ModelPoints");
    if (mp == nullptr) {
        return std::unexpected(std::string("no ModelPoints"));
    }
    LedModel m;
    for (std::size_t i = 0;; i++) {
        const cam::json::Value* p = mp->get(std::format("Point{}", i));
        if (p == nullptr) {
            break;
        }
        const auto a = p->as_f64_array<6>();
        if (!a) {
            return std::unexpected(std::string("ModelPoint is not 6 numbers"));
        }
        m.leds.push_back(Led{{(*a)[0], (*a)[1], (*a)[2]}, g::normalize({(*a)[3], (*a)[4], (*a)[5]})});
    }
    if (m.leds.size() < 4) {
        return std::unexpected(std::format("only {} LEDs", m.leds.size()));
    }
    const cam::json::Value* itd = v->get("ImuToDevice");
    const auto mm = itd != nullptr ? itd->as_f64_array<16>() : std::nullopt;
    if (!mm) {
        return std::unexpected(std::string("no ImuToDevice"));
    }
    const auto& x = *mm;
    const g::Mat3 r{{{x[0], x[1], x[2]}, {x[4], x[5], x[6]}, {x[8], x[9], x[10]}}};
    const auto str_or = [&](const char* k) {
        const cam::json::Value* h = v->get(k);
        const std::string* s = h != nullptr ? h->as_str() : nullptr;
        return s != nullptr ? *s : std::string("?");
    };
    m.hand = str_or("Hand");
    m.serial = str_or("SN");
    m.imu_to_device = g::Pose{r, {x[3], x[7], x[11]}};
    return m;
}

std::expected<LedModel, std::string> LedModel::load(const std::string& path) {
    auto t = cam::read_to_string(path);
    if (!t) {
        return std::unexpected(std::format("{}: {}", rf::utf8_lossy(path), t.error().text()));
    }
    auto m = parse(*t);
    if (!m) {
        return std::unexpected(std::format("{}: {}", rf::utf8_lossy(path), m.error()));
    }
    return m;
}

std::vector<std::string> LedModel::search_dirs(const Env& env) {
    const std::string home = env("HOME").value_or("");
    std::vector<std::string> dirs;
    if (const auto d = env("PIMAX_CTRL_MODEL_DIR"); d && !d->empty()) {
        dirs.push_back(*d);
    }
    dirs.push_back(cam::path_join(home, ".config/omarchyvr/controllers"));
    dirs.push_back(cam::path_join(home, ".local/share/omarchyvr/controller-models"));
    // the repo as omarchyvr_config::paths::repo_dir finds it
    std::string repo;
    if (const auto r = env("PIMAX_REPO"); r && !r->empty()) {
        repo = *r;
    } else {
        const std::string installed = cam::path_join(home, ".local/share/omarchyvr/data");
        repo = cam::is_file(cam::path_join(installed, "docs/LESSONS.md")) ? installed
                                                                           : cam::path_join(home, "omarchyvr");
    }
    dirs.push_back(cam::path_join(repo, "pimax-data/ProgramData-Pimax/slam/H1"));
    return dirs;
}

std::optional<std::array<std::string, 2>> LedModel::own_paths(const Env& env) {
    for (const std::string& d : search_dirs(env)) {
        if (auto p = pair_in(d)) {
            return p;
        }
    }
    return std::nullopt;
}

std::array<std::string, 2> LedModel::default_paths() {
    const Env env = process_env();
    if (auto p = own_paths(env)) {
        return *p;
    }
    const std::string c = cam::path_join(env("HOME").value_or(""), ".config/omarchyvr/controllers");
    return {cam::path_join(c, "left.json"), cam::path_join(c, "right.json")};
}

std::expected<std::pair<std::array<LedModel, 2>, ModelSource>, std::string> LedModel::resolve(
    const std::optional<std::string>& left, const std::optional<std::string>& right, const Env& env) {
    const auto own = own_paths(env);
    const auto pick = [&](const std::optional<std::string>& given, std::size_t i) -> std::optional<std::string> {
        if (given) {
            return given;
        }
        if (own) {
            return (*own)[i];
        }
        return std::nullopt;
    };
    const auto l = pick(left, 0);
    const auto r = pick(right, 1);
    if (l && r) {
        auto ml = load(*l);
        if (!ml) {
            return std::unexpected(ml.error());
        }
        auto mr = load(*r);
        if (!mr) {
            return std::unexpected(mr.error());
        }
        return std::pair{std::array<LedModel, 2>{std::move(*ml), std::move(*mr)},
                         ModelSource{std::array<std::string, 2>{*l, *r}}};
    }
    if (!l && !r && cam::reference_allowed(env)) {
        auto ml = parse(reference_left());
        if (!ml) {
            return std::unexpected(ml.error());
        }
        auto mr = parse(reference_right());
        if (!mr) {
            return std::unexpected(mr.error());
        }
        return std::pair{std::array<LedModel, 2>{std::move(*ml), std::move(*mr)}, ModelSource{}};
    }
    const std::string c = cam::path_join(env("HOME").value_or(""), ".config/omarchyvr/controllers");
    const std::string missing = !l ? cam::path_join(c, "left.json") : r.value_or(cam::path_join(c, "right.json"));
    return std::unexpected(std::format("controller models missing ({}: no such file); import them in the app (Room "
                                       "setup) or from a Pimax Windows install",
                                       rf::utf8_lossy(missing)));
}

std::optional<std::array<std::string, 2>> LedModel::pair_in(const std::string& dir) {
    const std::string l = cam::path_join(dir, "left.json");
    const std::string r = cam::path_join(dir, "right.json");
    if (cam::is_file(l) && cam::is_file(r)) {
        return std::array<std::string, 2>{l, r};
    }
    DIR* d = ::opendir(dir.empty() ? "." : dir.c_str());
    if (d == nullptr || dir.empty()) {
        // Rust read_dir("") fails (ENOENT)
        if (d != nullptr) {
            ::closedir(d);
        }
        return std::nullopt;
    }
    // (folder name, file): the paths share the directory and the file name, so Rust's component-wise Path order is
    // the byte order of the folder names
    std::vector<std::pair<std::string, std::string>> v;
    while (const dirent* e = ::readdir(d)) {
        const std::string name = e->d_name;
        if (name == "." || name == "..") {
            continue;
        }
        const std::string p = cam::path_join(cam::path_join(dir, name), "controller_params.json");
        if (cam::is_file(p)) {
            v.emplace_back(name, p);
        }
    }
    ::closedir(d);
    std::sort(v.begin(), v.end());
    if (v.size() < 2) {
        return std::nullopt;
    }
    // folder-name order is what the tracker always used; the files' own `Hand` field wins when it says otherwise
    const auto hand = [](const std::string& p) -> std::optional<std::string> {
        const auto t = cam::read_to_string(p);
        if (!t) {
            return std::nullopt;
        }
        const auto m = parse(*t);
        if (!m) {
            return std::nullopt;
        }
        std::string h = m->hand;
        for (char& ch : h) {
            if (ch >= 'A' && ch <= 'Z') {
                ch = static_cast<char>(ch - 'A' + 'a');
            }
        }
        return h;
    };
    if (hand(v[0].second) == "right" && hand(v[1].second) == "left") {
        std::swap(v[0], v[1]);
    }
    return std::array<std::string, 2>{v[0].second, v[1].second};
}

std::vector<std::array<std::size_t, 3>> LedModel::neighbour_triples(double max_dist, double max_normal_deg) const {
    const std::size_t n = leds.size();
    const auto ok = [&](std::size_t a, std::size_t b) {
        const Led& la = leds[a];
        const Led& lb = leds[b];
        return g::norm(g::sub(la.p, lb.p)) <= max_dist && g::angle_deg(la.n, lb.n) <= max_normal_deg;
    };
    std::vector<std::array<std::size_t, 3>> out;
    for (std::size_t i = 0; i < n; i++) {
        for (std::size_t j = 0; j < n; j++) {
            for (std::size_t k = 0; k < n; k++) {
                if (i != j && j != k && i != k && ok(i, j) && ok(j, k) && ok(i, k)) {
                    out.push_back({i, j, k});
                }
            }
        }
    }
    return out;
}

} // namespace pibridge::ctrl::track
