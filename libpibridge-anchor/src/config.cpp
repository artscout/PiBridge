// SPDX-License-Identifier: MIT
// Ported from omarchyvr omarchyvr-anchor/src/config.rs (MIT OR Apache-2.0). See config.h.
#include "pibridge/anchor/config.h"

#include "pibridge/anchor/rustx.h"
#include "pibridge/anchor/world.h"
#include "pibridge/cam/io.h"
#include "pibridge/cam/rustfmt.h"

#include <cstdio>
#include <cstdlib>
#include <optional>

namespace pibridge::anchor {

namespace rf = cam::rfmt;

std::string anchor_config_path() {
    for (const char* n : {"PIBRIDGE_ANCHOR_CONFIG", "PIMAX_ANCHOR_CONFIG"}) {
        if (auto v = cam::env_os(n)) {
            return *v;
        }
    }
    const std::string home = cam::env_os("HOME").value_or("");
    const std::string pibridge = cam::path_join(home, ".config/pibridge/anchor.toml");
    for (const char* rel : {".config/pibridge/anchor.toml", ".config/pimax-linux/anchor.toml", ".config/omarchyvr/anchor.toml"}) {
        const std::string p = cam::path_join(home, rel);
        if (cam::path_exists(p)) {
            return p;
        }
    }
    return pibridge;
}

namespace {

std::pair<std::string, std::string> arr(const cam::geom::Pose& p) {
    const auto [t, q] = pq(p);
    return {"[" + rf::fixed(t[0], 6) + ", " + rf::fixed(t[1], 6) + ", " + rf::fixed(t[2], 6) + "]",
            "[" + rf::fixed(q[0], 7) + ", " + rf::fixed(q[1], 7) + ", " + rf::fixed(q[2], 7) + ", " + rf::fixed(q[3], 7) +
                "]"};
}

} // namespace

std::string Anchor::to_toml() const {
    const auto [p, q] = arr(reference);
    std::string s = "# written by `omarchyvr-anchor calibrate`\n[tag]\nfamily = \"tag36h11\"\nid = " + std::to_string(id) +
                    "\nsize_m = " + rf::display_f64(size_m) + "\n\n[reference]\npos = " + p + "\nquat = " + q +
                    "\nsaved = \"" + saved + "\"\n";
    for (const auto& w : windows) {
        const auto [wp, wq] = arr(w.tag_from_window);
        s += "\n[[window]]\nname = \"" + w.name + "\"\npos = " + wp + "\nquat = " + wq + "\n";
    }
    return s;
}

std::expected<Anchor, std::string> Anchor::parse(std::string_view text) {
    std::string sect;
    std::optional<std::uint32_t> id;
    std::optional<double> size;
    std::optional<std::vector<double>> rpos;
    std::optional<std::vector<double>> rquat;
    std::string saved;
    struct W {
        std::string name;
        std::optional<std::vector<double>> p;
        std::optional<std::vector<double>> q;
    };
    std::vector<W> windows;
    for (std::string_view line : rs::lines(text)) {
        const std::size_t hash = line.find('#');
        const std::string_view l = rf::trim(hash == std::string_view::npos ? line : line.substr(0, hash));
        if (l.empty()) {
            continue;
        }
        if (l.front() == '[') {
            sect = std::string(rs::trim_matches(l, "[]"));
            if (sect == "window") {
                windows.push_back(W{});
            }
            continue;
        }
        const auto kv = rs::split_once(l, '=');
        if (!kv) {
            return std::unexpected("bad line: " + std::string(l));
        }
        const std::string_view k = rf::trim(kv->first);
        const std::string_view v = rf::trim(kv->second);
        std::string err;
        auto nums = [&]() -> std::expected<std::vector<double>, std::string> {
            std::vector<double> out;
            std::string_view rest = rs::trim_matches(v, "[]");
            for (;;) {
                const std::size_t c = rest.find(',');
                const std::string_view x = rf::trim(c == std::string_view::npos ? rest : rest.substr(0, c));
                std::string e;
                auto d = rs::parse_f64(x, &e);
                if (!d) {
                    return std::unexpected(std::string(k) + ": " + e);
                }
                out.push_back(*d);
                if (c == std::string_view::npos) {
                    break;
                }
                rest = rest.substr(c + 1);
            }
            return out;
        };
        const std::string s(rs::trim_matches(v, "\""));
        // [f64; N]: a vector of another length gives None (`try_into().ok()`)
        auto fixed_len = [](std::vector<double> x, std::size_t n) -> std::optional<std::vector<double>> {
            if (x.size() != n) {
                return std::nullopt;
            }
            return x;
        };
        if (sect == "tag" && k == "id") {
            auto x = rs::parse_u32(v, &err);
            if (!x) {
                return std::unexpected(err);
            }
            id = *x;
        } else if (sect == "tag" && k == "size_m") {
            auto x = rs::parse_f64(v, &err);
            if (!x) {
                return std::unexpected(err);
            }
            size = *x;
        } else if (sect == "reference" && k == "pos") {
            auto n = nums();
            if (!n) {
                return std::unexpected(n.error());
            }
            rpos = fixed_len(*n, 3);
        } else if (sect == "reference" && k == "quat") {
            auto n = nums();
            if (!n) {
                return std::unexpected(n.error());
            }
            rquat = fixed_len(*n, 4);
        } else if (sect == "reference" && k == "saved") {
            saved = s;
        } else if (sect == "window" && k == "name") {
            windows.back().name = s;
        } else if (sect == "window" && k == "pos") {
            auto n = nums();
            if (!n) {
                return std::unexpected(n.error());
            }
            windows.back().p = fixed_len(*n, 3);
        } else if (sect == "window" && k == "quat") {
            auto n = nums();
            if (!n) {
                return std::unexpected(n.error());
            }
            windows.back().q = fixed_len(*n, 4);
        }
    }
    if (!id) {
        return std::unexpected("tag.id missing");
    }
    if (!size) {
        return std::unexpected("tag.size_m missing");
    }
    if (!rpos) {
        return std::unexpected("reference.pos missing");
    }
    if (!rquat) {
        return std::unexpected("reference.quat missing");
    }
    Anchor a;
    a.id = *id;
    a.size_m = *size;
    a.reference = from_pq({(*rpos)[0], (*rpos)[1], (*rpos)[2]}, {(*rquat)[0], (*rquat)[1], (*rquat)[2], (*rquat)[3]});
    a.saved = saved;
    for (const auto& w : windows) {
        if (w.p && w.q) {
            a.windows.push_back(WindowRel{w.name, from_pq({(*w.p)[0], (*w.p)[1], (*w.p)[2]},
                                                          {(*w.q)[0], (*w.q)[1], (*w.q)[2], (*w.q)[3]})});
        }
    }
    return a;
}

std::expected<Anchor, std::string> Anchor::load() {
    const std::string p = anchor_config_path();
    auto t = cam::read_to_string(p);
    if (!t) {
        return std::unexpected(p + ": " + t.error().text() + " (run `omarchyvr-anchor calibrate` first)");
    }
    return parse(*t);
}

std::expected<std::string, std::string> Anchor::save() const {
    const std::string p = anchor_config_path();
    if (auto d = cam::path_parent(p); d && !d->empty()) {
        if (auto r = cam::create_dir_all(*d); !r) {
            return std::unexpected(r.error().text());
        }
    }
    const std::string tmp = rs::with_extension(p, "toml.tmp");
    if (auto w = cam::write_file(tmp, to_toml()); !w) {
        return std::unexpected(w.error().text());
    }
    if (std::rename(tmp.c_str(), p.c_str()) != 0) {
        return std::unexpected(cam::IoError::last_os().text());
    }
    return p;
}

} // namespace pibridge::anchor
