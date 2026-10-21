// SPDX-License-Identifier: MIT
#include "pibridge/roommap/guide_ipc.h"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <format>
#include <fstream>
#include <iterator>
#include <sstream>

namespace pibridge::roommap::guide {

namespace {

std::string one_line(std::string s) {
    for (auto& c : s) {
        if (c == '\n' || c == '\r') {
            c = ' ';
        }
    }
    return s;
}

std::vector<double> numbers(std::string_view rest) {
    std::vector<double> v;
    std::size_t i = 0;
    while (i < rest.size()) {
        while (i < rest.size() && rest[i] == ' ') {
            i++;
        }
        std::size_t j = i;
        while (j < rest.size() && rest[j] != ' ') {
            j++;
        }
        if (j > i) {
            double d = 0;
            const auto r = std::from_chars(rest.data() + i, rest.data() + j, d);
            if (r.ec != std::errc{} || r.ptr != rest.data() + j || !std::isfinite(d)) {
                return {std::nan("")};
            }
            v.push_back(d);
        }
        i = j;
    }
    return v;
}

bool bad(const std::vector<double>& v) {
    return v.size() == 1 && std::isnan(v[0]);
}

std::string num(double d) { return std::format("{:.4f}", d); }

template <class F>
std::expected<void, std::string> each_line(std::string_view text, std::string_view magic, F f) {
    std::size_t at = 0;
    bool first = true;
    while (at < text.size()) {
        std::size_t e = text.find('\n', at);
        if (e == std::string_view::npos) {
            e = text.size();
        }
        const std::string_view line = text.substr(at, e - at);
        at = e + 1;
        if (line.empty()) {
            continue;
        }
        if (first) {
            if (line != magic) {
                return std::unexpected(std::format("not a guide file (first line \"{}\")", line));
            }
            first = false;
            continue;
        }
        const std::size_t sp = line.find(' ');
        const std::string_view key = line.substr(0, sp);
        const std::string_view rest = sp == std::string_view::npos ? std::string_view{} : line.substr(sp + 1);
        if (auto r = f(key, rest); !r) {
            return r;
        }
    }
    if (first) {
        return std::unexpected(std::string("empty guide file"));
    }
    return {};
}

template <class T>
std::expected<T, std::string> integer(std::string_view key, std::string_view rest) {
    T v{};
    const auto r = std::from_chars(rest.data(), rest.data() + rest.size(), v);
    if (r.ec != std::errc{} || r.ptr != rest.data() + rest.size()) {
        return std::unexpected(std::format("bad {} \"{}\"", key, rest));
    }
    return v;
}

} // namespace

std::array<double, 3> Conv::apply(const std::array<double, 3>& p) const {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * p[0] + s * p[2] + t[0], p[1] + t[1], -s * p[0] + c * p[2] + t[2]};
}

std::array<double, 3> Conv::inverse_apply(const std::array<double, 3>& p) const {
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double x = p[0] - t[0], y = p[1] - t[1], z = p[2] - t[2];
    return {c * x - s * z, y, s * x + c * z};
}

std::string format_state(const State& s) {
    std::string o = "pibridge-roommap-guide 1\n";
    o += std::format("pid {}\nseq {}\nstep {}\ntitle {}\ntext {}\n", s.pid, s.seq, one_line(s.step), one_line(s.title),
                     one_line(s.text));
    if (s.progress) {
        o += "progress " + num(*s.progress) + "\n";
    }
    o += std::format("keyframes {}\npoints {}\n", s.keyframes, s.points);
    if (s.conv) {
        o += std::format("conv {:.6f} {} {} {}\n", s.conv->yaw, num(s.conv->t[0]), num(s.conv->t[1]), num(s.conv->t[2]));
    }
    if (s.floor) {
        o += "floor " + num(*s.floor) + "\n";
    }
    if (!s.boundary.empty()) {
        o += "boundary";
        for (const auto& p : s.boundary) {
            o += " " + num(p[0]) + " " + num(p[1]);
        }
        o += "\n";
    }
    return o;
}

std::expected<State, std::string> parse_state(std::string_view text) {
    State s;
    auto r = each_line(text, "pibridge-roommap-guide 1", [&](std::string_view k, std::string_view v) -> std::expected<void, std::string> {
        if (k == "pid") {
            auto x = integer<std::uint32_t>(k, v);
            if (!x) {
                return std::unexpected(x.error());
            }
            s.pid = *x;
        } else if (k == "seq") {
            auto x = integer<std::uint64_t>(k, v);
            if (!x) {
                return std::unexpected(x.error());
            }
            s.seq = *x;
        } else if (k == "keyframes" || k == "points") {
            auto x = integer<std::size_t>(k, v);
            if (!x) {
                return std::unexpected(x.error());
            }
            (k == "points" ? s.points : s.keyframes) = *x;
        } else if (k == "step") {
            s.step = std::string(v);
        } else if (k == "title") {
            s.title = std::string(v);
        } else if (k == "text") {
            s.text = std::string(v);
        } else {
            const auto n = numbers(v);
            if (bad(n)) {
                return std::unexpected(std::format("bad numbers in \"{}\"", k));
            }
            if (k == "progress" && n.size() == 1) {
                s.progress = n[0];
            } else if (k == "floor" && n.size() == 1) {
                s.floor = n[0];
            } else if (k == "conv" && n.size() == 4) {
                s.conv = Conv{n[0], {n[1], n[2], n[3]}};
            } else if (k == "boundary" && n.size() % 2 == 0) {
                for (std::size_t i = 0; i + 1 < n.size(); i += 2) {
                    s.boundary.push_back({n[i], n[i + 1]});
                }
            } else if (k == "progress" || k == "floor" || k == "conv" || k == "boundary") {
                return std::unexpected(std::format("wrong count of numbers in \"{}\"", k));
            } // unknown keys: a newer writer; ignored
        }
        return {};
    });
    if (!r) {
        return std::unexpected(r.error());
    }
    if (s.step.empty()) {
        return std::unexpected(std::string("guide state without a step"));
    }
    return s;
}

std::string format_answer(const Answer& a) {
    std::string o = std::format("pibridge-roomsetup-guide 1\npid {}\nid {}\nstep {}\nanswer {}", a.pid, a.id, one_line(a.step),
                                one_line(a.kind));
    for (const double d : a.nums) {
        o += " " + num(d);
    }
    return o + "\n";
}

std::expected<Answer, std::string> parse_answer(std::string_view text) {
    Answer a;
    bool have = false;
    auto r = each_line(text, "pibridge-roomsetup-guide 1", [&](std::string_view k, std::string_view v) -> std::expected<void, std::string> {
        if (k == "pid") {
            auto x = integer<std::uint32_t>(k, v);
            if (!x) {
                return std::unexpected(x.error());
            }
            a.pid = *x;
        } else if (k == "id") {
            auto x = integer<std::uint64_t>(k, v);
            if (!x) {
                return std::unexpected(x.error());
            }
            a.id = *x;
        } else if (k == "step") {
            a.step = std::string(v);
        } else if (k == "answer") {
            const std::size_t sp = v.find(' ');
            a.kind = std::string(v.substr(0, sp));
            if (sp != std::string_view::npos) {
                a.nums = numbers(v.substr(sp + 1));
                if (bad(a.nums)) {
                    return std::unexpected(std::string("bad numbers in the answer"));
                }
            }
            have = true;
        }
        return {};
    });
    if (!r) {
        return std::unexpected(r.error());
    }
    if (!have || a.kind.empty()) {
        return std::unexpected(std::string("guide answer without an answer"));
    }
    const std::size_t n = a.nums.size();
    // the room-setup rework adds the panel's control chips: mode (1 num: 1 = full, 0 = quick), posture (1 num:
    // 0 = stand, 1 = sit), size (1 num: 0/1/2 = small/medium/big), and back / repeat / next (0 nums).
    const bool ok = (a.kind == "yes" && n == 0) || (a.kind == "cancel" && n == 0) || (a.kind == "touch" && n == 1) ||
                    (a.kind == "height" && n == 2) || (a.kind == "forward" && n == 4) ||
                    (a.kind == "path" && n >= 6 && n % 2 == 0) || (a.kind == "mode" && n == 1) ||
                    (a.kind == "posture" && n == 1) || (a.kind == "size" && n == 1) || (a.kind == "back" && n == 0) ||
                    (a.kind == "repeat" && n == 0) || (a.kind == "next" && n == 0);
    if (!ok) {
        return std::unexpected(std::format("answer \"{}\" with {} numbers", a.kind, n));
    }
    return a;
}

std::expected<void, std::string> write_atomic(const std::string& dir, const char* name, const std::string& text) {
    const std::string p = dir + "/" + name;
    const std::string tmp = p + ".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f || !(f << text) || !f.flush()) {
            return std::unexpected("cannot write " + tmp);
        }
    }
    if (std::rename(tmp.c_str(), p.c_str()) != 0) {
        return std::unexpected("cannot rename " + tmp);
    }
    return {};
}

std::optional<std::string> read_file(const std::string& dir, const char* name) {
    std::ifstream f(dir + "/" + name, std::ios::binary);
    if (!f) {
        return std::nullopt;
    }
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

} // namespace pibridge::roommap::guide
