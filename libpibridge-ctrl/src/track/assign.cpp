// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-ctrltrack/src/assign.rs (MIT OR Apache-2.0).
#include "pibridge/ctrl/track/assign.h"

#include <cmath>
#include <limits>

namespace pibridge::ctrl::track {

namespace {

double dist(const V3& a, const V3& b) {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

const char* Body::implausible(std::size_t slot, const V3& p) const {
    if (dist(p, shoulder[slot]) > reach) {
        return "out of reach";
    }
    if ((slot == 0 && p[0] > cross) || (slot == 1 && p[0] < -cross)) {
        return "wrong side";
    }
    return nullptr;
}

std::optional<double> Body::cost(std::size_t slot, const Cand& c, const Slot& s, const Slot& other) const {
    if (implausible(slot, c.p) != nullptr) {
        return std::nullopt;
    }
    if (other.tracked_now && dist(*other.tracked_now, c.p) < min_sep) {
        return std::nullopt;
    }
    double cost = 0.0;
    if (s.last) {
        const auto& [lp, age] = *s.last;
        if (age < memory_s) {
            const double d = dist(lp, c.p);
            if (d > 0.12 + max_speed * age) {
                return std::nullopt;
            }
            cost += d;
        }
    }
    if ((slot == 0 && c.p[0] > 0.0) || (slot == 1 && c.p[0] < 0.0)) {
        cost += 0.15 * std::fabs(c.p[0]) / 0.1;
    }
    if (c.model != slot) {
        cost += 0.05;
    }
    return cost;
}

std::vector<std::pair<std::size_t, std::size_t>> Body::assign(std::span<const Cand> cands,
                                                              const std::array<Slot, 2>& slots) const {
    using Opt = std::vector<std::pair<std::size_t, std::size_t>>;
    std::vector<std::size_t> free;
    for (std::size_t s = 0; s < 2; s++) {
        if (!slots[s].tracked_now) {
            free.push_back(s);
        }
    }
    std::size_t best_n = 0;
    double best_total = 0.0;
    Opt best;
    // every injective map of a subset of candidates onto free slots (at most 2 x 2)
    const std::size_t n = std::min<std::size_t>(cands.size(), 4);
    std::vector<Opt> options{Opt{}};
    for (const std::size_t s : free) {
        std::vector<Opt> next;
        for (const Opt& o : options) {
            next.push_back(o);
            for (std::size_t c = 0; c < n; c++) {
                bool taken = false;
                for (const auto& [ci, _] : o) {
                    taken = taken || ci == c;
                }
                if (!taken) {
                    Opt o2 = o;
                    o2.emplace_back(c, s);
                    next.push_back(std::move(o2));
                }
            }
        }
        options = std::move(next);
    }
    for (Opt& o : options) {
        double total = 0.0;
        bool ok = true;
        for (const auto& [c, s] : o) {
            const std::size_t other = 1 - s;
            // the other slot's hand this frame: tracked, or assigned in this option
            Slot os = slots[other];
            for (const auto& [c2, s2] : o) {
                if (s2 == other) {
                    os.tracked_now = cands[c2].p;
                    break;
                }
            }
            const auto v = cost(s, cands[c], slots[s], os);
            if (!v) {
                ok = false;
                break;
            }
            total += *v;
        }
        if (ok && (o.size() > best_n || (o.size() == best_n && total < best_total))) {
            best_n = o.size();
            best_total = total;
            best = std::move(o);
        }
    }
    return best;
}

std::optional<std::size_t> Body::collapsed(const std::array<V3, 2>& now,
                                           const std::array<std::optional<V3>, 2>& last) const {
    if (dist(now[0], now[1]) >= min_sep) {
        return std::nullopt;
    }
    const auto jump = [&](std::size_t s) {
        return last[s] ? dist(*last[s], now[s]) : std::numeric_limits<double>::infinity();
    };
    return jump(0) > jump(1) ? 0 : 1;
}

} // namespace pibridge::ctrl::track
