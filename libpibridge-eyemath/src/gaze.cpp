// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/gaze.rs (MIT OR Apache-2.0).
#include "pibridge/eyemath/gaze.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <format>

#include "num.h"
#include "tlv_lite.h"

namespace pibridge::eyemath {

namespace {

using detail::Value;
using VK = detail::Value::Kind;

constexpr std::array<std::uint8_t, 13> FRAME_FIELDS{1, 14, 15, 18, 19, 20, 21, 26, 27, 28, 29, 34, 35};

std::uint64_t group_mask(std::size_t g) {
    std::uint64_t m = 0;
    for (auto f : EYE_FIELDS[g]) {
        m |= std::uint64_t{1} << f;
    }
    return m;
}

std::uint64_t all_mask() {
    std::uint64_t m = 0;
    for (auto f : WIRE_ORDER) {
        m |= std::uint64_t{1} << f;
    }
    return m;
}

/// `(group, index)` of an eye field.
std::optional<std::pair<std::size_t, std::size_t>> eye_slot(std::uint8_t n) {
    for (std::size_t g = 0; g < EYE_FIELDS.size(); g++) {
        for (std::size_t i = 0; i < EYE_FIELDS[g].size(); i++) {
            if (EYE_FIELDS[g][i] == n) {
                return std::pair{g, i};
            }
        }
    }
    return std::nullopt;
}

GazeError bad(std::int32_t field, const char* reason) {
    GazeError e;
    e.kind = GazeError::Kind::BadField;
    e.field = field;
    e.reason = reason;
    return e;
}

GazeError simple(GazeError::Kind k) {
    GazeError e;
    e.kind = k;
    return e;
}

struct Cursor {
    detail::Items items;
    bool cut = false;

    std::optional<Value> next() {
        detail::RawItem it;
        switch (items.next(it)) {
        case detail::Items::Step::Item:
            return it.decode();
        case detail::Items::Step::Error:
            cut = true;
            return std::nullopt;
        case detail::Items::Step::End:
            break;
        }
        return std::nullopt;
    }
};

struct Val {
    enum class Kind { I32, I64, F16, Vec3, Vec2, Pair22 };
    Kind kind = Kind::I32;
    std::int64_t x = 0;
    std::array<std::int64_t, 3> a{};
};

bool is_compound(const Value& v, std::uint16_t count, std::uint16_t type_id) {
    return v.kind == VK::Compound && v.count == count && v.type_id == type_id;
}

/// Read one element. An empty optional means the payload ended inside or before it.
std::expected<std::optional<std::pair<std::int32_t, Val>>, GazeError> read_element(Cursor& cur) {
    const auto head = cur.next();
    if (!head) {
        return std::nullopt;
    }
    if (!is_compound(*head, 2, TYPE_FIELD)) {
        return std::unexpected(bad(-1, "element does not start with a 0x20bb9 compound"));
    }
    const auto num = cur.next();
    if (!num) {
        return std::nullopt;
    }
    if (num->kind != VK::Int32) {
        return std::unexpected(bad(-1, "element field number is not an int32"));
    }
    const auto no = static_cast<std::int32_t>(num->i);
    const auto v = cur.next();
    if (!v) {
        return std::nullopt;
    }
    Val val;
    // read `n` components of kind `want` into val.a
    auto components = [&](std::size_t n, VK want, const char* reason) -> std::expected<bool, GazeError> {
        for (std::size_t k = 0; k < n; k++) {
            const auto c = cur.next();
            if (!c) {
                return false;
            }
            if (c->kind != want) {
                return std::unexpected(bad(no, reason));
            }
            val.a[k] = c->i;
        }
        return true;
    };
    switch (v->kind) {
    case VK::Int32:
        val.kind = Val::Kind::I32;
        val.x = v->i;
        break;
    case VK::Int64:
        val.kind = Val::Kind::I64;
        val.x = v->i;
        break;
    case VK::Fixed16_16:
        val.kind = Val::Kind::F16;
        val.x = v->i;
        break;
    case VK::Compound: {
        std::expected<bool, GazeError> r;
        if (is_compound(*v, 3, TYPE_VEC3_F16)) {
            val.kind = Val::Kind::Vec3;
            r = components(3, VK::Fixed16_16, "vec3 component is not S15.16");
        } else if (is_compound(*v, 2, TYPE_VEC2_F16)) {
            val.kind = Val::Kind::Vec2;
            r = components(2, VK::Fixed16_16, "vec2 component is not S15.16");
        } else if (is_compound(*v, 2, TYPE_PAIR_F22)) {
            val.kind = Val::Kind::Pair22;
            r = components(2, VK::Fixed22_42, "pair component is not S22.42");
        } else {
            return std::unexpected(bad(no, "unknown compound type"));
        }
        if (!r) {
            return std::unexpected(r.error());
        }
        if (!*r) {
            return std::nullopt;
        }
        break;
    }
    default:
        return std::unexpected(bad(no, "unsupported value tag"));
    }
    return std::pair{no, val};
}

double f16(std::int64_t raw) { return fixed16_16_to_f64(static_cast<std::int32_t>(raw)); }

Vec3 v3(const std::array<std::int64_t, 3>& a) { return {f16(a[0]), f16(a[1]), f16(a[2])}; }

/// Store a decoded value. Returns false for an unknown field number.
std::expected<bool, GazeError> apply(GazeSample& s, std::int32_t no, const Val& val) {
    if (no < 0 || no > 255) {
        return false;
    }
    const auto n = static_cast<std::uint8_t>(no);
    const auto mismatch = [&] { return std::unexpected(bad(no, "value type does not match the field")); };
    const auto is = [&](Val::Kind k) { return val.kind == k; };
    const auto i32 = static_cast<std::int32_t>(val.x);
    if (const auto slot = eye_slot(n)) {
        auto& e = s.eyes[slot->first];
        switch (slot->second) {
        case 0:
            if (!is(Val::Kind::Vec3)) return mismatch();
            e.origin_mm = v3(val.a);
            break;
        case 1:
            if (!is(Val::Kind::I32)) return mismatch();
            e.origin_valid = i32 != 0;
            break;
        case 2:
            if (!is(Val::Kind::Vec3)) return mismatch();
            e.direction = v3(val.a);
            break;
        case 3:
            if (!is(Val::Kind::I32)) return mismatch();
            e.direction_valid = i32 != 0;
            break;
        case 4:
            if (!is(Val::Kind::F16)) return mismatch();
            e.pupil_diameter_mm = f16(val.x);
            break;
        case 5:
            if (!is(Val::Kind::F16)) return mismatch();
            e.openness_raw = f16(val.x);
            break;
        case 6:
            if (!is(Val::Kind::Pair22)) return mismatch();
            e.aux_pair = {fixed22_42_to_f64(val.a[0]), fixed22_42_to_f64(val.a[1])};
            break;
        case 7:
            if (!is(Val::Kind::Vec2)) return mismatch();
            e.pupil_pos = {f16(val.a[0]), f16(val.a[1])};
            break;
        case 8:
            if (!is(Val::Kind::I32)) return mismatch();
            e.pupil_pos_valid = i32 != 0;
            break;
        case 9:
            if (!is(Val::Kind::I32)) return mismatch();
            e.flags[0] = i32;
            break;
        default:
            if (!is(Val::Kind::I32)) return mismatch();
            e.flags[1] = i32;
            break;
        }
    } else {
        auto& f = s.frame;
        const bool frame_field = std::ranges::find(FRAME_FIELDS, n) != FRAME_FIELDS.end();
        if (!frame_field) {
            return false;
        }
        bool ok = true;
        switch (n) {
        case field::TIMESTAMP:
            if ((ok = is(Val::Kind::I64))) s.timestamp_us = val.x;
            break;
        case field::COUNTER:
            if ((ok = is(Val::Kind::I32))) s.frame_counter = static_cast<std::uint32_t>(i32);
            break;
        case field::FIELD_14:
            if ((ok = is(Val::Kind::I32))) f.field_14 = i32;
            break;
        case field::AUX_POINT_1:
            if ((ok = is(Val::Kind::Vec3))) f.aux_points[0] = v3(val.a);
            break;
        case field::AUX_VALID_1:
            if ((ok = is(Val::Kind::I32))) f.aux_valid[0] = i32 != 0;
            break;
        case field::AUX_POINT_2:
            if ((ok = is(Val::Kind::Vec3))) f.aux_points[1] = v3(val.a);
            break;
        case field::AUX_VALID_2:
            if ((ok = is(Val::Kind::I32))) f.aux_valid[1] = i32 != 0;
            break;
        case field::FLAG_26:
            if ((ok = is(Val::Kind::I32))) f.flags[0] = i32;
            break;
        case field::FLAG_27:
            if ((ok = is(Val::Kind::I32))) f.flags[1] = i32;
            break;
        case field::FLAG_28:
            if ((ok = is(Val::Kind::I32))) f.flags[2] = i32;
            break;
        case field::FIELD_29:
            if ((ok = is(Val::Kind::F16))) f.field_29 = f16(val.x);
            break;
        case field::FIELD_34:
            if ((ok = is(Val::Kind::F16))) f.field_34 = f16(val.x);
            break;
        default: // FIELD_35
            if ((ok = is(Val::Kind::I32))) f.field_35 = i32;
            break;
        }
        if (!ok) {
            return mismatch();
        }
    }
    s.fields |= std::uint64_t{1} << n;
    return true;
}

std::expected<GazeSample, GazeError> parse_inner(std::span<const std::uint8_t> payload, bool strict) {
    if (payload.size() < 2) {
        return std::unexpected(simple(GazeError::Kind::TooShort));
    }
    Cursor cur{detail::Items(payload.subspan(2))};
    const auto first = cur.next();
    if (!first) {
        return std::unexpected(simple(GazeError::Kind::TooShort));
    }
    if (first->kind != VK::Compound || first->type_id != TYPE_GAZE) {
        return std::unexpected(simple(GazeError::Kind::NotGaze));
    }
    const std::uint16_t announced = first->count;
    GazeSample s;
    std::uint16_t found = 0;
    while (found < announced) {
        auto el = read_element(cur);
        if (!el) {
            return std::unexpected(el.error());
        }
        if (!*el) {
            break;
        }
        auto known = apply(s, (*el)->first, (*el)->second);
        if (!known) {
            return std::unexpected(known.error());
        }
        if (!*known) {
            s.unknown_fields++;
        }
        found++;
    }
    s.truncated = found < announced;
    if (strict) {
        if (s.truncated) {
            GazeError e;
            e.kind = GazeError::Kind::Truncated;
            e.found = found;
            e.announced = announced;
            return std::unexpected(e);
        }
        if (cur.next().has_value() || cur.cut) {
            GazeError e;
            e.kind = GazeError::Kind::Trailing;
            e.offset = cur.items.position();
            return std::unexpected(e);
        }
    } else if (!s.has(field::TIMESTAMP)) {
        return std::unexpected(simple(GazeError::Kind::TooShort));
    }
    for (std::size_t g = 0; g < s.eyes.size(); g++) {
        s.eyes[g].complete = (s.fields & group_mask(g)) == group_mask(g);
    }
    return s;
}

} // namespace

EyeState Eye::state() const {
    if (direction_valid) {
        return EyeState::Tracked;
    }
    if (origin_valid) {
        return EyeState::OriginOnly;
    }
    return EyeState::Lost;
}

std::optional<double> Eye::pupil_diameter() const {
    if (pupil_diameter_mm >= 0.0) {
        return pupil_diameter_mm;
    }
    return std::nullopt;
}

std::optional<double> Eye::openness() const {
    if (openness_raw >= 0.0) {
        return openness_raw;
    }
    return std::nullopt;
}

std::optional<std::pair<Vec3, Vec3>> Eye::ray() const {
    if (origin_valid && direction_valid) {
        return std::pair{origin_mm, direction};
    }
    return std::nullopt;
}

std::optional<std::pair<double, double>> Eye::yaw_pitch_deg() const {
    if (!direction_valid) {
        return std::nullopt;
    }
    const auto [x, y, z] = direction;
    return std::pair{detail::to_degrees(std::atan2(x, z)), detail::to_degrees(std::atan2(y, std::hypot(x, z)))};
}

bool GazeSample::is_complete() const {
    const auto all = all_mask();
    return (fields & all) == all;
}

std::string GazeError::message() const {
    switch (kind) {
    case Kind::TooShort:
        return "payload too short to hold a gaze sample";
    case Kind::NotGaze:
        return "payload does not start with a 0x..0bb8 container";
    case Kind::BadField:
        return std::format("field {}: {}", field, reason);
    case Kind::Truncated:
        return std::format("payload truncated: {} of {} elements", found, announced);
    case Kind::Trailing:
        return std::format("trailing items at offset {}", offset);
    }
    return {};
}

bool operator==(const GazeError& a, const GazeError& b) {
    if (a.kind != b.kind) {
        return false;
    }
    switch (a.kind) {
    case GazeError::Kind::BadField:
        return a.field == b.field && std::strcmp(a.reason, b.reason) == 0;
    case GazeError::Kind::Truncated:
        return a.found == b.found && a.announced == b.announced;
    case GazeError::Kind::Trailing:
        return a.offset == b.offset;
    default:
        return true;
    }
}

std::expected<GazeSample, GazeError> parse_payload(std::span<const std::uint8_t> payload) {
    return parse_inner(payload, true);
}

std::expected<GazeSample, GazeError> parse_payload_lenient(std::span<const std::uint8_t> payload) {
    return parse_inner(payload, false);
}

std::vector<std::uint8_t> encode_payload(const GazeSample& s) {
    using namespace detail;
    const auto raw16 = [](double x) { return sat_cast<std::int32_t>(std::round(x * 65536.0)); };
    const auto raw22 = [](double x) { return sat_cast<std::int64_t>(std::round(x * 4'398'046'511'104.0)); };
    std::vector<std::uint8_t> items;
    push_compound(items, static_cast<std::uint16_t>(WIRE_ORDER.size()), TYPE_GAZE);
    const auto vec3 = [&](const Vec3& v) {
        push_compound(items, 3, TYPE_VEC3_F16);
        for (double c : v) {
            push_fixed16_16(items, raw16(c));
        }
    };
    for (auto n : WIRE_ORDER) {
        push_compound(items, 2, TYPE_FIELD);
        push_i32(items, std::int32_t{n});
        if (const auto slot = eye_slot(n)) {
            const auto& e = s.eyes[slot->first];
            switch (slot->second) {
            case 0: vec3(e.origin_mm); break;
            case 1: push_i32(items, e.origin_valid ? 1 : 0); break;
            case 2: vec3(e.direction); break;
            case 3: push_i32(items, e.direction_valid ? 1 : 0); break;
            case 4: push_fixed16_16(items, raw16(e.pupil_diameter_mm)); break;
            case 5: push_fixed16_16(items, raw16(e.openness_raw)); break;
            case 6:
                push_compound(items, 2, TYPE_PAIR_F22);
                for (double c : e.aux_pair) {
                    push_fixed22_42(items, raw22(c));
                }
                break;
            case 7:
                push_compound(items, 2, TYPE_VEC2_F16);
                for (double c : e.pupil_pos) {
                    push_fixed16_16(items, raw16(c));
                }
                break;
            case 8: push_i32(items, e.pupil_pos_valid ? 1 : 0); break;
            case 9: push_i32(items, e.flags[0]); break;
            default: push_i32(items, e.flags[1]); break;
            }
        } else {
            const auto& f = s.frame;
            switch (n) {
            case field::TIMESTAMP: push_i64(items, s.timestamp_us); break;
            case field::COUNTER: push_i32(items, static_cast<std::int32_t>(s.frame_counter.value_or(0))); break;
            case field::FIELD_14: push_i32(items, f.field_14); break;
            case field::AUX_POINT_1: vec3(f.aux_points[0]); break;
            case field::AUX_VALID_1: push_i32(items, f.aux_valid[0] ? 1 : 0); break;
            case field::AUX_POINT_2: vec3(f.aux_points[1]); break;
            case field::AUX_VALID_2: push_i32(items, f.aux_valid[1] ? 1 : 0); break;
            case field::FLAG_26: push_i32(items, f.flags[0]); break;
            case field::FLAG_27: push_i32(items, f.flags[1]); break;
            case field::FLAG_28: push_i32(items, f.flags[2]); break;
            case field::FIELD_29: push_fixed16_16(items, raw16(f.field_29)); break;
            case field::FIELD_34: push_fixed16_16(items, raw16(f.field_34)); break;
            default: push_i32(items, f.field_35); break;
            }
        }
    }
    std::vector<std::uint8_t> payload{0, 0};
    payload.insert(payload.end(), items.begin(), items.end());
    return payload;
}

void GazeStats::add(const GazeSample& s) {
    samples++;
    for (std::size_t g = 0; g < s.eyes.size(); g++) {
        std::size_t i = 0;
        switch (s.eyes[g].state()) {
        case EyeState::Lost: i = 0; break;
        case EyeState::OriginOnly: i = 1; break;
        case EyeState::Tracked: i = 2; break;
        }
        states[g][i]++;
    }
    if (last_counter_ && s.frame_counter) {
        const std::uint32_t d = *s.frame_counter - *last_counter_; // wrapping
        if (d > 1 && d < (std::uint32_t{1} << 31)) {
            counter_gaps += d - 1;
        }
    }
    if (s.frame_counter) {
        last_counter_ = s.frame_counter;
    }
    if (last_ts) {
        const std::int64_t d = detail::wrapping_sub(s.timestamp_us, *last_ts);
        if (d <= 0) {
            timestamp_regressions++;
        } else if (d > NOMINAL_PERIOD_US * 3 / 2) { // d * 2 > 3 periods, without the overflow
            timestamp_gaps++;
        }
    }
    if (!first_ts) {
        first_ts = s.timestamp_us;
    }
    last_ts = s.timestamp_us;
}

std::optional<double> GazeStats::device_rate_hz() const {
    if (!first_ts || !last_ts) {
        return std::nullopt;
    }
    const auto a = *first_ts;
    const auto b = *last_ts;
    if (samples > 1 && b > a) {
        return static_cast<double>(samples - 1) / (static_cast<double>(b - a) * 1e-6);
    }
    return std::nullopt;
}

std::string format_line(const GazeSample& s) {
    std::string out = std::format("ts={:>12}us ctr={:>6}", s.timestamp_us,
                                  s.frame_counter ? std::to_string(*s.frame_counter) : std::string("-"));
    const char* names[2] = {"A", "B"};
    for (std::size_t i = 0; i < 2; i++) {
        const auto& e = s.eyes[i];
        out += std::format(" | {}: ", names[i]);
        switch (e.state()) {
        case EyeState::Tracked: {
            const auto yp = e.yaw_pitch_deg().value_or(std::pair{0.0, 0.0});
            out += std::format(
                "org({:+6.1f},{:+6.1f},{:+6.1f}) dir({:+.3f},{:+.3f},{:+.3f}) yaw{:+6.1f} pitch{:+6.1f} pupil{:5.2f}mm",
                e.origin_mm[0], e.origin_mm[1], e.origin_mm[2], e.direction[0], e.direction[1], e.direction[2],
                yp.first, yp.second, e.pupil_diameter_mm);
            break;
        }
        case EyeState::OriginOnly:
            out += std::format("org({:+6.1f},{:+6.1f},{:+6.1f}) no gaze", e.origin_mm[0], e.origin_mm[1],
                               e.origin_mm[2]);
            break;
        case EyeState::Lost:
            out += e.complete ? "invalid" : "n/a";
            break;
        }
    }
    return out;
}

} // namespace pibridge::eyemath
