// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/tlv.rs tests (MIT OR Apache-2.0).
#include "pibridge/eye/tlv.h"

#include "pibridge_test.h"

#include <cstdint>
#include <vector>

using namespace pibridge::eye::tlv;
using B = std::vector<std::uint8_t>;

namespace {
View view(const B& b) {
    return View{b};
}
} // namespace

TEST(decodes_every_known_tag) {
    B b;
    push_i32(b, -7);
    push_fixed16_16(b, -65536);
    push_fixed22_42(b, -(std::int64_t{1} << 42));
    push_compound(b, 3, 0x1f42);
    push_i64(b, 50'376'465);
    push_item(b, tag::STRING, B{0, 0, 0, 2, 'h', 'i'});
    push_item(b, tag::BLOB, B{0, 0, 0, 1, 0xaa});
    push_item(b, tag::ARRAY, B{0, 0, 0, 1, 1, 2, 3, 4});
    push_item(b, tag::UINT32, B{0xff, 0, 0, 1});
    push_item(b, tag::ARRAY_FIXED_22_42, B{0, 0, 0, 1, 1, 2, 3, 4, 5, 6, 7, 8});
    push_item(b, 0x77, B{9});
    std::vector<Value> v;
    Items it(b);
    while (auto r = it.next()) {
        CHECK(r->has_value());
        v.push_back((*r)->decode());
    }
    const B hi = {'h', 'i'}, aa = {0xaa}, a4 = {1, 2, 3, 4}, a8 = {1, 2, 3, 4, 5, 6, 7, 8}, nine = {9};
    const std::vector<Value> want = {
        Int32{-7},
        Fixed16_16{-65536},
        Fixed22_42{-(std::int64_t{1} << 42)},
        Compound{3, 0x1f42},
        Int64{50'376'465},
        Str{view(hi)},
        Blob{view(aa)},
        Array{1, view(a4)},
        Uint32{0xff000001u},
        ArrayFixed22_42{1, view(a8)},
        Unknown{0x77, view(nine)},
    };
    CHECK(v == want);
}

TEST(wrong_lengths_become_unknown) {
    // tag 2 with 3 bytes, string whose prefix disagrees, array whose count disagrees
    const std::vector<std::pair<std::uint8_t, B>> cases = {
        {tag::INT32, {0, 0, 1}},
        {tag::STRING, {0, 0, 0, 5, 'a'}},
        {tag::ARRAY, {0, 0, 0, 2, 1, 2, 3, 4}},
        {tag::ARRAY_FIXED_22_42, {0, 0, 0, 1, 1, 2, 3, 4}},
        {tag::UINT32, B(8, 0)},
        {tag::INT64, B(4, 0)},
        {tag::COMPOUND, B(8, 0)},
    };
    for (const auto& [t, val] : cases) {
        B b;
        push_item(b, t, val);
        auto it = Items(b).next();
        CHECK(it && it->has_value());
        CHECK(std::holds_alternative<Unknown>((*it)->decode()));
    }
}

TEST(truncation_is_reported_once) {
    B b;
    push_i32(b, 1);
    push_i32(b, 2);
    for (std::size_t cut = 1; cut < b.size(); cut++) {
        Items it(std::span<const std::uint8_t>(b).first(cut));
        auto first = it.next();
        if (cut == 9) {
            // clean end after the first item
            CHECK(first && first->has_value());
            CHECK(!it.next());
            continue;
        } else if (cut > 9) {
            CHECK(first && first->has_value());
            auto second = it.next();
            CHECK(second && !second->has_value());
        } else {
            CHECK(first && !first->has_value());
        }
        CHECK(!it.next()); // the iterator must stop after an error
    }
}

TEST(huge_length_does_not_overflow) {
    const B b = {0x02, 0xff, 0xff, 0xff, 0xff, 0, 0};
    auto r = Items(b).next();
    CHECK(r && !r->has_value());
    CHECK(r->error().needed >= 0xffffffffu);
}

TEST(fixed_point_conversions) {
    CHECK(fixed16_16_to_f64(65536) == 1.0);
    CHECK(fixed16_16_to_f64(-65536) == -1.0);
    CHECK(fixed16_16_to_f64(0x00030000 + 0x8000) == 3.5);
    CHECK(fixed22_42_to_f64(-(std::int64_t{1} << 42)) == -1.0);
    CHECK(fixed22_42_to_f64(std::int64_t{1} << 47) == 32.0);
}

// From tests/props.rs (tlv_iteration_is_total, tlv_roundtrip), with a fixed pseudo-random sample instead of proptest.
TEST(tlv_iteration_is_total_and_roundtrips) {
    std::uint32_t x = 0x9e3779b9u;
    const auto rnd = [&] {
        x = x * 1664525u + 1013904223u;
        return x >> 24;
    };
    for (int round = 0; round < 300; round++) {
        B garbage(static_cast<std::size_t>(rnd()) + static_cast<std::size_t>(round % 50));
        for (auto& c : garbage) {
            c = static_cast<std::uint8_t>(rnd());
        }
        Items it(garbage);
        std::size_t n = 0;
        bool err = false;
        while (auto r = it.next()) {
            CHECK(!err); // nothing after an error
            if (!r->has_value()) {
                err = true;
            } else {
                (void)(*r)->decode();
            }
            CHECK(++n <= garbage.size() + 1);
        }
        // roundtrip: items pushed are read back in order
        B items;
        std::vector<std::pair<std::uint8_t, B>> want;
        for (std::uint32_t i = 0; i < rnd() % 20; i++) {
            B v(rnd() % 40);
            for (auto& c : v) {
                c = static_cast<std::uint8_t>(rnd());
            }
            const auto t = static_cast<std::uint8_t>(rnd());
            push_item(items, t, v);
            want.emplace_back(t, v);
        }
        Items back(items);
        std::size_t k = 0;
        while (auto r = back.next()) {
            CHECK(r->has_value());
            CHECK(k < want.size());
            if (k < want.size()) {
                CHECK((*r)->tag == want[k].first);
                CHECK(std::ranges::equal((*r)->value, want[k].second));
            }
            k++;
        }
        CHECK(k == want.size());
        CHECK(back.position() == items.size());
    }
}

PIBRIDGE_TEST_MAIN()
