// SPDX-License-Identifier: MIT
// The Tobii chip's own calibration (calibration.h) against the in-process FakeChip with scripted answers: the
// captured target table, the calibration guard (refused outside a session, only the nine payloads, exactly the five
// opcodes; frames::Guard still refuses all of them), the two passes in the captured order, the profile of the last
// download, and the live channel inside a running live::Tracker. No device node is opened.
#include "pibridge/eye/calibration.h"

#include "pibridge/eye/fake.h"
#include "pibridge/eye/frames.h"
#include "pibridge/eye/live.h"
#include "pibridge/eye/session.h"

#include "pibridge_test.h"
#include "testutil.h"

#include <thread>

using namespace pibridge::eye;
using namespace pibridge::eye::calibration;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {

// A synthetic profile body in the layout frames::check_profile wants (u32 LE len - 168, then anything).
Bytes synthetic_body(std::size_t n, std::uint8_t fill) {
    Bytes b(n, fill);
    const auto lead = static_cast<std::uint32_t>(n - 168);
    for (int i = 0; i < 4; i++) {
        b[static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(lead >> (8 * i));
    }
    return b;
}

fake::FakeChip cal_chip(const Bytes& profile) {
    fake::FakeChip c;
    c.start_on = std::nullopt;
    c.reply_payload[OP_DOWNLOAD] = frames::profile_payload(profile);
    c.reply_aux[OP_TARGET] = 0x11;
    c.reply_aux[OP_COMPUTE] = 0x22;
    c.in_max_body = 16384 - 8; // several IN frames for the download, as the chip sends it
    return c;
}

std::vector<ttp::Message> sent_messages(const fake::FakeChip& c) {
    ttp::StreamDecoder dec(ttp::Direction::Out);
    std::vector<ttp::Message> out;
    for (const auto& s : c.sent) {
        if (const auto* b = std::get_if<fake::SentBulkOut>(&s)) {
            for (auto& m : dec.push(b->data)) {
                if (m) {
                    out.push_back(std::move(*m));
                }
            }
        }
    }
    return out;
}

Bytes request_frame(std::uint32_t seq, const Step& s) {
    auto m = request(seq, s);
    CHECK(m.has_value());
    return m ? frame(*m) : Bytes{};
}

Bytes raw_frame(ttp::Header h, const Bytes& payload) {
    ttp::Message m;
    m.header = h;
    m.payload = payload;
    m.header.payload_len = static_cast<std::uint32_t>(payload.size());
    return frame(m);
}

} // namespace

TEST(the_table_has_the_nine_captured_targets) {
    const auto& t = targets();
    CHECK(t.size() == TARGET_COUNT);
    const std::array<std::array<std::int32_t, 2>, 9> want = {{{0, 0},
                                                             {-536, 536},
                                                             {536, -536},
                                                             {536, 536},
                                                             {-536, -536},
                                                             {-932, 0},
                                                             {932, 0},
                                                             {0, 840},
                                                             {0, -932}}};
    for (std::size_t i = 0; i < t.size() && i < want.size(); i++) {
        CHECK(t[i].x_mm == want[i][0]);
        CHECK(t[i].y_mm == want[i][1]);
        CHECK(t[i].z_mm == 2000);
        CHECK(t[i].payload.size() == TARGET_PAYLOAD_LEN);
        for (std::size_t j = 0; j < i; j++) {
            CHECK(t[i].payload != t[j].payload);
        }
    }
    // the compiled-in table is the data file
    CHECK(targets_text() == testutil::read_text(fs::path(PIBRIDGE_EYE_DATA_DIR) / "tobii-calibration-targets.txt"));
    // the first payload byte for byte (centre, eyes 3)
    CHECK(frames::hex(t[0].payload) ==
          "000004000000080000000000000000040000000800000000000000000400000008001f400000000000020000000400000003");
}

TEST(a_broken_table_is_refused) {
    const std::string good =
        "target x=0 y=0 z=2000 "
        "payload=000004000000080000000000000000040000000800000000000000000400000008001f400000000000020000000400000003\n";
    CHECK(parse_targets(good).has_value());
    CHECK(!parse_targets("# only comments\n").has_value());
    CHECK(!parse_targets("tarket x=0 y=0 z=2000 payload=00\n").has_value());
    // the line says x=1 but the bytes encode 0
    std::string moved = good;
    moved.replace(moved.find("x=0"), 3, "x=1");
    auto r = parse_targets(moved);
    CHECK(!r.has_value() && r.error().find("encodes") != std::string::npos);
    // a payload cut short, a non-hex payload
    std::string cut = good;
    cut.replace(cut.find("03\n"), 2, "");
    CHECK(!parse_targets(cut).has_value());
    std::string nothex = good;
    nothex.replace(nothex.find("payload=0000"), 12, "payload=zz00");
    CHECK(!parse_targets(nothex).has_value());
    CHECK(!parse_targets("target x=0 z=2000 payload=00\n").has_value());
}

TEST(requests_are_built_only_from_the_table) {
    const auto f = request(27, Step{OP_START, std::nullopt, 1});
    CHECK(f.has_value());
    CHECK(f->header.kind == ttp::Kind::Request && f->header.seq == 27 && f->header.status == 0 &&
          f->header.id == OP_START && f->header.aux == 0 && f->header.payload_len == 0 && f->payload.empty());
    const auto t = request(28, Step{OP_TARGET, 3, 1});
    CHECK(t.has_value() && t->payload == targets()[3].payload && t->header.payload_len == 50);
    CHECK(frame(*t).size() == 8 + 24 + 50);
    CHECK(!request(1, Step{OP_TARGET, std::nullopt, 1}).has_value());
    CHECK(!request(1, Step{OP_TARGET, 9, 1}).has_value());
    CHECK(!request(1, Step{OP_COMPUTE, 0, 1}).has_value());
    CHECK(!request(1, Step{0x456, std::nullopt, 1}).has_value());
}

TEST(the_guard_refuses_everything_outside_a_session) {
    const auto g = Guard::captured();
    CHECK(!g.is_open());
    fake::FakeChip c;
    for (const auto& s : sequence()) {
        const Bytes f = request_frame(40, s);
        CHECK(!g.check(f).has_value());
        auto r = g.bulk_out(c, f, 1s);
        CHECK(!r.has_value() && r.error().code == Errc::permission_denied);
    }
    CHECK(c.sent.empty());
    // the normal start gate keeps refusing the calibration opcodes, even when a frame is allowed explicitly
    CHECK((frames::DENIED_OPS == std::array<std::uint32_t, 5>{0x3f2, 0x407, 0x42e, 0x3fc, 0x44c}));
    auto steps = frames::parse(frames::base_list());
    CHECK(steps.has_value());
    auto fg = frames::Guard::make(*steps);
    CHECK(fg.has_value());
    for (const auto& s : sequence()) {
        const Bytes f = request_frame(41, s);
        fg->allow_frame(f);
        CHECK(!fg->bulk_out(c, f, 1s).has_value());
    }
    CHECK(c.sent.empty());
    // a session that was closed again refuses again
    auto g2 = Guard::captured();
    g2.open();
    CHECK(g2.check(request_frame(42, sequence()[0])).has_value());
    g2.close();
    CHECK(!g2.check(request_frame(42, sequence()[0])).has_value());
}

TEST(the_open_guard_lets_out_exactly_the_five_ops_and_the_nine_payloads) {
    auto g = Guard::captured();
    g.open();
    fake::FakeChip c;
    std::uint32_t seq = 27;
    for (const auto& s : sequence()) {
        CHECK(g.bulk_out(c, request_frame(seq++, s), 1s).has_value());
    }
    CHECK(c.sent.size() == sequence().size());
    const ttp::Header req{ttp::Kind::Request, 50, 0, OP_TARGET, 0, 0};
    // every captured payload passes, a changed byte does not
    for (const auto& t : targets()) {
        CHECK(g.check(raw_frame(req, t.payload)).has_value());
        Bytes p = t.payload;
        p[10] ^= 1;
        CHECK(!g.check(raw_frame(req, p)).has_value());
    }
    CHECK(!g.check(raw_frame(req, {})).has_value());
    // the payload-free ops with a payload, other opcodes, other headers
    for (const auto op : {OP_START, OP_COMPUTE, OP_DOWNLOAD, OP_FINISH}) {
        ttp::Header h = req;
        h.id = op;
        CHECK(g.check(raw_frame(h, {})).has_value());
        CHECK(!g.check(raw_frame(h, {0, 0})).has_value());
        CHECK(!g.check(raw_frame(h, targets()[0].payload)).has_value());
    }
    for (const std::uint32_t op : {0x456u, 0x4c4u, 0x4ceu, 0x76cu, 0x3e8u, 0x408u, 0x3f3u}) {
        ttp::Header h = req;
        h.id = op;
        CHECK(!g.check(raw_frame(h, {})).has_value());
    }
    ttp::Header bad = req;
    bad.status = 1;
    CHECK(!g.check(raw_frame(bad, targets()[0].payload)).has_value());
    bad = req;
    bad.aux = 5;
    CHECK(!g.check(raw_frame(bad, targets()[0].payload)).has_value());
    bad = req;
    bad.kind = ttp::Kind::Response;
    CHECK(!g.check(raw_frame(bad, targets()[0].payload)).has_value());
    // a request split into two frames, extra bytes after it, a cut frame
    auto m = request(60, Step{OP_TARGET, 0, 1});
    const auto two = m->to_frames(ttp::Direction::Out, 40);
    CHECK(two.size() == 2 && !g.check(two[0]).has_value() && !g.check(two[1]).has_value());
    Bytes longer = frame(*m);
    longer.push_back(0);
    CHECK(!g.check(longer).has_value());
    Bytes cut = frame(*m);
    cut.resize(20);
    CHECK(!g.check(cut).has_value());
}

TEST(a_calibration_runs_the_two_captured_passes_and_keeps_the_last_download) {
    const Bytes body = synthetic_body(33887, 0x5a);
    auto c = cal_chip(body);
    auto g = Guard::captured();
    g.open();
    ttp::StreamDecoder dec(ttp::Direction::In);
    Run run;
    std::uint32_t seq = FIRST_SEQ;
    std::vector<Answer> answers;
    while (const Step* s = run.current()) {
        auto a = exchange(c, g, dec, seq, *s, 2s);
        CHECK(a.has_value());
        if (!a) {
            break;
        }
        answers.push_back(*a);
        CHECK(run.answer(seq, *a).has_value());
        seq++;
    }
    CHECK(run.done() && !run.failed());
    CHECK(!run.pass_open());
    // what went out: the captured order, fresh sequence numbers from 27, the captured target payloads
    const auto sent = sent_messages(c);
    const std::vector<std::uint32_t> want = {0x3f2, 0x407, 0x407, 0x407, 0x407, 0x407, 0x42e, 0x44c, 0x3fc,
                                             0x3f2, 0x407, 0x407, 0x407, 0x407, 0x42e, 0x44c, 0x3fc, 0x44c};
    CHECK(sent.size() == want.size());
    std::size_t target = 0;
    for (std::size_t i = 0; i < sent.size() && i < want.size(); i++) {
        CHECK(sent[i].header.id == want[i]);
        CHECK(sent[i].header.seq == FIRST_SEQ + i);
        CHECK(sent[i].header.status == 0 && sent[i].header.aux == 0);
        if (want[i] == OP_TARGET) {
            CHECK(sent[i].payload == targets()[target].payload);
            target++;
        } else {
            CHECK(sent[i].payload.empty());
        }
    }
    CHECK(target == 9);
    CHECK(answers[1].aux == 0x11 && answers[6].aux == 0x22);
    CHECK(run.download_sizes().size() == 3 && run.download_sizes()[2] == body.size() + frames::PROFILE_PREFIX_LEN);
    CHECK(run.profile().has_value() && *run.profile() == body);
    // nothing else went out (no control transfer, no other frame)
    CHECK(c.sent.size() == want.size());
}

TEST(a_refused_step_or_a_broken_download_fails_the_run_and_the_open_pass_is_finished) {
    {
        auto c = cal_chip(synthetic_body(1000, 1));
        c.status_override[OP_TARGET] = 0;
        auto g = Guard::captured();
        g.open();
        ttp::StreamDecoder dec(ttp::Direction::In);
        Run run;
        CHECK(!run.abort_step().has_value());
        std::uint32_t seq = 30;
        auto a = exchange(c, g, dec, seq, *run.current(), 2s); // start
        CHECK(a && run.answer(seq, *a).has_value());
        CHECK(run.pass_open());
        seq++;
        a = exchange(c, g, dec, seq, *run.current(), 2s); // the chip refuses the target
        CHECK(!a.has_value() && a.error().find("status 0") != std::string::npos);
        Answer refused{seq, OP_TARGET, 0, 0, {}};
        CHECK(!run.answer(seq, refused).has_value());
        CHECK(run.failed() && run.current() == nullptr);
        const auto ab = run.abort_step();
        CHECK(ab.has_value() && ab->op == OP_FINISH);
        CHECK(exchange(c, g, dec, seq + 1, *ab, 2s).has_value());
    }
    {
        // the last download is not a profile
        Run run;
        const auto seqs = sequence();
        std::uint32_t seq = 1;
        for (std::size_t i = 0; i + 1 < seqs.size(); i++) {
            CHECK(run.answer(seq, Answer{seq, seqs[i].op, 1, 0, {}}).has_value());
            seq++;
        }
        auto r = run.answer(seq, Answer{seq, OP_DOWNLOAD, 1, 0, Bytes(100, 0)});
        CHECK(!r.has_value() && r.error().find("not a profile") != std::string::npos);
        CHECK(run.failed() && !run.profile().has_value());
    }
    {
        // an answer to another request or with another seq
        Run run;
        CHECK(!run.answer(5, Answer{5, OP_TARGET, 1, 0, {}}).has_value());
        Run run2;
        CHECK(!run2.answer(5, Answer{6, OP_START, 1, 0, {}}).has_value());
    }
    {
        // a broken profile body in a download that has the profile layout
        Bytes body = synthetic_body(2000, 2);
        body[0] ^= 1;
        auto p = downloaded_profile(Answer{1, OP_DOWNLOAD, 1, 0, frames::profile_payload(body)});
        CHECK(!p.has_value());
        auto ok = downloaded_profile(Answer{1, OP_DOWNLOAD, 1, 0, frames::profile_payload(synthetic_body(2000, 2))});
        CHECK(ok.has_value() && ok->size() == 2000);
    }
}

TEST(the_link_runs_a_calibration_inside_a_live_tracker_session) {
    const Bytes body = synthetic_body(30875, 0x33);
    auto fc = std::make_shared<fake::FakeChip>(cal_chip(body));
    fc->payload = live::fake_payload;
    fc->start_on = session::OP_SUBSCRIBE;
    auto link = std::make_shared<Link>();
    // nothing is allowed before a chip link exists
    CHECK(!link->attached());
    CHECK(!link->open_session().has_value());
    CHECK(!link->submit(sequence()[0]).has_value());
    live::Config cfg;
    cfg.pace = false;
    cfg.retry = 20ms;
    cfg.history = 64;
    std::atomic<int> opens{0};
    auto tr = live::Tracker::start_with(
        cfg,
        [&](const live::Config&, std::stop_token) -> std::expected<std::unique_ptr<Transport>, std::string> {
            opens++;
            return std::make_unique<Channel>(fc, link);
        },
        [](std::string_view) {});
    const auto t0 = Clock::now();
    while (tr->state() != live::State::Streaming && Clock::now() - t0 < 5s) {
        std::this_thread::sleep_for(10ms);
    }
    CHECK(tr->state() == live::State::Streaming);
    CHECK(link->attached());
    // the session is closed: submit refuses
    CHECK(!link->submit(sequence()[0]).has_value());
    CHECK(link->open_session().has_value());
    Run run;
    std::uint32_t first_seq = 0;
    while (const Step* s = run.current()) {
        auto t = link->submit(*s);
        CHECK(t.has_value());
        if (!t) {
            break;
        }
        const std::uint32_t seq = link->seq_of(*t);
        if (first_seq == 0) {
            first_seq = seq;
        }
        auto a = link->wait(*t, 3s);
        CHECK(a.has_value());
        if (!a) {
            break;
        }
        CHECK(run.answer(seq, *a).has_value());
    }
    CHECK(run.done() && run.profile().has_value() && *run.profile() == body);
    link->close_session();
    CHECK(!link->submit(sequence()[0]).has_value());
    const auto samples = tr->stats().samples;
    CHECK(samples > 0); // the gaze went on during the calibration
    tr->stop();
    CHECK(opens == 1);
    CHECK(!link->attached()); // the channel left with the link
    // the calibration requests continued the link's own numbering, after the start list's requests
    const auto sent = sent_messages(*fc);
    std::uint32_t max_start = 0;
    std::vector<std::uint32_t> cal_ops;
    for (const auto& m : sent) {
        if (is_calibration_op(m.header.id)) {
            cal_ops.push_back(m.header.id);
        } else if (cal_ops.empty() && m.header.id != session::OP_UNSUBSCRIBE) {
            max_start = std::max(max_start, m.header.seq);
        }
    }
    CHECK(cal_ops.size() == sequence().size());
    CHECK(first_seq == std::max(max_start + 1, FIRST_SEQ));
}

TEST(a_lost_link_fails_the_open_requests) {
    auto fc = std::make_shared<fake::FakeChip>();
    fc->start_on = std::nullopt;
    fc->silent.push_back(OP_TARGET);
    auto link = std::make_shared<Link>();
    {
        Channel ch(fc, link);
        CHECK(link->open_session().has_value());
        auto t0 = link->submit(Step{OP_START, std::nullopt, 1});
        CHECK(t0.has_value());
        CHECK(!link->poll(*t0).has_value()); // not sent yet
        (void)ch.bulk_in(0x83, 16384, 10ms); // sends it
        (void)ch.bulk_in(0x83, 16384, 10ms); // reads the answer
        auto a = link->poll(*t0);
        CHECK(a.has_value() && a->has_value() && (*a)->status == 1);
        auto t1 = link->submit(Step{OP_TARGET, 0, 1});
        CHECK(t1.has_value());
        (void)ch.bulk_in(0x83, 16384, 10ms);
        CHECK(!link->poll(*t1).has_value()); // the chip stays silent
        auto t2 = link->submit(Step{OP_COMPUTE, std::nullopt, 1});
        CHECK(t2.has_value());
        link->close_session(); // drops the unsent one
        auto r2 = link->poll(*t2);
        CHECK(r2.has_value() && !r2->has_value());
        CHECK(link->open_session().has_value());
        // the channel goes away (the tracker's link ended)
        auto keep = *t1;
        (void)keep;
    }
    CHECK(!link->attached());
    CHECK(!link->session_open());
    // the silent target request was failed by the detach
    // (its ticket is the second one handed out)
    auto r = link->poll(2);
    CHECK(r.has_value() && !r->has_value() && r->error().find("link ended") != std::string::npos);
}

PIBRIDGE_TEST_MAIN()
