// SPDX-License-Identifier: MIT
// Ported from omarchyvr pimax-eye/src/session.rs (MIT OR Apache-2.0).
#include "pibridge/eye/session.h"

#include <algorithm>
#include <format>

namespace pibridge::eye::session {

using namespace std::chrono_literals;
using replay::EP_BULK_IN;
using replay::EP_BULK_OUT;

std::string ReplayError::to_string() const {
    switch (kind) {
    case Kind::EmptyPlan: return "the capture holds no vendor control or TTP request transfer for this device";
    case Kind::Incomplete: {
        std::string s =
            std::format("refusing to replay: {} step(s) are cut by the capture snap length:", incomplete.size());
        for (const auto& i : incomplete) {
            s += std::format("\n  step {}: {} ({} of {} bytes missing)", i.step, i.what, i.missing, i.total);
        }
        return s;
    }
    case Kind::Io: return std::format("step {} ({}): {}", step, what, io.to_string());
    case Kind::NoResponse:
        return std::format("step {}: no response to request seq {} opcode {:#x} ({})", step, seq, opcode,
                           ttp::opcode_name(opcode));
    case Kind::BadStatus:
        return std::format("step {}: request seq {} opcode {:#x} answered with status {}, Windows saw {}", step, seq,
                           opcode, got, expected);
    case Kind::Framing: return "IN stream framing error: " + framing.to_string();
    }
    return "replay error";
}

std::expected<ReplayReport, ReplayError> run_plan(Transport& t, const replay::Plan& plan, const ReplayOptions& opts,
                                                  ttp::StreamDecoder& dec, const LogFn& log) {
    const auto say = [&](const std::string& s) {
        if (log) {
            log(s);
        }
    };
    if (plan.steps.empty()) {
        return std::unexpected(ReplayError{});
    }
    if (!opts.allow_incomplete && !plan.is_complete()) {
        ReplayError e;
        e.kind = ReplayError::Kind::Incomplete;
        for (std::size_t i = 0; i < plan.steps.size(); i++) {
            const auto& s = plan.steps[i];
            if (!s.is_complete()) {
                e.incomplete.push_back({i, s.label(), s.missing_bytes, s.total_bytes});
            }
        }
        return std::unexpected(e);
    }
    ReplayReport report;
    for (std::size_t i = 0; i < plan.steps.size(); i++) {
        const auto& step = plan.steps[i];
        if (!step.is_complete()) {
            say(std::format("step {}: SKIPPED (incomplete in the capture): {}", i, step.label()));
            report.skipped.push_back(i);
            continue;
        }
        const auto io_err = [&](Error e) {
            ReplayError r;
            r.kind = ReplayError::Kind::Io;
            r.step = i;
            r.what = step.label();
            r.io = std::move(e);
            return std::unexpected(r);
        };
        if (const auto* c = std::get_if<replay::ControlOut>(&step.kind)) {
            if (auto r = t.control(c->setup, c->data, opts.timeout); !r) {
                return io_err(r.error());
            }
        } else if (const auto* ci = std::get_if<replay::ControlIn>(&step.kind)) {
            auto r = t.control(ci->setup, {}, opts.timeout);
            if (!r) {
                return io_err(r.error());
            }
            report.control_in.emplace_back(i, r->size());
        } else {
            const auto& req = std::get<replay::Request>(step.kind);
            for (const auto& f : req.frames) {
                if (auto r = t.bulk_out(EP_BULK_OUT, f, opts.timeout); !r) {
                    return io_err(r.error());
                }
            }
            const auto sent_at = Clock::now();
            const auto deadline = sent_at + opts.timeout;
            std::optional<ttp::Message> resp;
            while (!resp) {
                const auto now = Clock::now();
                if (now >= deadline) {
                    ReplayError e;
                    e.kind = ReplayError::Kind::NoResponse;
                    e.step = i;
                    e.seq = req.seq;
                    e.opcode = req.opcode;
                    return std::unexpected(e);
                }
                const Duration wait = std::min<Duration>(deadline - now, 250ms);
                auto got = t.bulk_in(EP_BULK_IN, IN_BUFFER, wait);
                if (!got) {
                    return io_err(got.error());
                }
                if (!*got) {
                    continue;
                }
                std::optional<ttp::Message> found;
                for (auto& r : dec.push(**got)) {
                    if (!r) {
                        ReplayError e;
                        e.kind = ReplayError::Kind::Framing;
                        e.framing = r.error();
                        return std::unexpected(e);
                    }
                    if (r->header.kind == ttp::Kind::Response && r->header.seq == req.seq) {
                        found = std::move(*r);
                    } else if (r->header.kind == ttp::Kind::Notification && r->header.id == ttp::STREAM_GAZE) {
                        report.notifications++;
                    }
                }
                resp = std::move(found);
            }
            if (req.expect && resp->header.status != req.expect->status) {
                ReplayError e;
                e.kind = ReplayError::Kind::BadStatus;
                e.step = i;
                e.seq = req.seq;
                e.opcode = req.opcode;
                e.got = resp->header.status;
                e.expected = req.expect->status;
                return std::unexpected(e);
            }
            const auto rt = Clock::now() - sent_at;
            say(std::format("step {}: seq {} op {:#05x} -> status {} payload {} ({} ms)", i, req.seq, req.opcode,
                            resp->header.status, resp->header.payload_len,
                            std::chrono::duration_cast<std::chrono::milliseconds>(rt).count()));
            report.exchanges.push_back(
                {i, req.seq, req.opcode, resp->header.status, resp->header.payload_len, Clock::now() - sent_at});
        }
        report.sent++;
    }
    return report;
}

std::optional<double> StreamSummary::host_rate_hz() const {
    if (samples > 1 && sample_span.count() != 0) {
        return static_cast<double>(samples - 1) / std::chrono::duration<double>(sample_span).count();
    }
    return std::nullopt;
}

Result<StreamSummary> read_stream(Transport& t, ttp::StreamDecoder& dec, Duration duration,
                                  const GazeHandler& on_gaze) {
    const auto start = Clock::now();
    StreamSummary s;
    std::optional<Duration> first_at;
    Duration last_at{};
    const auto elapsed = [&] { return Duration(Clock::now() - start); };
    while (elapsed() < duration) {
        const Duration left = std::max<Duration>(duration - elapsed(), Duration::zero());
        const Duration wait = std::max<Duration>(std::min<Duration>(left, 100ms), 1ms);
        auto got = t.bulk_in(EP_BULK_IN, IN_BUFFER, wait);
        if (!got) {
            return std::unexpected(got.error());
        }
        if (!*got) {
            s.timeouts++;
            continue;
        }
        const Duration at = elapsed();
        s.bytes += (*got)->size();
        s.transfers++;
        for (auto& r : dec.push(**got)) {
            if (!r) {
                s.framing_errors++;
                continue;
            }
            const auto& m = *r;
            if (m.header.kind != ttp::Kind::Notification || m.header.id != ttp::STREAM_GAZE) {
                s.other_messages++;
                continue;
            }
            std::expected<void, std::string> ok;
            if (on_gaze) {
                ok = on_gaze(m, at);
            }
            if (ok) {
                s.samples++;
                if (!first_at) {
                    first_at = at;
                }
                last_at = at;
            } else {
                s.parse_errors++;
                if (!s.first_parse_error) {
                    s.first_parse_error = ok.error();
                }
            }
        }
    }
    s.elapsed = elapsed();
    s.first_sample_after = first_at;
    const Duration f = first_at.value_or(Duration::zero());
    s.sample_span = last_at > f ? last_at - f : Duration::zero();
    return s;
}

ttp::StreamDecoder in_decoder() {
    return ttp::StreamDecoder(ttp::Direction::In);
}

} // namespace pibridge::eye::session
