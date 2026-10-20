// SPDX-License-Identifier: MIT
//
// The Tobii chip's own calibration, as Pimax Play runs it (Windows capture "take calibrate" of 2026-10-18, three runs;
// omarchyvr analysis/crystal-super-eye-20261018.md, "The calibration is the chip's own").
//
// One calibration is two passes over the bulk pipe, every request answered with status 1:
//
//   pass 1: 0x3f2 start | 0x407 x 5 (centre + 4 diagonals) | 0x42e compute | 0x44c download | 0x3fc finish
//   pass 2: 0x3f2 start | 0x407 x 4 (the edges)            | 0x42e compute | 0x44c download | 0x3fc finish | 0x44c
//
// 0x3f2, 0x42e, 0x44c and 0x3fc carry no payload. 0x407 carries one target: x, y, z in millimetres (S22.42 fixed
// point; z = 2000) and the eye mask (3 = both), 50 bytes, byte-identical for the same target in every run. The chip
// answers a 0x407 after about 165 ms (it collects the gaze while the wearer looks at the target), 0x42e after about
// 500 ms ("computed calibration for both eyes" in Tobii's log: from then on the chip uses the new calibration), and
// 0x44c with the new profile in the layout of the 0x456 upload (`00 00 15 | BE len+4 | BE len | body`). The last
// download is the profile the client stores and uploads at the next chip start.
//
// Safety model (docs/CONVENTIONS.md: "calibration opcodes refused unless a later, explicit feature allows them"):
// frames::Guard keeps refusing every calibration opcode (frames::DENIED_OPS). Only calibration::Guard lets them out, and
// only while a calibration session is explicitly open: exactly the five opcodes, with an empty payload, except 0x407
// whose payload must be one of the nine captured target payloads (data/tobii-calibration-targets.txt, compiled in).
// Requests are built only from that table; the header is the usual request header with a fresh sequence number.
//
// Two ways to talk to the chip:
// - exchange(): one request and its answer on a transport nothing else reads (tests, a direct session).
// - Link + Channel: the calibration runs inside a live::Tracker session (the gaze stream goes on, as under Windows,
//   and the calibration tool needs it to see where the wearer looks). Channel is a Transport between the tracker and
//   the chip: it forwards everything, sends the queued calibration requests from the tracker's own thread (inside
//   its bulk IN reads, so the pipe has one user), and picks the answers out of the IN stream. Link is the caller's
//   side: open the session, submit a step, poll or wait for its answer.
#pragma once

#include "pibridge/eye/error.h"
#include "pibridge/eye/transport.h"
#include "pibridge/eye/ttp.h"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <expected>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace pibridge::eye::calibration {

/// Start a calibration pass (Tobii "calibration start").
inline constexpr std::uint32_t OP_START = 0x3f2;
/// Collect one target.
inline constexpr std::uint32_t OP_TARGET = 0x407;
/// Compute the calibration from the collected targets (the chip uses it from then on).
inline constexpr std::uint32_t OP_COMPUTE = 0x42e;
/// Download the profile (the answer carries it in the 0x456 layout).
inline constexpr std::uint32_t OP_DOWNLOAD = 0x44c;
/// Finish the pass.
inline constexpr std::uint32_t OP_FINISH = 0x3fc;
/// Every opcode the calibration guard lets out (the same five as frames::DENIED_OPS).
inline constexpr std::array<std::uint32_t, 5> OPS = {OP_START, OP_TARGET, OP_COMPUTE, OP_DOWNLOAD, OP_FINISH};
/// Whether `op` is one of OPS.
bool is_calibration_op(std::uint32_t op);
/// "start", "target", "compute", "download", "finish" (or "0x...").
std::string op_name(std::uint32_t op);

/// Payload length of every captured 0x407 request.
inline constexpr std::size_t TARGET_PAYLOAD_LEN = 50;
/// Sequence number of the first calibration request when the link sent nothing before (the capture's own: Pimax
/// Play's counter after its 26 start requests).
inline constexpr std::uint32_t FIRST_SEQ = 27;

/// One captured target request.
struct Target {
    /// Position in the chip's frame, millimetres, as the comment line of the table says (and the payload encodes).
    std::int32_t x_mm = 0;
    std::int32_t y_mm = 0;
    std::int32_t z_mm = 0;
    /// The captured 0x407 payload (2-byte prefix and items), byte for byte.
    Bytes payload;
    bool operator==(const Target&) const = default;
};

/// The table compiled in from `libpibridge-eye/data/tobii-calibration-targets.txt`.
std::string_view targets_text();
/// Parse a table (`target x=.. y=.. z=.. payload=<hex>` lines, `#` comments). Each payload must be 50 bytes in the
/// captured layout and encode the x, y, z of its line (S22.42 mm, within 0.01 mm).
std::expected<std::vector<Target>, std::string> parse_targets(std::string_view text);
/// The nine captured targets (the compiled-in table, parsed once). Pass 1 uses 0..4, pass 2 uses 5..8.
const std::vector<Target>& targets();
/// Number of captured targets.
inline constexpr std::size_t TARGET_COUNT = 9;
/// Targets of each pass, in the captured order.
inline constexpr std::array<std::size_t, 5> PASS1_TARGETS = {0, 1, 2, 3, 4};
inline constexpr std::array<std::size_t, 4> PASS2_TARGETS = {5, 6, 7, 8};

/// One request of the calibration.
struct Step {
    /// Opcode (one of OPS).
    std::uint32_t op = 0;
    /// Index into targets() for OP_TARGET.
    std::optional<std::size_t> target;
    /// 1 or 2.
    std::uint8_t pass = 1;
    bool operator==(const Step&) const = default;
};

/// The captured sequence of one calibration: the two passes above (18 requests).
std::vector<Step> sequence();

/// The request of `step` with sequence number `seq`: request header (status 0, aux 0), the captured payload for a
/// target, nothing for the other opcodes. Fails for an opcode outside OPS or a target index outside the table.
std::expected<ttp::Message, std::string> request(std::uint32_t seq, const Step& step);
/// The single bulk OUT frame of a request (every calibration request fits one 4087-byte frame).
Bytes frame(const ttp::Message& m);

/// A chip answer.
struct Answer {
    std::uint32_t seq = 0;
    std::uint32_t op = 0;
    std::uint32_t status = 0;
    /// Header word 16 (a handle-like value in the answers to 0x407 / 0x42e).
    std::uint32_t aux = 0;
    /// Response payload (the profile download for 0x44c).
    Bytes payload;
    bool operator==(const Answer&) const = default;
};

/// The calibration send gate. Closed by default; open() is the explicit "a calibration runs now".
class Guard {
public:
    /// The gate for the compiled-in target table.
    static Guard captured();
    /// The gate for another table (tests).
    static Guard with_targets(std::span<const Target> t);

    /// Allow calibration requests from now on.
    void open() { open_ = true; }
    /// Refuse them again.
    void close() { open_ = false; }
    bool is_open() const { return open_; }

    /// Why `frame` may not go out (empty: it may). The frame must be one whole request in one bulk OUT frame
    /// (wrapper length = body length, payload length = the bytes after the header), request header with status 0
    /// and aux 0, an opcode of OPS; 0x407 with one of the captured payloads, the others with none; the gate open.
    std::expected<void, std::string> check(ByteView frame) const;
    /// Bulk OUT (EP 0x05) through the gate.
    Result<void> bulk_out(Transport& t, ByteView frame, Duration timeout) const;

private:
    Guard() = default;
    std::set<Bytes> payloads_;
    bool open_ = false;
};

/// The answer of one request, checked: the right opcode and sequence number, status 1.
std::expected<void, std::string> check_answer(const Step& step, std::uint32_t seq, const Answer& a);

/// The profile body of a 0x44c answer, checked (profile layout and frames::check_profile).
std::expected<Bytes, std::string> downloaded_profile(const Answer& a);

/// Where a calibration is in the captured sequence. Pure: the caller sends current(), hands the answer to
/// answer(), and sends the next step. A 0x407 should be sent only while the wearer looks at that target.
class Run {
public:
    Run();
    /// The step to send next; nullptr when the calibration is complete (or failed).
    const Step* current() const;
    /// Index of current() in sequence().
    std::size_t index() const { return i_; }
    /// All steps answered.
    bool done() const { return i_ >= steps_.size(); }
    /// The calibration failed (a bad answer); error() says why.
    bool failed() const { return !error_.empty(); }
    const std::string& error() const { return error_; }
    /// Account for the answer to current() (sent with `seq`). A bad answer (other opcode or seq, status other than
    /// 1, a final download that is not a profile) fails the run.
    std::expected<void, std::string> answer(std::uint32_t seq, const Answer& a);
    /// A pass is open (its 0x3f2 was answered and its 0x3fc not yet): aborting must send OP_FINISH.
    bool pass_open() const { return pass_open_; }
    /// The step that ends an open pass on abort (OP_FINISH), when one is open.
    std::optional<Step> abort_step() const;
    /// The validated profile of the last download, once the run is done.
    const std::optional<Bytes>& profile() const { return profile_; }
    /// Payload size of every download so far, in order.
    const std::vector<std::size_t>& download_sizes() const { return downloads_; }

private:
    std::vector<Step> steps_;
    std::size_t i_ = 0;
    bool pass_open_ = false;
    std::string error_;
    std::optional<Bytes> profile_;
    std::vector<std::size_t> downloads_;
};

/// How long to wait for the answer of `op` (0x407 3 s, 0x42e and 0x44c 10 s, others 3 s: the capture took 0.17,
/// 0.5 and 0.09 s).
Duration answer_wait(std::uint32_t op);

/// Send one request through `g` and read until its answer (other messages go to `other`, gaze notifications too).
/// For a transport nothing else reads (tests, a direct session); `dec` is the IN decoder of that transport.
std::expected<Answer, std::string> exchange(Transport& t, const Guard& g, ttp::StreamDecoder& dec, std::uint32_t seq,
                                            const Step& step, Duration wait,
                                            const std::function<void(const ttp::Message&)>& other = {});

class Channel;

/// The caller's side of a calibration inside a live session (see the file comment). Thread-safe.
class Link {
public:
    explicit Link(Guard g = Guard::captured());

    /// A Channel is attached (a chip link is up).
    bool attached() const;
    /// Open the calibration session: from now on submit() is allowed. Fails without a link.
    std::expected<void, std::string> open_session();
    /// Close it: the gate refuses again; requests not sent yet are dropped (their tickets fail).
    void close_session();
    bool session_open() const;

    /// Queue `step` for sending; the ticket names its answer. Refused when no session is open or no link is up.
    std::expected<std::uint64_t, std::string> submit(const Step& step);
    /// The answer of `ticket` once it is there (or why there will be none: refused by the gate, link lost, no
    /// answer in time); nullopt while it is still on its way. The answer is checked with check_answer().
    std::optional<std::expected<Answer, std::string>> poll(std::uint64_t ticket);
    /// poll() until there is a result or `timeout` passed (then an error).
    std::expected<Answer, std::string> wait(std::uint64_t ticket, Duration timeout);
    /// The sequence number `ticket` was (or will be) sent with; 0 for an unknown ticket.
    std::uint32_t seq_of(std::uint64_t ticket) const;

private:
    friend class Channel;
    struct Pending {
        Step step;
        std::uint32_t seq = 0;
        Bytes frame;
        bool sent = false;
        Clock::time_point sent_at{};
        std::optional<std::expected<Answer, std::string>> result;
    };

    // Channel side (under mu_).
    void attach(std::uint64_t id);
    void detach(std::uint64_t id);
    void note_out(ByteView frame);
    // The requests to send now and the gate they go through (a copy, so the send happens outside the lock).
    struct Outbox {
        Guard guard;
        std::vector<std::pair<std::uint64_t, Bytes>> frames;
    };
    std::optional<Outbox> take_unsent();
    void sent(std::uint64_t ticket, std::expected<void, std::string> r);
    void deliver(const ttp::Message& m);
    void expire_locked(Clock::time_point now);

    mutable std::mutex mu_;
    std::condition_variable cv_;
    Guard guard_;
    std::optional<std::uint64_t> channel_;
    std::uint32_t max_seq_ = 0;
    std::uint32_t next_seq_ = 0;
    std::uint64_t next_ticket_ = 1;
    std::map<std::uint64_t, Pending> pending_;
};

/// The Transport between a live::Tracker and the chip (see the file comment). One Channel per chip link: it attaches
/// to the Link when made and detaches when destroyed (open requests then fail with "link lost").
class Channel final : public Transport {
public:
    Channel(std::shared_ptr<Transport> inner, std::shared_ptr<Link> link);
    ~Channel() override;
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    Result<Bytes> control(const Setup& setup, ByteView out, Duration timeout) override;
    /// Forwarded; the request's sequence number is noted (calibration requests continue after it).
    Result<void> bulk_out(std::uint8_t ep, ByteView data, Duration timeout) override;
    /// Sends the queued calibration requests (through the calibration guard), then reads; answers to calibration
    /// requests are handed to the Link. The bytes are returned unchanged.
    Result<std::optional<Bytes>> bulk_in(std::uint8_t ep, std::size_t max_len, Duration timeout) override;

private:
    std::shared_ptr<Transport> inner_;
    std::shared_ptr<Link> link_;
    std::uint64_t id_;
    ttp::StreamDecoder dec_{ttp::Direction::In};
};

} // namespace pibridge::eye::calibration
