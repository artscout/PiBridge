// SPDX-License-Identifier: MIT
// Ported from the frame loop of omarchyvr pimax-eyecal/src/xrapp.rs (MIT OR Apache-2.0).
#include "runner.h"

#include "report.h"

#include <chrono>
#include <format>
#include <thread>

namespace pibridge::eyecal {

namespace fs = std::filesystem;
namespace live = pibridge::eye::live;
using namespace std::chrono_literals;

std::string raw_csv_header() {
    return "t_s,tag,l_valid,l_x,l_y,l_z,l_ox,l_oy,l_oz,l_pupil,r_valid,r_x,r_y,r_z,r_ox,r_oy,r_oz,r_pupil,head_qx,head_qy,"
           "head_qz,head_qw";
}

std::string raw_csv_row(const RawSample& s, std::string_view tag, const std::optional<std::array<double, 4>>& head) {
    std::string row = std::format("{:.6f},{}", s.t, tag);
    for (const auto& e : s.eyes) {
        if (e) {
            row += std::format(",1,{:.6f},{:.6f},{:.6f},{:.5f},{:.5f},{:.5f},{:.3f}", e->dir[0], e->dir[1], e->dir[2],
                               e->origin[0], e->origin[1], e->origin[2], e->pupil_mm);
        } else {
            row += ",0,,,,,,,";
        }
    }
    if (head) {
        row += std::format(",{:.6f},{:.6f},{:.6f},{:.6f}", (*head)[0], (*head)[1], (*head)[2], (*head)[3]);
    } else {
        row += ",,,,";
    }
    return row;
}

Runner::Runner(Procedure proc, RunFiles files, std::shared_ptr<pibridge::eye::calibration::Link> link, StartTracker start,
               Print print, Service service)
    : proc_(std::move(proc)), files_(std::move(files)), link_(std::move(link)), start_(std::move(start)),
      print_(std::move(print)), service_(std::move(service)), driver_(link_), store_(files_.profile) {
    if (files_.raw_log) {
        std::error_code ec;
        fs::create_directories(files_.results_dir, ec);
        const fs::path p = files_.results_dir / std::format("eyecal-{}-raw.csv", files_.stamp);
        std::ofstream f(p, std::ios::trunc);
        if (f) {
            f << raw_csv_header() << '\n';
            raw_.emplace(std::move(f));
            print_("raw samples: " + p.string());
        } else {
            print_(std::format("warning: no raw log ({} cannot be written)", p.string()));
        }
    }
}

Runner::~Runner() { finish(); }

void Runner::begin() {
    if (!tracker_ && !finished_) {
        tracker_ = start_(proc_.s.conv);
    }
}

std::optional<std::optional<Cmd>> key_command(char c) {
    switch (c) {
    case ' ':
    case 'a':
    case '\n': return std::optional<Cmd>(Cmd::Next);
    case 'r':
    case 'b': return std::optional<Cmd>(Cmd::Redo);
    case 'y':
    case 'm': return std::optional<Cmd>(Cmd::Accept);
    case 'p': return std::optional<Cmd>(Cmd::Pursuit);
    case 'c': return std::optional<Cmd>(Cmd::Cursor);
    case 's': return std::optional<Cmd>(Cmd::Service);
    case 'q':
    case '\x1b': return std::optional<Cmd>{};
    default: return std::nullopt;
    }
}

Vec3 forward_of(const std::array<double, 4>& q) {
    const double x = q[0];
    const double y = q[1];
    const double z = q[2];
    const double w = q[3];
    // rotate (0, 0, -1)
    return {-(2.0 * (x * z + w * y)), -(2.0 * (y * z - w * x)), -(1.0 - 2.0 * (x * x + y * y))};
}

std::string Runner::tracker_state() const {
    return tracker_ ? std::string(live::state_name(tracker_->state())) : std::string("Stopped");
}

std::vector<std::string> Runner::hud_prefix() const {
    if (tracker_ && tracker_->state() != live::State::Streaming) {
        std::string s = tracker_state();
        for (auto& c : s) {
            c = static_cast<char>(c >= 'a' && c <= 'z' ? c - 'a' + 'A' : c);
        }
        return {"EYE TRACKER: " + s};
    }
    return {};
}

void Runner::step(double now, const std::optional<std::array<double, 4>>& head, const std::array<Vec3, 2>& eye_pos,
                  const std::optional<Vec3>& service, const std::vector<Cmd>& cmds) {
    head_ = head;
    std::vector<RawSample> batch;
    if (tracker_) {
        const auto st = tracker_->state();
        if (st != last_state_) {
            print_(std::format("eye tracker: {}", live::state_name(st)));
            last_state_ = st;
        }
        for (const auto& s : tracker_->samples_since(last_seq_)) {
            last_seq_ = s.seq;
            batch.push_back(RawSample::from_sample(s));
        }
        driver_.follow(st);
    }
    proc_.set_chip_ready(driver_.ready());
    if (raw_) {
        const std::string tag = proc_.tag();
        for (const auto& s : batch) {
            *raw_ << raw_csv_row(s, tag, head) << '\n';
        }
    }
    std::vector<Effect> fx = proc_.frame(now, head, eye_pos, batch, service);
    for (const auto c : cmds) {
        auto more = proc_.command(c);
        fx.insert(fx.end(), more.begin(), more.end());
    }
    handle(std::move(fx));
    while (auto r = driver_.poll()) {
        handle(proc_.chip_answer(r->first, r->second));
    }
}

void Runner::handle(std::vector<Effect> fx) {
    for (auto& e : fx) {
        switch (e.kind) {
        case Effect::Kind::Print: print_(e.text); break;
        case Effect::Kind::SaveResults: {
            const fs::path p = files_.results_dir / std::format("eyecal-{}.json", files_.stamp);
            if (auto r = write_with_backup(p, e.text, files_.stamp)) {
                print_("results: " + p.string());
            } else {
                print_("could not save the results: " + r.error());
            }
            break;
        }
        case Effect::Kind::SaveCalibration:
            if (auto r = write_with_backup(files_.calibration, e.text, files_.stamp)) {
                print_(std::format("calibration saved: {}{} (pibridge-eyed reads it at its next start)", files_.calibration.string(),
                                   *r ? ", previous kept as " + (*r)->string() : std::string()));
            } else {
                print_("could not save the calibration: " + r.error());
            }
            break;
        case Effect::Kind::RestartTracker:
            if (tracker_) {
                tracker_->stop();
            }
            last_seq_.reset();
            tracker_ = start_(e.conv);
            break;
        case Effect::Kind::StartService:
            if (tracker_) {
                tracker_->stop(); // unsubscribes and releases the chip
                tracker_.reset();
            }
            if (!service_ || !service_(true)) {
                print_("the runtime offers no eye gaze (XR_EXT_eye_gaze_interaction): the check cannot run");
            }
            break;
        case Effect::Kind::StopService:
            if (service_) {
                (void)service_(false);
            }
            std::this_thread::sleep_for(300ms);
            last_seq_.reset();
            tracker_ = start_(proc_.s.conv);
            break;
        case Effect::Kind::Quit: quit_ = true; break;
        case Effect::Kind::ChipRequest: driver_.request(e.step); break;
        case Effect::Kind::SaveProfile:
            if (auto r = store_.save(e.profile, files_.stamp)) {
                print_(std::format("the chip's new profile is saved to {}{}", store_.path().string(),
                                   *r ? " (the previous one is kept as " + (*r)->string() + ")" : std::string()));
            } else {
                print_("could not save the new profile: " + r.error());
            }
            break;
        case Effect::Kind::KeepProfile:
            if (store_.saved()) {
                store_.keep();
                print_("the new profile stays: pibridge-eyed uploads it at its next start");
            }
            break;
        }
    }
}

void Runner::finish() {
    if (finished_) {
        return;
    }
    finished_ = true;
    handle(proc_.shutdown());
    // the answer to the finish of an open pass (or whatever is still out), briefly
    const auto until = std::chrono::steady_clock::now() + 1500ms;
    while (driver_.outstanding() > 0 && std::chrono::steady_clock::now() < until) {
        while (auto r = driver_.poll()) {
            for (auto& e : proc_.chip_answer(r->first, r->second)) {
                if (e.kind == Effect::Kind::Print) {
                    print_(e.text);
                }
            }
        }
        std::this_thread::sleep_for(10ms);
    }
    if (tracker_) {
        tracker_->stop();
        tracker_.reset();
    }
    if (store_.saved() && !store_.kept()) {
        if (auto r = store_.rollback()) {
            print_("not saved: " + *r);
        } else {
            print_("could not put the previous profile back: " + r.error());
        }
    }
    if (raw_) {
        raw_->flush();
    }
}

} // namespace pibridge::eyecal
