// SPDX-License-Identifier: MIT
// Ported from ZoneTweak chap.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
#include "pibridge/room/chap.h"

#include "pibridge/room/files.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <sys/stat.h>

namespace fs = std::filesystem;

namespace pibridge::room {

namespace {

double r6(double v) { return std::round(v * 1e6) / 1e6; }

std::string lower(std::string s) {
    for (char& c : s) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return s;
}

std::time_t mtimeOf(const std::string& p) {
    struct stat st{};
    return ::stat(p.c_str(), &st) == 0 ? st.st_mtime : 0;
}

Standing standingOf(const Json& u, const char* key) {
    const Json& st = u.get(key);
    const Json& tr = st.get("translation");
    Standing s;
    if (tr.size() >= 3) {
        s.x = tr.at(0).num();
        s.y = tr.at(1).num();
        s.z = tr.at(2).num();
    } else if (tr.size() >= 2) {
        s.y = tr.at(1).num();
    }
    s.yaw = st.get("yaw").num();
    return s;
}

/// ZoneTweak standingFromRigid: the standing pose of a rigid map standing -> raw.
Standing standingFromRigid(const Rigid2& r, double y) { return {r.tx, y, r.tz, -std::atan2(r.c, r.a)}; }

} // namespace

const char* frameModeName(FrameMode m) {
    switch (m) {
    case FrameMode::A: return "A";
    case FrameMode::B: return "B";
    case FrameMode::Off: return "OFF";
    }
    return "?";
}

FrameMode defaultFrameMode(bool pimax) { return pimax ? FrameMode::A : FrameMode::Off; }

std::vector<Json*> universesOf(Json& doc) {
    std::vector<Json*> out;
    if (!doc.isObject() || !doc.has("universes")) {
        return out;
    }
    for (Json& u : doc["universes"].items()) {
        if (u.isObject()) {
            out.push_back(&u);
        }
    }
    return out;
}

std::vector<const Json*> universesOf(const Json& doc) {
    std::vector<const Json*> out;
    for (const Json& u : doc.get("universes").items()) {
        if (u.isObject()) {
            out.push_back(&u);
        }
    }
    return out;
}

std::string universeId(const Json& u) { return u.get("universeID").str(); }

bool isPimax(const Json& u) { return u.has("center_calied") || u.has("floor_calied"); }

std::pair<Poly, double> boundsOf(const Json& u) {
    Poly pts;
    double h = 0;
    for (const Json& q : u.get("collision_bounds").items()) {
        if (q.size() < 4) {
            continue;
        }
        const Json& v0 = q.at(0);
        if (v0.size() < 3) {
            continue;
        }
        pts.push_back({v0.at(0).num(), v0.at(2).num()});
        for (const Json& v : q.items()) {
            if (v.size() >= 3) {
                h = std::max(h, v.at(1).num());
            }
        }
    }
    if (h <= 0) {
        h = kDefaultHeight;
    }
    return {pts, h};
}

UniverseData universeData(const Json& u) {
    UniverseData d;
    std::tie(d.bounds, d.height) = boundsOf(u);
    d.standing = standingOf(u, "standing");
    const Json& pa = u.get("play_area");
    if (pa.size() >= 2) {
        d.playArea = {pa.at(0).num(), pa.at(1).num()};
    }
    d.pimax = isPimax(u);
    d.time = u.get("time").str();
    return d;
}

const Json* findUniverse(const Json& doc, const std::string& id) {
    for (const Json* u : universesOf(doc)) {
        if (universeId(*u) == id) {
            return u;
        }
    }
    return nullptr;
}

Json* findUniverse(Json& doc, const std::string& id) {
    for (Json* u : universesOf(doc)) {
        if (universeId(*u) == id) {
            return u;
        }
    }
    return nullptr;
}

Json makeBounds(const Poly& p, double h) {
    Json out = Json::array();
    const auto v = [](double x, double y, double z) {
        return Json::array({Json::number(r6(x)), Json::number(y), Json::number(r6(z))});
    };
    for (std::size_t i = 0; i < p.size(); i++) {
        const Pt2 a = p[i];
        const Pt2 b = p[(i + 1) % p.size()];
        out.push(Json::array({v(a.x, 0.0, a.z), v(a.x, h, a.z), v(b.x, h, b.z), v(b.x, 0.0, b.z)}));
    }
    return out;
}

std::expected<Json, std::string> readJsonFile(const std::string& path) {
    auto t = readFile(path);
    if (!t) {
        return std::unexpected(t.error());
    }
    auto j = parseJson(*t);
    if (!j) {
        return std::unexpected(std::format("{}: {}", fs::path(path).filename().string(), j.error()));
    }
    return j;
}

std::expected<Store, std::string> Store::load(const std::vector<std::string>& dirs, const std::string& prefer) {
    Store s;
    std::set<std::string> seen;
    for (const auto& d : dirs) {
        std::vector<std::string> m;
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(d, ec)) {
            const std::string n = e.path().filename().string();
            if (n.starts_with("chaperone_info") && n.ends_with(".vrchap")) {
                m.push_back(e.path().string());
            }
        }
        std::ranges::sort(m);
        for (const auto& f : m) {
            if (seen.insert(lower(f)).second) {
                s.files_.push_back(f);
            }
        }
    }
    if (s.files_.empty()) {
        return std::unexpected(std::string(
            "no chaperone_info.vrchap in the SteamVR config folders: run SteamVR's Room Setup once first"));
    }
    std::map<std::string, std::time_t> newestTime; // by the entry's "time" string when it parses, else the file
    for (const auto& f : s.files_) {
        auto doc = readJsonFile(f);
        if (!doc) {
            continue;
        }
        const std::time_t mt = mtimeOf(f);
        for (const Json* u : universesOf(*doc)) {
            const std::string id = universeId(*u);
            if (id.empty()) {
                continue;
            }
            auto [it, fresh] = s.unis_.try_emplace(id);
            UniverseInfo& inf = it->second;
            if (fresh) {
                inf.id = id;
                s.ids_.push_back(id);
            }
            const auto [p, h] = boundsOf(*u);
            inf.pimax = inf.pimax || isPimax(*u);
            inf.files.push_back(f);
            const bool pimaxFile = lower(f).find("pimax") != std::string::npos;
            const bool srcPimax = lower(inf.source).find("pimax") != std::string::npos;
            if (inf.source.empty() || (pimaxFile && !srcPimax) || (pimaxFile == srcPimax && mt > inf.newest)) {
                inf.newest = mt;
                inf.source = f;
                inf.points = static_cast<int>(p.size());
                inf.time = u->get("time").str();
            }
        }
    }
    if (s.ids_.empty()) {
        return std::unexpected(std::string("the chaperone files have no universes: run SteamVR's Room Setup once first"));
    }
    std::ranges::sort(s.ids_);
    std::string pick;
    if (!prefer.empty() && s.unis_.contains(prefer)) {
        pick = prefer;
    }
    if (pick.empty()) {
        for (const auto& id : s.ids_) {
            if (s.unis_[id].pimax && (pick.empty() || s.unis_[id].points > s.unis_[pick].points)) {
                pick = id;
            }
        }
    }
    if (pick.empty()) {
        // SteamVR writes "time" as text; the file order puts the last written universe last, so prefer the entry
        // whose source file is newest and, among equals, the one with most points.
        for (const auto& id : s.ids_) {
            const auto& u = s.unis_[id];
            if (pick.empty() || u.newest > s.unis_[pick].newest ||
                (u.newest == s.unis_[pick].newest && u.points > s.unis_[pick].points)) {
                pick = id;
            }
        }
    }
    if (auto r = s.selectUniverse(pick); !r) {
        return std::unexpected(r.error());
    }
    return s;
}

std::expected<void, std::string> Store::selectUniverse(const std::string& id) {
    const auto it = unis_.find(id);
    if (it == unis_.end()) {
        return std::unexpected(std::format("universe {} not found", id));
    }
    const UniverseInfo& inf = it->second;
    auto doc = readJsonFile(inf.source);
    if (!doc) {
        return std::unexpected(doc.error());
    }
    const Json* u = findUniverse(*doc, id);
    if (u == nullptr) {
        return std::unexpected(std::format("universe {} missing from {}", id, inf.source));
    }
    const UniverseData d = universeData(*u);
    uniId_ = id;
    uniFiles_ = inf.files;
    source_ = inf.source;
    rawPts_ = d.bounds;
    height_ = d.height;
    standY_ = d.standing.y;
    tx_ = d.standing.x;
    tz_ = d.standing.z;
    yaw_ = d.standing.yaw;
    playArea_ = d.playArea;
    mode_ = defaultFrameMode(inf.pimax);
    pts_ = toStageAll(rawPts_);
    return {};
}

std::string Store::nextUniverse() const {
    for (std::size_t i = 0; i < ids_.size(); i++) {
        if (ids_[i] == uniId_) {
            return ids_[(i + 1) % ids_.size()];
        }
    }
    return uniId_;
}

Pt2 Store::toStageMode(FrameMode m, Pt2 p) const {
    if (m == FrameMode::Off) {
        return p;
    }
    const double y = m == FrameMode::A ? yaw_ : -yaw_;
    const double c = std::cos(y);
    const double sn = std::sin(y);
    const double dx = p.x - tx_;
    const double dz = p.z - tz_;
    return {c * dx - sn * dz, sn * dx + c * dz};
}

Pt2 Store::toFileMode(FrameMode m, Pt2 q) const {
    if (m == FrameMode::Off) {
        return q;
    }
    const double y = m == FrameMode::A ? yaw_ : -yaw_;
    const double c = std::cos(y);
    const double sn = std::sin(y);
    return {c * q.x + sn * q.z + tx_, -sn * q.x + c * q.z + tz_};
}

Poly Store::toStageAll(const Poly& p) const {
    Poly out;
    out.reserve(p.size());
    for (const Pt2 q : p) {
        out.push_back(adj_.apply(toStageMode(mode_, q)));
    }
    return out;
}

Poly Store::toFileAll(const Poly& p) const {
    const Rigid2 inv = adj_.inv();
    Poly out;
    out.reserve(p.size());
    for (const Pt2 q : p) {
        out.push_back(toFileMode(mode_, inv.apply(q)));
    }
    return out;
}

void Store::setAdj(const Rigid2& r) {
    adj_ = r;
    pts_ = toStageAll(rawPts_);
}

void Store::moveStage(const Rigid2& r) {
    adj_ = adj_.then(r);
    pts_ = r.all(pts_);
}

Poly Store::setMode(FrameMode m, const Poly& p) {
    Poly out;
    out.reserve(p.size());
    for (const Pt2 q : p) {
        out.push_back(toStageMode(m, toFileMode(mode_, q)));
    }
    mode_ = m;
    pts_ = toStageAll(rawPts_);
    return out;
}

Standing Store::standingForCenter(const Rigid2& center) const {
    const Rigid2 m = center.then(adj_.inv());
    return standingFromRigid(m.then(standingRigid()), standY_);
}

Rigid2 Store::centerForStanding(const Standing& s) const {
    const Rigid2 sNew = rigidFromYaw(-s.yaw, s.x, s.z);
    const Rigid2 m = sNew.then(standingRigid().inv());
    return m.then(adj_);
}

std::expected<SaveResult, std::string> Store::writeAll(const std::string& id, const std::vector<std::string>& files,
                                                       const Poly& filePts, double h, const SaveExtras& extras,
                                                       std::time_t now) {
    const std::string stamp = backupStamp(now);
    SaveResult res;
    for (const auto& path : files) {
        auto doc = readJsonFile(path);
        if (!doc) {
            res.skipped.push_back(path);
            continue;
        }
        Json* u = findUniverse(*doc, id);
        if (u == nullptr) {
            res.skipped.push_back(path);
            continue;
        }
        (*u)["collision_bounds"] = makeBounds(filePts, h);
        if (extras.standing && u->has("standing")) {
            Json& st = (*u)["standing"];
            if (st.isObject()) {
                if (st.has("translation") && st["translation"].size() >= 3) {
                    st["translation"].at(0) = Json::number(r6(extras.standing->x));
                    st["translation"].at(2) = Json::number(r6(extras.standing->z));
                }
                st["yaw"] = Json::number(r6(extras.standing->yaw));
            }
        }
        if (extras.playArea) {
            (*u)["play_area"] = Json::array({Json::number(r6(extras.playArea->w)), Json::number(r6(extras.playArea->h))});
        }
        if (extras.floorDelta != 0 && u->has("standing")) {
            Json& st = (*u)["standing"];
            if (st.has("translation") && st["translation"].size() >= 2) {
                Json& ty = st["translation"].at(1);
                ty = Json::number(r6(ty.num() + extras.floorDelta));
            }
        }
        if (!backedUp_.contains(path)) {
            if (auto b = readFile(path)) {
                if (auto w = writeFileAtomic(path + ".bak-" + stamp, *b); !w) {
                    return std::unexpected(std::format("couldn't back up {}: {}", path, w.error()));
                }
            }
            backedUp_.insert(path);
        }
        if (auto w = writeFileAtomic(path, doc->dump() + "\n"); !w) {
            return std::unexpected(w.error());
        }
        res.written.push_back(path);
    }
    if (res.written.empty()) {
        return std::unexpected(std::string("no chaperone file contained this universe"));
    }
    return res;
}

std::expected<SaveResult, std::string> Store::save(const Poly& stagePts, double h, const SaveExtras& extras,
                                                   std::time_t now) {
    Poly filePts = toFileAll(stagePts);
    if (mode_ == FrameMode::Off && extras.standing) {
        // the file holds standing coordinates: express the walls in the new standing frame
        const Rigid2 sNew = rigidFromYaw(-extras.standing->yaw, extras.standing->x, extras.standing->z);
        const Rigid2 m = sNew.then(standingRigid().inv()); // new standing -> old standing
        filePts = m.inv().all(filePts);
    }
    auto r = writeAll(uniId_, uniFiles_, filePts, h, extras, now);
    if (!r) {
        return r;
    }
    standY_ += extras.floorDelta;
    rawPts_ = filePts;
    height_ = h;
    if (extras.standing) {
        tx_ = extras.standing->x;
        tz_ = extras.standing->z;
        yaw_ = extras.standing->yaw;
    }
    if (extras.playArea) {
        playArea_ = *extras.playArea;
    }
    unis_[uniId_].points = static_cast<int>(filePts.size());
    return r;
}

std::expected<SaveResult, std::string> Store::writeUniverse(const std::string& id, const UniverseData& data,
                                                            bool withStanding, bool withPlayArea, std::time_t now) {
    const auto it = unis_.find(id);
    if (it == unis_.end()) {
        return std::unexpected(std::format("universe {} not found", id));
    }
    if (data.bounds.size() < 3) {
        return std::unexpected(std::string("a boundary needs at least 3 points"));
    }
    SaveExtras ex;
    if (withStanding) {
        ex.standing = data.standing;
    }
    if (withPlayArea) {
        ex.playArea = data.playArea;
    }
    // the floor height is part of the standing pose
    double curY = 0;
    if (auto doc = readJsonFile(it->second.source)) {
        if (const Json* u = findUniverse(*doc, id)) {
            curY = standingOf(*u, "standing").y;
        }
    }
    if (withStanding) {
        ex.floorDelta = data.standing.y - curY;
    }
    auto r = writeAll(id, it->second.files, data.bounds, data.height, ex, now);
    if (r && id == uniId_) {
        (void)selectUniverse(id); // the edited universe follows the file
    }
    if (r) {
        it->second.points = static_cast<int>(data.bounds.size());
    }
    return r;
}

std::expected<Store::Backup, std::string> Store::oldestBackup() const {
    std::vector<std::string> baks;
    for (const auto& f : uniFiles_) {
        const auto b = backupsOf(f);
        baks.insert(baks.end(), b.begin(), b.end());
    }
    if (baks.empty()) {
        return std::unexpected(std::string("no backups found"));
    }
    std::ranges::sort(baks, [](const std::string& a, const std::string& b) {
        return a.substr(a.rfind(".bak-")) < b.substr(b.rfind(".bak-"));
    });
    for (const auto& b : baks) {
        auto doc = readJsonFile(b);
        if (!doc) {
            continue;
        }
        if (const Json* u = findUniverse(*doc, uniId_)) {
            UniverseData d = universeData(*u);
            if (d.bounds.size() >= 3) {
                Poly pts = d.bounds;
                if (mode_ == FrameMode::Off) {
                    // standing coordinates of the backup's standing pose -> those of the current one
                    const Rigid2 sb = rigidFromYaw(-d.standing.yaw, d.standing.x, d.standing.z);
                    pts = sb.then(standingRigid().inv()).all(pts);
                }
                return Backup{toStageAll(pts), b, d};
            }
        }
    }
    return std::unexpected(std::string("no backup has this universe"));
}

} // namespace pibridge::room
