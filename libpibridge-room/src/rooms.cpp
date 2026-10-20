// SPDX-License-Identifier: MIT
// Ported from ZoneTweak profiles.go (MIT, Copyright (c) 2026 killdano; see NOTICE).
#include "pibridge/room/rooms.h"

#include "pibridge/room/files.h"
#include "pibridge/room/json.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <format>

namespace fs = std::filesystem;

namespace pibridge::room {

namespace {

std::string trim(const std::string& s) {
    std::size_t a = 0;
    std::size_t b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a])) != 0) {
        a++;
    }
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1])) != 0) {
        b--;
    }
    return s.substr(a, b - a);
}

std::string lower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

} // namespace

UniverseData Room::toUniverse() const {
    UniverseData d;
    d.bounds = points;
    d.height = height;
    if (standing) {
        d.standing = *standing;
    }
    if (playArea) {
        d.playArea = *playArea;
    }
    return d;
}

Room Room::fromUniverse(const std::string& name, const std::string& universe, const UniverseData& d) {
    Room r;
    r.name = name;
    r.universe = universe;
    r.height = d.height;
    r.points = d.bounds;
    r.standing = d.standing;
    r.playArea = d.playArea;
    return r;
}

std::string rfc3339(std::time_t t) {
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[40];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%S%z", &tm);
    std::string s = buf;
    if (s.size() >= 5) {
        s.insert(s.size() - 2, ":"); // +0200 -> +02:00
    }
    return s;
}

std::string roomToJson(const Room& r) {
    Json j = Json::object();
    j["name"] = Json::string(r.name);
    j["universe"] = Json::string(r.universe);
    j["height"] = Json::number(r.height);
    Json pts = Json::array();
    for (const Pt2 p : r.points) {
        pts.push(Json::array({Json::number(p.x), Json::number(p.z)}));
    }
    j["points"] = std::move(pts);
    if (!r.frame.empty()) {
        j["frame"] = Json::string(r.frame);
    }
    j["saved"] = Json::string(r.saved);
    if (!r.created.empty()) {
        j["created"] = Json::string(r.created);
    }
    if (r.standing) {
        Json st = Json::object();
        st["translation"] =
            Json::array({Json::number(r.standing->x), Json::number(r.standing->y), Json::number(r.standing->z)});
        st["yaw"] = Json::number(r.standing->yaw);
        j["standing"] = std::move(st);
    }
    if (r.playArea) {
        j["play_area"] = Json::array({Json::number(r.playArea->w), Json::number(r.playArea->h)});
    }
    if (r.warningDistance) {
        j["warning_distance"] = Json::number(*r.warningDistance);
    }
    return j.dump(2) + "\n";
}

std::expected<Room, std::string> roomFromJson(std::string_view text) {
    auto j = parseJson(text);
    if (!j) {
        return std::unexpected(j.error());
    }
    if (!j->isObject()) {
        return std::unexpected(std::string("a room is a JSON object"));
    }
    Room r;
    r.name = j->get("name").str();
    r.universe = j->get("universe").str();
    r.height = j->get("height").num(kDefaultHeight);
    if (r.height <= 0) {
        r.height = kDefaultHeight;
    }
    for (const Json& p : j->get("points").items()) {
        if (p.size() >= 2) {
            r.points.push_back({p.at(0).num(), p.at(1).num()});
        }
    }
    r.frame = j->get("frame").str();
    r.saved = j->get("saved").str();
    r.created = j->get("created").str();
    if (const Json& st = j->get("standing"); st.isObject()) {
        Standing s;
        const Json& tr = st.get("translation");
        if (tr.size() >= 3) {
            s.x = tr.at(0).num();
            s.y = tr.at(1).num();
            s.z = tr.at(2).num();
        }
        s.yaw = st.get("yaw").num();
        r.standing = s;
    }
    if (const Json& pa = j->get("play_area"); pa.size() >= 2) {
        r.playArea = Size2{pa.at(0).num(), pa.at(1).num()};
    }
    if (const Json& wd = j->get("warning_distance"); wd.isNumber()) {
        r.warningDistance = wd.num();
    }
    if (r.name.empty() || r.points.size() < 3) {
        return std::unexpected(std::string("a room needs a name and at least 3 points"));
    }
    return r;
}

std::string RoomLib::fileFor(const std::string& name) const {
    std::string b;
    for (const char ch : lower(trim(name))) {
        if ((ch >= 'a' && ch <= 'z') || (ch >= '0' && ch <= '9')) {
            b += ch;
        } else if (ch == ' ' || ch == '-' || ch == '_') {
            b += '-';
        }
    }
    while (!b.empty() && b.front() == '-') {
        b.erase(b.begin());
    }
    while (!b.empty() && b.back() == '-') {
        b.pop_back();
    }
    if (b.empty()) {
        b = "space";
    }
    return dir_ + "/" + b + ".json";
}

std::vector<Room> RoomLib::list() const {
    std::vector<Room> out;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(dir_, ec)) {
        if (e.path().extension() != ".json") {
            continue;
        }
        auto t = readFile(e.path().string());
        if (!t) {
            continue;
        }
        if (auto r = roomFromJson(*t)) {
            out.push_back(std::move(*r));
        }
    }
    std::ranges::sort(out, [](const Room& a, const Room& b) { return lower(a.name) < lower(b.name); });
    return out;
}

std::optional<Room> RoomLib::find(const std::string& name) const {
    const std::string want = lower(trim(name));
    for (auto& r : list()) {
        if (lower(trim(r.name)) == want) {
            return r;
        }
    }
    return std::nullopt;
}

std::expected<void, std::string> RoomLib::save(Room r, std::time_t now) {
    r.name = trim(r.name);
    if (r.name.empty()) {
        return std::unexpected(std::string("name the room first"));
    }
    if (r.points.size() < 3) {
        return std::unexpected(std::string("a room needs at least 3 points"));
    }
    std::error_code ec;
    fs::create_directories(dir_, ec);
    if (ec) {
        return std::unexpected(std::format("cannot create {}", dir_));
    }
    const std::string path = fileFor(r.name);
    if (auto old = readFile(path)) {
        if (auto o = roomFromJson(*old); o && !o->created.empty()) {
            r.created = o->created;
        }
    }
    r.saved = rfc3339(now);
    if (r.created.empty()) {
        r.created = r.saved;
    }
    return writeFileAtomic(path, roomToJson(r));
}

std::expected<void, std::string> RoomLib::remove(const std::string& name) {
    std::error_code ec;
    if (!fs::remove(fileFor(name), ec) || ec) {
        return std::unexpected(std::format("no room called {}", name));
    }
    if (lower(active()) == lower(trim(name))) {
        setActive("");
    }
    return {};
}

std::expected<void, std::string> RoomLib::rename(const std::string& from, const std::string& to, std::time_t now) {
    auto r = find(from);
    if (!r) {
        return std::unexpected(std::format("no room called {}", from));
    }
    if (trim(to).empty()) {
        return std::unexpected(std::string("name the room first"));
    }
    const bool sameFile = fileFor(from) == fileFor(to);
    if (!sameFile && find(to)) {
        return std::unexpected(std::format("there is already a room called {}", trim(to)));
    }
    const bool wasActive = lower(active()) == lower(trim(from));
    r->name = trim(to);
    if (auto s = save(*r, now); !s) {
        return s;
    }
    if (!sameFile) {
        std::error_code ec;
        fs::remove(fileFor(from), ec);
    }
    if (wasActive) {
        setActive(r->name);
    }
    return {};
}

std::string RoomLib::active() const {
    auto t = readFile(dir_ + "/active.txt");
    return t ? trim(*t) : std::string();
}

void RoomLib::setActive(const std::string& name) const {
    std::error_code ec;
    fs::create_directories(dir_, ec);
    (void)writeFileAtomic(dir_ + "/active.txt", name);
}

FrameMode RoomLib::frameMode(FrameMode fallback) const {
    auto t = readFile(dir_ + "/align.txt");
    if (!t) {
        return fallback;
    }
    const std::string v = trim(*t);
    if (v == "0") {
        return FrameMode::A;
    }
    if (v == "1") {
        return FrameMode::B;
    }
    if (v == "2") {
        return FrameMode::Off;
    }
    return fallback;
}

void RoomLib::setFrameMode(FrameMode m) const {
    std::error_code ec;
    fs::create_directories(dir_, ec);
    (void)writeFileAtomic(dir_ + "/align.txt", std::to_string(static_cast<int>(m)));
}

} // namespace pibridge::room
