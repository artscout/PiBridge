// SPDX-License-Identifier: MIT
#include "conf.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <charconv>
#include <cmath>
#include <vector>

#include "paths.h"

namespace pibridge::app {

namespace {

std::string_view trim(std::string_view s) {
    const auto b = s.find_first_not_of(" \t\r");
    if (b == std::string_view::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r");
    return s.substr(b, e - b + 1);
}

std::vector<std::string_view> splitLines(std::string_view text) {
    std::vector<std::string_view> out;
    size_t pos = 0;
    while (pos < text.size()) {
        const auto nl = text.find('\n', pos);
        if (nl == std::string_view::npos) {
            out.push_back(text.substr(pos));
            break;
        }
        out.push_back(text.substr(pos, nl - pos));
        pos = nl + 1;
    }
    return out;
}

// The key of a live line, or empty for blank lines, comments and lines without '='.
std::string_view keyOf(std::string_view line) {
    const std::string_view t = trim(line);
    if (t.empty() || t.front() == '#') {
        return {};
    }
    const auto eq = t.find('=');
    if (eq == std::string_view::npos) {
        return {};
    }
    return trim(t.substr(0, eq));
}

// The value part of a live line without a trailing comment.
std::string_view valueOf(std::string_view line) {
    const std::string_view t = trim(line);
    std::string_view v = t.substr(t.find('=') + 1);
    if (const auto hash = v.find('#'); hash != std::string_view::npos) {
        v = v.substr(0, hash);
    }
    return trim(v);
}

const char* const kHeader = "# pibridge settings (docs/CONVENTIONS.md). The app changes the keys you switch "
                            "and keeps every other line.\n";

} // namespace

std::optional<std::string> confGet(std::string_view text, std::string_view key) {
    std::optional<std::string> out;
    for (const auto line : splitLines(text)) {
        if (keyOf(line) == key) {
            out = std::string(valueOf(line));
        }
    }
    return out;
}

std::string confSet(std::string_view text, std::string_view key, std::string_view value) {
    std::string out;
    bool found = false;
    for (const auto line : splitLines(text)) {
        if (!key.empty() && keyOf(line) == key) {
            found = true;
            out += key;
            out += '=';
            out += value;
            // keep a trailing comment ("refresh_hz=90  # my note")
            const std::string_view t = trim(line);
            if (const auto hash = t.find('#', t.find('=')); hash != std::string_view::npos) {
                out += "  ";
                out += t.substr(hash);
            }
        } else {
            out += line;
        }
        out += '\n';
    }
    if (!found) {
        out += key;
        out += '=';
        out += value;
        out += '\n';
    }
    return out;
}

std::string confNumber(double v) {
    char buf[64];
    if (std::isfinite(v) && v == std::round(v) && std::fabs(v) < 1e15) {
        const auto r = std::to_chars(buf, buf + sizeof buf, static_cast<long long>(v));
        return std::string(buf, r.ptr);
    }
    const auto r = std::to_chars(buf, buf + sizeof buf, v);
    return std::string(buf, r.ptr);
}

bool confBool(const std::optional<std::string>& v, bool def) {
    if (!v) {
        return def;
    }
    if (*v == "on" || *v == "1" || *v == "true" || *v == "yes") {
        return true;
    }
    if (*v == "off" || *v == "0" || *v == "false" || *v == "no") {
        return false;
    }
    return def;
}

Conf::Conf(QString path, QString fallback) : path_(std::move(path)), fallback_(std::move(fallback)) {
    reload();
}

void Conf::reload() {
    text_.clear();
    // The live settings win; then the legacy path (PiBridge's earlier name); then the shipped read-only default seed
    // (defaultsConfPath(), <prefix>/etc/pibridge/pibridge.conf). set() always writes path_, so none of the fallbacks
    // is ever overwritten.
    QString src = path_;
    if (!QFileInfo::exists(src)) {
        if (!fallback_.isEmpty() && QFileInfo::exists(fallback_)) {
            src = fallback_;
        } else if (const QString def = defaultsConfPath(); QFileInfo::exists(def)) {
            src = def;
        }
    }
    QFile f(src);
    if (f.open(QIODevice::ReadOnly)) {
        text_ = f.readAll().toStdString();
    }
}

QString Conf::value(std::string_view key, const QString& def) const {
    const auto v = confGet(text_, key);
    return v ? QString::fromStdString(*v) : def;
}

bool Conf::flag(std::string_view key, bool def) const {
    return confBool(confGet(text_, key), def);
}

int Conf::refreshHz() const {
    const auto v = confGet(text_, "refresh_hz");
    return v && *v == "72" ? 72 : 90;
}

bool Conf::set(std::string_view key, std::string_view value) {
    reload(); // another writer (an editor, a second app window) may have changed it
    const std::string base = text_.empty() ? std::string(kHeader) : text_;
    const std::string next = confSet(base, key, value);
    QDir().mkpath(QFileInfo(path_).absolutePath());
    QSaveFile f(path_); // temp file + rename: a daemon never reads half a file
    if (!f.open(QIODevice::WriteOnly)) {
        return false;
    }
    f.write(QByteArray::fromStdString(next));
    if (!f.commit()) {
        return false;
    }
    text_ = next;
    return true;
}

} // namespace pibridge::app
