// SPDX-License-Identifier: MIT
#include "apps_own.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QLocale>
#include <QRegularExpression>
#include <QSaveFile>
#include <QSet>
#include <algorithm>
#include <variant>

namespace pibridge::app {

QString appSlug(const QString& name) {
    QString s;
    bool dash = false;
    for (const QChar c : name.toLower()) {
        if ((c >= QLatin1Char('a') && c <= QLatin1Char('z')) || (c >= QLatin1Char('0') && c <= QLatin1Char('9'))) {
            if (dash && !s.isEmpty()) {
                s += QLatin1Char('-');
            }
            dash = false;
            s += c;
        } else {
            dash = true;
        }
    }
    return s.isEmpty() ? QStringLiteral("app") : s.left(48);
}

QString qvProfileName(const OwnApp& a) {
    if (!a.qvProfile.trimmed().isEmpty()) {
        return a.qvProfile.trimmed();
    }
    return QFileInfo(a.exec).completeBaseName();
}

QStringList validateOwnApp(const OwnApp& a) {
    QStringList p;
    if (a.name.trimmed().isEmpty()) {
        p << QStringLiteral("Give the app a name.");
    }
    if (a.exec.trimmed().isEmpty()) {
        p << QStringLiteral("Choose the program to start.");
    }
    static const QRegularExpression envLine(QStringLiteral("^[A-Za-z_][A-Za-z0-9_]*=.*$"));
    for (qsizetype i = 0; i < a.env.size(); i++) {
        if (!envLine.match(a.env[i]).hasMatch()) {
            p << QStringLiteral("Environment line %1 is not KEY=value.").arg(i + 1);
        }
    }
    return p;
}

QString joinArgs(const QStringList& args) {
    QStringList out;
    for (const QString& a : args) {
        const bool plain = !a.isEmpty() && std::none_of(a.begin(), a.end(), [](QChar c) {
            return c.isSpace() || c == QLatin1Char('"') || c == QLatin1Char('\\') || c == QLatin1Char('\'');
        });
        if (plain) {
            out << a;
            continue;
        }
        QString q = QStringLiteral("\"");
        for (const QChar c : a) {
            if (c == QLatin1Char('"') || c == QLatin1Char('\\')) {
                q += QLatin1Char('\\');
            }
            q += c;
        }
        out << q + QLatin1Char('"');
    }
    return out.join(QLatin1Char(' '));
}

QStringList splitArgs(const QString& line) {
    // shell-like: white space separates, "..." and '...' group (an empty pair is an empty argument), a backslash
    // takes the next character as it is (outside single quotes)
    QStringList out;
    QString cur;
    bool have = false;
    QChar quote;
    for (qsizetype i = 0; i < line.size(); i++) {
        const QChar c = line[i];
        if (c == QLatin1Char('\\') && quote != QLatin1Char('\'') && i + 1 < line.size()) {
            cur += line[++i];
            have = true;
        } else if (!quote.isNull()) {
            if (c == quote) {
                quote = QChar();
            } else {
                cur += c;
            }
        } else if (c == QLatin1Char('"') || c == QLatin1Char('\'')) {
            quote = c;
            have = true;
        } else if (c.isSpace()) {
            if (have) {
                out << cur;
                cur.clear();
                have = false;
            }
        } else {
            cur += c;
            have = true;
        }
    }
    if (have) {
        out << cur;
    }
    return out;
}

QString lastPlayedWords(qint64 secs, const QDateTime& now) {
    if (secs <= 0) {
        return QStringLiteral("Never played");
    }
    const QDateTime t = QDateTime::fromSecsSinceEpoch(secs, now.timeZone());
    const qint64 days = t.date().daysTo(now.date());
    const QLocale en(QLocale::English);
    if (days <= 0) {
        return QStringLiteral("Played today, %1").arg(en.toString(t.time(), QStringLiteral("HH:mm")));
    }
    if (days == 1) {
        return QStringLiteral("Played yesterday");
    }
    if (days < 7) {
        return QStringLiteral("Played %1 days ago").arg(days);
    }
    return QStringLiteral("Played %1").arg(en.toString(t.date(), QStringLiteral("d MMM yyyy")));
}

QString tomlQuote(const QString& s) {
    QString o = QStringLiteral("\"");
    for (const QChar c : s) {
        switch (c.unicode()) {
        case '"': o += QStringLiteral("\\\""); break;
        case '\\': o += QStringLiteral("\\\\"); break;
        case '\n': o += QStringLiteral("\\n"); break;
        case '\t': o += QStringLiteral("\\t"); break;
        case '\r': o += QStringLiteral("\\r"); break;
        default:
            if (c.unicode() < 0x20 || c.unicode() == 0x7f) {
                o += QStringLiteral("\\u%1").arg(static_cast<uint>(c.unicode()), 4, 16, QLatin1Char('0'));
            } else {
                o += c;
            }
        }
    }
    return o + QLatin1Char('"');
}

QString tomlStringArray(const QStringList& l) {
    QStringList q;
    for (const QString& s : l) {
        q << tomlQuote(s);
    }
    return QLatin1Char('[') + q.join(QStringLiteral(", ")) + QLatin1Char(']');
}

namespace {

using Value = std::variant<QString, bool, qint64, QStringList>;

// A tiny cursor over one value's text.
struct Cur {
    QStringView t;
    qsizetype i = 0;
    bool end() const { return i >= t.size(); }
    QChar peek() const { return end() ? QChar() : t[i]; }
    // Skips spaces, newlines and (when `comments`) # comments: inside arrays both may appear.
    void skip(bool comments) {
        while (!end()) {
            const QChar c = t[i];
            if (c == QLatin1Char(' ') || c == QLatin1Char('\t') || c == QLatin1Char('\n') || c == QLatin1Char('\r')) {
                i++;
            } else if (comments && c == QLatin1Char('#')) {
                while (!end() && t[i] != QLatin1Char('\n')) {
                    i++;
                }
            } else {
                return;
            }
        }
    }
};

bool hex(QStringView s, char32_t& out) {
    bool ok = false;
    out = static_cast<char32_t>(s.toUInt(&ok, 16));
    return ok;
}

bool readString(Cur& c, QString& out) {
    const QChar q = c.peek();
    if (q != QLatin1Char('"') && q != QLatin1Char('\'')) {
        return false;
    }
    if (c.t.mid(c.i, 3) == QStringView(u"\"\"\"") || c.t.mid(c.i, 3) == QStringView(u"'''")) {
        return false; // multi-line strings: kept as an unknown line
    }
    c.i++;
    out.clear();
    while (!c.end()) {
        const QChar ch = c.t[c.i++];
        if (ch == q) {
            return true;
        }
        if (ch == QLatin1Char('\n')) {
            return false;
        }
        if (q == QLatin1Char('"') && ch == QLatin1Char('\\')) {
            if (c.end()) {
                return false;
            }
            const QChar e = c.t[c.i++];
            switch (e.unicode()) {
            case '"': out += QLatin1Char('"'); break;
            case '\\': out += QLatin1Char('\\'); break;
            case 'n': out += QLatin1Char('\n'); break;
            case 't': out += QLatin1Char('\t'); break;
            case 'r': out += QLatin1Char('\r'); break;
            case 'b': out += QChar(0x08); break;
            case 'f': out += QChar(0x0c); break;
            case 'e': out += QChar(0x1b); break;
            case 'u':
            case 'U': {
                const qsizetype n = e == QLatin1Char('u') ? 4 : 8;
                char32_t cp = 0;
                if (c.i + n > c.t.size() || !hex(c.t.mid(c.i, n), cp)) {
                    return false;
                }
                c.i += n;
                out += QString::fromUcs4(&cp, 1);
                break;
            }
            default: return false;
            }
            continue;
        }
        out += ch;
    }
    return false;
}

bool readValue(Cur& c, Value& out) {
    const QChar ch = c.peek();
    if (ch == QLatin1Char('"') || ch == QLatin1Char('\'')) {
        QString s;
        if (!readString(c, s)) {
            return false;
        }
        out = s;
        return true;
    }
    if (ch == QLatin1Char('[')) {
        c.i++;
        QStringList l;
        for (;;) {
            c.skip(true);
            if (c.peek() == QLatin1Char(']')) {
                c.i++;
                out = l;
                return true;
            }
            QString s;
            if (!readString(c, s)) {
                return false; // arrays of other things: unknown
            }
            l << s;
            c.skip(true);
            if (c.peek() == QLatin1Char(',')) {
                c.i++;
            } else if (c.peek() != QLatin1Char(']')) {
                return false;
            }
        }
    }
    // a bare word: true / false / an integer
    qsizetype j = c.i;
    while (j < c.t.size() && !c.t[j].isSpace() && c.t[j] != QLatin1Char('#')) {
        j++;
    }
    const QString w = c.t.mid(c.i, j - c.i).toString();
    c.i = j;
    if (w == QLatin1String("true") || w == QLatin1String("false")) {
        out = w == QLatin1String("true");
        return true;
    }
    static const QRegularExpression intRe(QStringLiteral("^[+-]?[0-9](?:_?[0-9])*$"));
    if (intRe.match(w).hasMatch()) {
        bool ok = false;
        const qint64 v = QString(w).remove(QLatin1Char('_')).toLongLong(&ok);
        if (ok) {
            out = v;
            return true;
        }
    }
    return false;
}

// After the value only spaces and a comment may follow.
bool onlyCommentLeft(Cur& c) {
    while (!c.end() && (c.peek() == QLatin1Char(' ') || c.peek() == QLatin1Char('\t') || c.peek() == QLatin1Char('\r'))) {
        c.i++;
    }
    return c.end() || c.peek() == QLatin1Char('#');
}

// Whether the brackets of an array value are closed (strings and comments are skipped).
bool arrayClosed(QStringView v) {
    int depth = 0;
    QChar quote;
    for (qsizetype i = 0; i < v.size(); i++) {
        const QChar c = v[i];
        if (!quote.isNull()) {
            if (quote == QLatin1Char('"') && c == QLatin1Char('\\')) {
                i++;
            } else if (c == quote) {
                quote = QChar();
            }
            continue;
        }
        if (c == QLatin1Char('"') || c == QLatin1Char('\'')) {
            quote = c;
        } else if (c == QLatin1Char('#')) {
            while (i < v.size() && v[i] != QLatin1Char('\n')) {
                i++;
            }
        } else if (c == QLatin1Char('[')) {
            depth++;
        } else if (c == QLatin1Char(']')) {
            depth--;
        }
    }
    return depth <= 0;
}

// "key = rest": the key (bare or quoted) and the text after '='; false when the line is no key/value pair.
bool splitKey(const QString& line, QString& key, QString& rest) {
    Cur c{line};
    c.skip(false);
    if (c.peek() == QLatin1Char('"') || c.peek() == QLatin1Char('\'')) {
        if (!readString(c, key)) {
            return false;
        }
    } else {
        const qsizetype start = c.i;
        while (!c.end() && (c.peek().isLetterOrNumber() || c.peek() == QLatin1Char('_') || c.peek() == QLatin1Char('-') ||
                            c.peek() == QLatin1Char('.'))) {
            c.i++;
        }
        key = line.mid(start, c.i - start);
        if (key.isEmpty()) {
            return false;
        }
    }
    c.skip(false);
    if (c.peek() != QLatin1Char('=')) {
        return false;
    }
    c.i++;
    c.skip(false);
    rest = line.mid(c.i);
    return true;
}

enum class Header { None, App, AppSub, Other };

Header headerKind(const QString& line) {
    const QString t = line.trimmed();
    if (!t.startsWith(QLatin1Char('['))) {
        return Header::None;
    }
    static const QRegularExpression app(QStringLiteral("^\\[\\[\\s*app\\s*\\]\\]\\s*(#.*)?$"));
    static const QRegularExpression sub(QStringLiteral("^\\[{1,2}\\s*app\\s*\\."));
    if (app.match(t).hasMatch()) {
        return Header::App;
    }
    if (sub.match(t).hasMatch()) {
        return Header::AppSub;
    }
    return Header::Other;
}

// Sets a known key; false when the key is unknown or the value has another type (then the line is kept as is).
bool setKnown(OwnApp& a, const QString& key, const Value& v) {
    const auto str = [&v](QString& into) {
        if (const auto* s = std::get_if<QString>(&v)) {
            into = *s;
            return true;
        }
        return false;
    };
    const auto list = [&v](QStringList& into) {
        if (const auto* l = std::get_if<QStringList>(&v)) {
            into = *l;
            return true;
        }
        return false;
    };
    if (key == QLatin1String("id")) return str(a.id);
    if (key == QLatin1String("name")) return str(a.name);
    if (key == QLatin1String("exec")) return str(a.exec);
    if (key == QLatin1String("workdir")) return str(a.workdir);
    if (key == QLatin1String("icon")) return str(a.icon);
    if (key == QLatin1String("qv_profile")) return str(a.qvProfile);
    if (key == QLatin1String("args")) return list(a.args);
    if (key == QLatin1String("env")) return list(a.env);
    if (key == QLatin1String("quad_views")) {
        if (const auto* b = std::get_if<bool>(&v)) {
            a.quadViews = *b;
            return true;
        }
        return false;
    }
    if (key == QLatin1String("last_played")) {
        if (const auto* n = std::get_if<qint64>(&v)) {
            a.lastPlayed = *n;
            return true;
        }
        return false;
    }
    return false;
}

QStringList appLines(const OwnApp& a) {
    QStringList o{QStringLiteral("[[app]]"), QStringLiteral("id = ") + tomlQuote(a.id),
                  QStringLiteral("name = ") + tomlQuote(a.name), QStringLiteral("exec = ") + tomlQuote(a.exec)};
    if (!a.args.isEmpty()) o << QStringLiteral("args = ") + tomlStringArray(a.args);
    if (!a.workdir.isEmpty()) o << QStringLiteral("workdir = ") + tomlQuote(a.workdir);
    if (!a.env.isEmpty()) o << QStringLiteral("env = ") + tomlStringArray(a.env);
    if (!a.icon.isEmpty()) o << QStringLiteral("icon = ") + tomlQuote(a.icon);
    o << QStringLiteral("quad_views = ") + (a.quadViews ? QStringLiteral("true") : QStringLiteral("false"));
    if (!a.qvProfile.isEmpty()) o << QStringLiteral("qv_profile = ") + tomlQuote(a.qvProfile);
    if (a.lastPlayed > 0) o << QStringLiteral("last_played = %1").arg(a.lastPlayed);
    o << a.extra;
    return o;
}

void trimBlankTail(QStringList& l) {
    while (!l.isEmpty() && l.last().trimmed().isEmpty()) {
        l.removeLast();
    }
}

} // namespace

AppsDoc AppsDoc::parse(QStringView text, QStringList* warnings) {
    AppsDoc d;
    QStringList lines = text.toString().split(QLatin1Char('\n'));
    if (!lines.isEmpty() && lines.last().isEmpty()) {
        lines.removeLast(); // the final newline
    }
    Block* cur = nullptr;
    bool inAppSub = false; // lines of an [app.x] sub-table: kept with the app, not read
    for (qsizetype i = 0; i < lines.size(); i++) {
        QString line = lines[i];
        if (line.endsWith(QLatin1Char('\r'))) {
            line.chop(1);
        }
        const Header h = headerKind(line);
        if (h == Header::App) {
            d.blocks_.push_back({true, {}, {}});
            cur = &d.blocks_.back();
            inAppSub = false;
            continue;
        }
        if (h == Header::AppSub && cur && cur->isApp) {
            cur->app.extra << line;
            inAppSub = true;
            continue;
        }
        if (h != Header::None) {
            d.blocks_.push_back({false, {}, {line}});
            cur = &d.blocks_.back();
            inAppSub = false;
            continue;
        }
        if (!cur) {
            d.preamble_ << line;
            continue;
        }
        if (!cur->isApp) {
            cur->raw << line;
            continue;
        }
        const QString t = line.trimmed();
        if (inAppSub || t.isEmpty() || t.startsWith(QLatin1Char('#'))) {
            cur->app.extra << line;
            continue;
        }
        QString key, rest;
        if (!splitKey(line, key, rest)) {
            cur->app.extra << line;
            if (warnings) {
                *warnings << QStringLiteral("apps.toml line %1 is not key = value; kept as it is.").arg(i + 1);
            }
            continue;
        }
        // an array may go on over the next lines
        QStringList raw{line};
        QString value = rest;
        if (rest.trimmed().startsWith(QLatin1Char('['))) {
            while (!arrayClosed(value) && i + 1 < lines.size()) {
                raw << lines[++i];
                value += QLatin1Char('\n') + raw.last();
            }
        }
        Cur c{value};
        Value v;
        if (readValue(c, v) && onlyCommentLeft(c) && setKnown(cur->app, key, v)) {
            continue;
        }
        cur->app.extra << raw;
    }
    for (Block& b : d.blocks_) {
        if (b.isApp) {
            trimBlankTail(b.app.extra);
        } else {
            trimBlankTail(b.raw);
        }
    }
    trimBlankTail(d.preamble_);
    // ids: missing ones from the name, duplicates made unique (in file order)
    QSet<QString> used;
    for (Block& b : d.blocks_) {
        if (!b.isApp) {
            continue;
        }
        QString id = b.app.id.isEmpty() ? appSlug(b.app.name) : b.app.id;
        if (used.contains(id)) {
            int n = 2;
            while (used.contains(QStringLiteral("%1-%2").arg(id).arg(n))) {
                n++;
            }
            id = QStringLiteral("%1-%2").arg(id).arg(n);
        }
        used.insert(id);
        b.app.id = id;
    }
    return d;
}

QString AppsDoc::text() const {
    QStringList out = preamble_;
    for (const Block& b : blocks_) {
        if (!out.isEmpty()) {
            out << QString();
        }
        out << (b.isApp ? appLines(b.app) : b.raw);
    }
    return out.isEmpty() ? QString() : out.join(QLatin1Char('\n')) + QLatin1Char('\n');
}

std::vector<OwnApp> AppsDoc::apps() const {
    std::vector<OwnApp> v;
    for (const Block& b : blocks_) {
        if (b.isApp) {
            v.push_back(b.app);
        }
    }
    return v;
}

std::optional<OwnApp> AppsDoc::find(const QString& id) const {
    for (const Block& b : blocks_) {
        if (b.isApp && b.app.id == id) {
            return b.app;
        }
    }
    return std::nullopt;
}

QString AppsDoc::uniqueId(const QString& wanted) const {
    const auto taken = [this](const QString& id) { return find(id).has_value(); };
    if (!taken(wanted)) {
        return wanted;
    }
    int n = 2;
    while (taken(QStringLiteral("%1-%2").arg(wanted).arg(n))) {
        n++;
    }
    return QStringLiteral("%1-%2").arg(wanted).arg(n);
}

QString AppsDoc::upsert(OwnApp a) {
    if (!a.id.isEmpty()) {
        for (Block& b : blocks_) {
            if (b.isApp && b.app.id == a.id) {
                b.app = std::move(a);
                return b.app.id;
            }
        }
    }
    a.id = uniqueId(a.id.isEmpty() ? appSlug(a.name) : a.id);
    blocks_.push_back({true, std::move(a), {}});
    return blocks_.back().app.id;
}

bool AppsDoc::remove(const QString& id) {
    for (auto it = blocks_.begin(); it != blocks_.end(); ++it) {
        if (it->isApp && it->app.id == id) {
            blocks_.erase(it);
            return true;
        }
    }
    return false;
}

QString appsConfigPath() {
    QString base = qEnvironmentVariable("XDG_CONFIG_HOME");
    if (base.isEmpty()) {
        base = QDir::homePath() + QStringLiteral("/.config");
    }
    return base + QStringLiteral("/pibridge/apps.toml");
}

QString appsLogDir() {
    QString base = qEnvironmentVariable("XDG_STATE_HOME");
    if (base.isEmpty()) {
        base = QDir::homePath() + QStringLiteral("/.local/state");
    }
    return base + QStringLiteral("/pibridge/apps");
}

AppsStore::AppsStore(QString path) : path_(std::move(path)) {}

QStringList AppsStore::load() {
    QFile f(path_);
    if (!f.open(QIODevice::ReadOnly)) {
        doc_ = {};
        return {};
    }
    QStringList w;
    doc_ = AppsDoc::parse(QString::fromUtf8(f.read(4 * 1024 * 1024)), &w);
    return w;
}

bool AppsStore::save(QString* error) {
    QDir().mkpath(QFileInfo(path_).absolutePath());
    QSaveFile f(path_);
    const QByteArray data = doc_.text().toUtf8();
    if (!f.open(QIODevice::WriteOnly) || f.write(data) != data.size() || !f.commit()) {
        if (error) {
            *error = QStringLiteral("Could not write %1: %2").arg(path_, f.errorString());
        }
        return false;
    }
    return true;
}

std::vector<OwnApp> fakeOwnApps(qint64 now) {
    OwnApp sim;
    sim.id = QStringLiteral("flight-sim");
    sim.name = QStringLiteral("Flight Sim (Wine)");
    sim.exec = QStringLiteral("/fake/tools/flight-sim.sh");
    sim.args = {QStringLiteral("-xr")};
    sim.workdir = QStringLiteral("/fake/games/flight-sim");
    sim.quadViews = true;
    sim.qvProfile = QStringLiteral("Flight Sim");
    sim.lastPlayed = now - 26 * 3600;
    OwnApp viewer;
    viewer.id = QStringLiteral("model-viewer");
    viewer.name = QStringLiteral("Model Viewer");
    viewer.exec = QStringLiteral("/fake/bin/model-viewer");
    viewer.env = {QStringLiteral("XR_RUNTIME_JSON=/fake/steamvr.json")};
    viewer.quadViews = false;
    return {sim, viewer};
}

} // namespace pibridge::app
