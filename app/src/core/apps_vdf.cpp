// SPDX-License-Identifier: MIT
#include "apps_vdf.h"

#include <QFile>

namespace pibridge::app {

const VdfNode* VdfNode::child(QStringView name) const {
    for (const VdfNode& c : children) {
        if (QStringView(c.key).compare(name, Qt::CaseInsensitive) == 0) {
            return &c;
        }
    }
    return nullptr;
}

QString VdfNode::str(QStringView name, const QString& def) const {
    const VdfNode* c = child(name);
    return c && !c->block ? c->value : def;
}

qint64 VdfNode::num(QStringView name, qint64 def) const {
    bool ok = false;
    const qint64 v = str(name).trimmed().toLongLong(&ok);
    return ok ? v : def;
}

namespace {

constexpr int kMaxDepth = 64; // far deeper than any Steam file; stops a hostile file from exhausting the stack

class Lexer {
public:
    explicit Lexer(QStringView t) : t_(t) {}

    enum Kind { End, Open, Close, String, Bad };
    struct Token {
        Kind kind = End;
        QString text;
    };

    Token next() {
        skipSpace();
        if (i_ >= t_.size()) {
            return {End, {}};
        }
        const QChar c = t_[i_];
        if (c == QLatin1Char('{')) {
            i_++;
            return {Open, {}};
        }
        if (c == QLatin1Char('}')) {
            i_++;
            return {Close, {}};
        }
        if (c == QLatin1Char('"')) {
            return quoted();
        }
        // a bare word: up to white space, a brace or a quote
        QString w;
        while (i_ < t_.size()) {
            const QChar d = t_[i_];
            if (d.isSpace() || d == QLatin1Char('{') || d == QLatin1Char('}') || d == QLatin1Char('"')) {
                break;
            }
            w += d;
            i_++;
        }
        return {String, w};
    }

    qsizetype line() const {
        qsizetype n = 1;
        for (qsizetype k = 0; k < i_ && k < t_.size(); k++) {
            n += t_[k] == QLatin1Char('\n') ? 1 : 0;
        }
        return n;
    }

private:
    void skipSpace() {
        while (i_ < t_.size()) {
            const QChar c = t_[i_];
            if (c.isSpace() || c == QChar(0xFEFF)) {
                i_++;
            } else if (c == QLatin1Char('/') && i_ + 1 < t_.size() && t_[i_ + 1] == QLatin1Char('/')) {
                skipLine();
            } else if (c == QLatin1Char('#') && atLineStart()) {
                skipLine(); // #include / #base: not followed, Steam's own files do not use them
            } else if (c == QLatin1Char('[')) {
                // a [$WIN32] condition tag after a value: ignored
                const qsizetype close = t_.indexOf(QLatin1Char(']'), i_);
                const qsizetype nl = t_.indexOf(QLatin1Char('\n'), i_);
                if (close < 0 || (nl >= 0 && nl < close)) {
                    return; // not a tag: the next token reads it as a word
                }
                i_ = close + 1;
            } else {
                return;
            }
        }
    }

    bool atLineStart() const {
        for (qsizetype k = i_ - 1; k >= 0; k--) {
            if (t_[k] == QLatin1Char('\n')) {
                return true;
            }
            if (!t_[k].isSpace()) {
                return false;
            }
        }
        return true;
    }

    void skipLine() {
        while (i_ < t_.size() && t_[i_] != QLatin1Char('\n')) {
            i_++;
        }
    }

    Token quoted() {
        i_++; // opening quote
        QString s;
        while (i_ < t_.size()) {
            const QChar c = t_[i_++];
            if (c == QLatin1Char('"')) {
                return {String, s};
            }
            if (c == QLatin1Char('\\') && i_ < t_.size()) {
                const QChar e = t_[i_++];
                switch (e.unicode()) {
                case 'n': s += QLatin1Char('\n'); break;
                case 't': s += QLatin1Char('\t'); break;
                case '\\': s += QLatin1Char('\\'); break;
                case '"': s += QLatin1Char('"'); break;
                default: // Steam writes Windows paths with doubled backslashes; anything else stays as written
                    s += QLatin1Char('\\');
                    s += e;
                    break;
                }
                continue;
            }
            s += c;
        }
        return {Bad, QStringLiteral("a quoted string is not closed")};
    }

    QStringView t_;
    qsizetype i_ = 0;
};

// Reads entries into `into` until the closing brace (or the end at depth 0). Returns false on an error.
bool parseBlock(Lexer& lx, VdfNode& into, int depth, QString& err) {
    if (depth > kMaxDepth) {
        err = QStringLiteral("blocks nested too deep");
        return false;
    }
    for (;;) {
        Lexer::Token k = lx.next();
        switch (k.kind) {
        case Lexer::End:
            if (depth > 0) {
                err = QStringLiteral("a block is not closed");
                return false;
            }
            return true;
        case Lexer::Close:
            if (depth == 0) {
                err = QStringLiteral("a '}' without a block (line %1)").arg(lx.line());
                return false;
            }
            return true;
        case Lexer::Open:
            err = QStringLiteral("a '{' without a key (line %1)").arg(lx.line());
            return false;
        case Lexer::Bad:
            err = k.text;
            return false;
        case Lexer::String: break;
        }
        VdfNode node;
        node.key = k.text;
        Lexer::Token v = lx.next();
        if (v.kind == Lexer::String) {
            node.value = v.text;
            into.children.push_back(std::move(node));
            continue;
        }
        if (v.kind == Lexer::Open) {
            node.block = true;
            const bool ok = parseBlock(lx, node, depth + 1, err);
            into.children.push_back(std::move(node));
            if (!ok) {
                return false;
            }
            continue;
        }
        err = v.kind == Lexer::Bad ? v.text : QStringLiteral("the key \"%1\" has no value").arg(node.key);
        return false;
    }
}

} // namespace

VdfDoc parseVdf(QStringView text) {
    VdfDoc d;
    d.root.block = true;
    Lexer lx(text);
    parseBlock(lx, d.root, 0, d.error);
    return d;
}

VdfDoc readVdfFile(const QString& path) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        VdfDoc d;
        d.root.block = true;
        d.error = QStringLiteral("cannot read %1").arg(path);
        return d;
    }
    // Steam's files are a few KB; a cap keeps a stray huge file from being read whole.
    const QString text = QString::fromUtf8(f.read(8 * 1024 * 1024));
    return parseVdf(text);
}

} // namespace pibridge::app
