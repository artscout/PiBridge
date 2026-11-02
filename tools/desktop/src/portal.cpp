// SPDX-License-Identifier: MIT
#include "portal.h"

#include <QDBusArgument>
#include <QDBusMessage>
#include <QDBusObjectPath>
#include <QDBusReply>
#include <QDBusUnixFileDescriptor>
#include <QEventLoop>
#include <QTimer>

#include <algorithm>
#include <unistd.h>

namespace pibridge::desktopsvc {

namespace {

const QString kService = QStringLiteral("org.freedesktop.portal.Desktop");
const QString kPath = QStringLiteral("/org/freedesktop/portal/desktop");
const QString kRemote = QStringLiteral("org.freedesktop.portal.RemoteDesktop");
const QString kCast = QStringLiteral("org.freedesktop.portal.ScreenCast");

// RemoteDesktop device types, ScreenCast source types and cursor modes (the portal's bit values)
constexpr uint kKeyboard = 1, kPointer = 2;
constexpr uint kMonitor = 1;
constexpr uint kCursorEmbedded = 2;
constexpr uint kPersistUntilRevoked = 2;

bool pair(const QVariant& v, int& a, int& b) {
    if (!v.canConvert<QDBusArgument>()) {
        return false;
    }
    const QDBusArgument arg = v.value<QDBusArgument>();
    arg.beginStructure();
    arg >> a >> b;
    arg.endStructure();
    return true;
}

} // namespace

QString requestPath(const QString& uniqueName, const QString& token) {
    QString sender = uniqueName;
    if (sender.startsWith(QLatin1Char(':'))) {
        sender.remove(0, 1);
    }
    sender.replace(QLatin1Char('.'), QLatin1Char('_'));
    return QStringLiteral("/org/freedesktop/portal/desktop/request/") + sender + QLatin1Char('/') + token;
}

std::vector<PortalStream> parseStreams(const QVariant& streams) {
    std::vector<PortalStream> out;
    if (!streams.canConvert<QDBusArgument>()) {
        return out;
    }
    const QDBusArgument arg = streams.value<QDBusArgument>();
    arg.beginArray();
    while (!arg.atEnd()) {
        PortalStream s;
        QVariantMap props;
        arg.beginStructure();
        arg >> s.node >> props;
        arg.endStructure();
        pair(props.value(QStringLiteral("position")), s.x, s.y);
        pair(props.value(QStringLiteral("size")), s.w, s.h);
        out.push_back(s);
    }
    arg.endArray();
    std::stable_sort(out.begin(), out.end(), [](const PortalStream& a, const PortalStream& b) {
        return a.x != b.x ? a.x < b.x : a.y < b.y;
    });
    return out;
}

Portal::Portal(QObject* parent) : QObject(parent), bus_(QDBusConnection::sessionBus()) {}

Portal::~Portal() { close(); }

void Portal::onResponse(uint response, const QVariantMap& results) {
    answered_ = true;
    response_ = response;
    results_ = results;
}

void Portal::onSessionClosed(const QVariantMap&) {
    closed_ = true;
}

std::optional<QVariantMap> Portal::request(const QString& iface, const QString& method, const QList<QVariant>& args,
                                           QVariantMap options, int timeoutMs, PortalResult& r) {
    const QString token = QStringLiteral("pibridge%1_%2").arg(::getpid()).arg(++tokenNo_);
    options.insert(QStringLiteral("handle_token"), token);
    const QString path = requestPath(bus_.baseService(), token);
    // listen before calling: the answer may come at once
    bus_.connect(kService, path, QStringLiteral("org.freedesktop.portal.Request"), QStringLiteral("Response"), this,
                 SLOT(onResponse(uint, QVariantMap)));
    answered_ = false;
    QDBusMessage msg = QDBusMessage::createMethodCall(kService, kPath, iface, method);
    QList<QVariant> all = args;
    all << options;
    msg.setArguments(all);
    const QDBusMessage reply = bus_.call(msg, QDBus::Block, 10000);
    if (reply.type() == QDBusMessage::ErrorMessage) {
        bus_.disconnect(kService, path, QStringLiteral("org.freedesktop.portal.Request"), QStringLiteral("Response"),
                        this, SLOT(onResponse(uint, QVariantMap)));
        r.error = method + QStringLiteral(": ") + reply.errorMessage();
        return std::nullopt;
    }
    if (!answered_) {
        QEventLoop loop;
        QTimer poll;
        QObject::connect(&poll, &QTimer::timeout, &loop, [&loop, this] {
            if (answered_) {
                loop.quit();
            }
        });
        QTimer::singleShot(timeoutMs, &loop, &QEventLoop::quit);
        poll.start(20);
        loop.exec();
    }
    bus_.disconnect(kService, path, QStringLiteral("org.freedesktop.portal.Request"), QStringLiteral("Response"), this,
                    SLOT(onResponse(uint, QVariantMap)));
    if (!answered_) {
        r.denied = true;
        r.error = method + QStringLiteral(": no answer (a question on the desktop nobody answered?)");
        // the request is left open on the portal's side; closing it takes the dialog away
        bus_.call(QDBusMessage::createMethodCall(kService, path, QStringLiteral("org.freedesktop.portal.Request"),
                                                 QStringLiteral("Close")),
                  QDBus::Block, 2000);
        return std::nullopt;
    }
    if (response_ != 0) {
        r.denied = response_ == 1;
        r.error = method + (response_ == 1 ? QStringLiteral(": not allowed") : QStringLiteral(": failed"));
        return std::nullopt;
    }
    return results_;
}

PortalResult Portal::open(const QString& restoreToken, int timeoutMs) {
    PortalResult r;
    close();
    closed_ = false;
    QVariantMap opts;
    opts.insert(QStringLiteral("session_handle_token"), QStringLiteral("pibridge_desktop%1").arg(::getpid()));
    const auto created = request(kRemote, QStringLiteral("CreateSession"), {}, opts, 10000, r);
    if (!created) {
        return r;
    }
    session_ = created->value(QStringLiteral("session_handle")).toString();
    if (session_.isEmpty()) {
        r.error = QStringLiteral("CreateSession: no session");
        return r;
    }
    bus_.connect(kService, session_, QStringLiteral("org.freedesktop.portal.Session"), QStringLiteral("Closed"), this,
                 SLOT(onSessionClosed(QVariantMap)));
    const QVariant session = QVariant::fromValue(QDBusObjectPath(session_));
    QVariantMap dev;
    dev.insert(QStringLiteral("types"), kKeyboard | kPointer);
    dev.insert(QStringLiteral("persist_mode"), kPersistUntilRevoked);
    if (!restoreToken.isEmpty()) {
        dev.insert(QStringLiteral("restore_token"), restoreToken);
    }
    if (!request(kRemote, QStringLiteral("SelectDevices"), {session}, dev, 10000, r)) {
        close();
        return r;
    }
    QVariantMap src;
    src.insert(QStringLiteral("types"), kMonitor);
    src.insert(QStringLiteral("multiple"), true);
    src.insert(QStringLiteral("cursor_mode"), kCursorEmbedded);
    if (!request(kCast, QStringLiteral("SelectSources"), {session}, src, 10000, r)) {
        close();
        return r;
    }
    const auto started = request(kRemote, QStringLiteral("Start"), {session, QString()}, {}, timeoutMs, r);
    if (!started) {
        close();
        return r;
    }
    r.restoreToken = started->value(QStringLiteral("restore_token")).toString();
    r.streams = parseStreams(started->value(QStringLiteral("streams")));
    if (r.streams.empty()) {
        r.error = QStringLiteral("Start: no monitors were shared");
        close();
        return r;
    }
    QDBusMessage pwMsg = QDBusMessage::createMethodCall(kService, kPath, kCast, QStringLiteral("OpenPipeWireRemote"));
    pwMsg.setArguments({session, QVariantMap()});
    const QDBusReply<QDBusUnixFileDescriptor> fd = bus_.call(pwMsg, QDBus::Block, 10000);
    if (!fd.isValid() || !fd.value().isValid()) {
        r.error = QStringLiteral("OpenPipeWireRemote: ") + fd.error().message();
        close();
        return r;
    }
    r.pipewireFd = ::dup(fd.value().fileDescriptor());
    r.ok = r.pipewireFd >= 0;
    if (!r.ok) {
        r.error = QStringLiteral("OpenPipeWireRemote: no fd");
        close();
    }
    return r;
}

void Portal::close() {
    if (session_.isEmpty()) {
        return;
    }
    bus_.disconnect(kService, session_, QStringLiteral("org.freedesktop.portal.Session"), QStringLiteral("Closed"), this,
                    SLOT(onSessionClosed(QVariantMap)));
    bus_.call(QDBusMessage::createMethodCall(kService, session_, QStringLiteral("org.freedesktop.portal.Session"),
                                             QStringLiteral("Close")),
              QDBus::Block, 2000);
    session_.clear();
}

void Portal::notify(const QString& method, const QList<QVariant>& args) {
    if (session_.isEmpty()) {
        return;
    }
    QDBusMessage msg = QDBusMessage::createMethodCall(kService, kPath, kRemote, method);
    QList<QVariant> all{QVariant::fromValue(QDBusObjectPath(session_)), QVariantMap()};
    all << args;
    msg.setArguments(all);
    bus_.send(msg); // no answer is waited for: the pointer keeps up with the laser
}

void Portal::pointerMotion(std::uint32_t node, double x, double y) {
    notify(QStringLiteral("NotifyPointerMotionAbsolute"), {QVariant::fromValue(static_cast<uint>(node)), x, y});
}

void Portal::pointerButton(int evdevButton, bool down) {
    notify(QStringLiteral("NotifyPointerButton"), {evdevButton, static_cast<uint>(down ? 1 : 0)});
}

void Portal::pointerWheel(int steps) {
    notify(QStringLiteral("NotifyPointerAxisDiscrete"), {static_cast<uint>(0), steps});
}

} // namespace pibridge::desktopsvc
