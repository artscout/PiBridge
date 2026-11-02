// SPDX-License-Identifier: MIT
// The desktop portal session (xdg-desktop-portal, KDE's backend): RemoteDesktop with ScreenCast in one session, so the
// same session gives the monitors' PipeWire streams and the pointer. The flow is the portal's:
//   RemoteDesktop.CreateSession -> SelectDevices (pointer + keyboard, persist_mode 2 = until revoked, the restore
//   token of the last time) -> ScreenCast.SelectSources (monitors, several, the cursor drawn in) -> RemoteDesktop.Start
//   (the streams: node, position, size; a new restore token) -> ScreenCast.OpenPipeWireRemote (the fd).
// With a valid restore token KDE starts it without asking; without one it asks on the desktop (`pibridge-desktop allow`).
// Each step's answer comes as the Response signal of a Request object, waited for in a local event loop.
#pragma once

#include <QDBusConnection>
#include <QObject>
#include <QString>
#include <QVariantMap>

#include <cstdint>
#include <optional>
#include <vector>

namespace pibridge::desktopsvc {

struct PortalStream {
    std::uint32_t node = 0;
    int x = 0, y = 0, w = 0, h = 0; // its place and size on the desktop (logical pixels)
};

struct PortalResult {
    bool ok = false;
    bool denied = false; // the user said no, or there was no answer in time (a dialog nobody saw)
    QString error;
    QString restoreToken; // the token for next time ("" when the portal gave none)
    std::vector<PortalStream> streams; // sorted left to right, then top to bottom
    int pipewireFd = -1;               // owned by the caller
};

/// "/org/freedesktop/portal/desktop/request/<sender>/<token>": the sender's unique name without ':' and '.' -> '_'.
QString requestPath(const QString& uniqueName, const QString& token);
/// The streams of RemoteDesktop.Start's results (a(ua{sv}) with position (ii) and size (ii)), sorted.
std::vector<PortalStream> parseStreams(const QVariant& streams);

class Portal : public QObject {
    Q_OBJECT
public:
    explicit Portal(QObject* parent = nullptr);
    ~Portal() override;
    /// Set up the session; `timeoutMs` per step (a dialog needs long, a restore none).
    PortalResult open(const QString& restoreToken, int timeoutMs);
    void close();
    bool isOpen() const { return !session_.isEmpty(); }
    /// The session ended on the portal's side (KDE's "stop sharing", a logout).
    bool closedByPortal() const { return closed_; }

    void pointerMotion(std::uint32_t node, double x, double y);
    void pointerButton(int evdevButton, bool down);
    void pointerWheel(int steps);

private slots:
    void onResponse(uint response, const QVariantMap& results);
    void onSessionClosed(const QVariantMap& details);

private:
    std::optional<QVariantMap> request(const QString& iface, const QString& method, const QList<QVariant>& args,
                                       QVariantMap options, int timeoutMs, PortalResult& r);
    void notify(const QString& method, const QList<QVariant>& args);

    QDBusConnection bus_;
    QString session_;
    bool closed_ = false;
    int tokenNo_ = 0;
    // the answer being waited for
    bool answered_ = false;
    uint response_ = 2;
    QVariantMap results_;
};

} // namespace pibridge::desktopsvc
