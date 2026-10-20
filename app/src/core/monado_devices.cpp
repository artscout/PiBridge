// SPDX-License-Identifier: MIT
#include "monado_devices.h"

#include <QFile>
#include <QFileInfo>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <dlfcn.h>

namespace pibridge::app {

// ---------------------------------------------------------------- libmonado (dlopen)

namespace {

// The few declarations of libmonado's monado.h (API 1.x) PiBridge uses; the values are the header's.
struct mnd_root;
using mnd_result = int;
constexpr mnd_result kMndSuccess = 0;
constexpr unsigned kMndPropertyName = 0; // MND_PROPERTY_NAME_STRING

using FnVersion = void (*)(std::uint32_t*, std::uint32_t*, std::uint32_t*);
using FnCreate = mnd_result (*)(mnd_root**);
using FnDestroy = void (*)(mnd_root**);
using FnCount = mnd_result (*)(mnd_root*, std::uint32_t*);
using FnString = mnd_result (*)(mnd_root*, std::uint32_t, unsigned, const char**);
using FnInfo = mnd_result (*)(mnd_root*, std::uint32_t, std::uint32_t*, const char**); // deprecated name lookup
using FnRole = mnd_result (*)(mnd_root*, const char*, std::int32_t*);
using FnBattery = mnd_result (*)(mnd_root*, std::uint32_t, bool*, bool*, float*);

class LibMonadoSource : public MonadoSource {
public:
    ~LibMonadoSource() override { disconnect(); } // the library itself stays loaded (it may have threads)

    MndConnect connect(const QString& library) override {
        disconnect();
        if (!handle_ || library != loaded_) {
            if (library.isEmpty() || !QFileInfo(library).isFile()) {
                return {MndConnect::NoLibrary, QStringLiteral("libmonado.so was not found in the Monado build.")};
            }
            void* h = ::dlopen(QFile::encodeName(library).constData(), RTLD_NOW | RTLD_LOCAL);
            if (!h) {
                const char* e = ::dlerror();
                return {MndConnect::NoLibrary,
                        QStringLiteral("libmonado could not be loaded: %1").arg(QString::fromLocal8Bit(e ? e : "?"))};
            }
            const auto sym = [h](const char* n) { return ::dlsym(h, n); };
            version_ = reinterpret_cast<FnVersion>(sym("mnd_api_get_version"));
            create_ = reinterpret_cast<FnCreate>(sym("mnd_root_create"));
            destroy_ = reinterpret_cast<FnDestroy>(sym("mnd_root_destroy"));
            count_ = reinterpret_cast<FnCount>(sym("mnd_root_get_device_count"));
            string_ = reinterpret_cast<FnString>(sym("mnd_root_get_device_info_string"));
            info_ = reinterpret_cast<FnInfo>(sym("mnd_root_get_device_info"));
            role_ = reinterpret_cast<FnRole>(sym("mnd_root_get_device_from_role"));
            battery_ = reinterpret_cast<FnBattery>(sym("mnd_root_get_device_battery_status"));
            if (!version_ || !create_ || !destroy_ || !count_ || !role_ || (!string_ && !info_)) {
                return {MndConnect::NoLibrary, QStringLiteral("This libmonado lacks the device functions.")};
            }
            std::uint32_t major = 0, minor = 0, patch = 0;
            version_(&major, &minor, &patch);
            if (major != 1) {
                return {MndConnect::NoLibrary, QStringLiteral("libmonado %1.%2 is not the 1.x API PiBridge knows.")
                                                   .arg(major)
                                                   .arg(minor)};
            }
            handle_ = h;
            loaded_ = library;
        }
        mnd_root* r = nullptr;
        if (create_(&r) != kMndSuccess || r == nullptr) {
            return {MndConnect::Failed, QStringLiteral("Monado did not answer.")};
        }
        root_ = r;
        return {MndConnect::Ok, {}};
    }

    std::optional<std::vector<MonadoDevice>> poll() override {
        if (!root_) {
            return std::nullopt;
        }
        std::uint32_t n = 0;
        if (count_(root_, &n) != kMndSuccess) {
            return std::nullopt;
        }
        std::int32_t left = -1, right = -1, head = -1;
        if (role_(root_, "left", &left) != kMndSuccess || role_(root_, "right", &right) != kMndSuccess ||
            role_(root_, "head", &head) != kMndSuccess) {
            return std::nullopt;
        }
        std::vector<MonadoDevice> out;
        for (std::uint32_t i = 0; i < n && i < 64; i++) {
            MonadoDevice d;
            d.index = static_cast<int>(i);
            const char* name = nullptr;
            if (string_) {
                (void)string_(root_, i, kMndPropertyName, &name);
            } else {
                std::uint32_t id = 0;
                (void)info_(root_, i, &id, &name);
            }
            d.name = QString::fromUtf8(name ? name : "");
            d.role = static_cast<std::int32_t>(i) == left ? VrRole::Left
                     : static_cast<std::int32_t>(i) == right ? VrRole::Right
                                                              : VrRole::None;
            d.head = static_cast<std::int32_t>(i) == head;
            if (battery_) {
                bool present = false, charging = false;
                float charge = 0;
                // an error here only means the device has no battery (MND_ERROR_OPERATION_FAILED for those)
                if (battery_(root_, i, &present, &charging, &charge) == kMndSuccess) {
                    d.batterySupported = true;
                    d.batteryPresent = present;
                    d.charging = charging;
                    d.battery = charge;
                }
            }
            out.push_back(d);
        }
        return out;
    }

    void disconnect() override {
        if (root_ && destroy_) {
            destroy_(&root_);
        }
        root_ = nullptr;
    }

private:
    void* handle_ = nullptr;
    QString loaded_;
    mnd_root* root_ = nullptr;
    FnVersion version_ = nullptr;
    FnCreate create_ = nullptr;
    FnDestroy destroy_ = nullptr;
    FnCount count_ = nullptr;
    FnString string_ = nullptr;
    FnInfo info_ = nullptr;
    FnRole role_ = nullptr;
    FnBattery battery_ = nullptr;
};

} // namespace

std::unique_ptr<MonadoSource> makeLibMonadoSource() {
    return std::make_unique<LibMonadoSource>();
}

// ---------------------------------------------------------------- fake

MndConnect FakeMonadoSource::connect(const QString& library) {
    libraries << library;
    MndConnect c;
    c.result = MndConnect::Ok;
    if (!connects.empty()) {
        c = connects.front();
        if (connects.size() > 1) {
            connects.erase(connects.begin());
        }
    }
    connected = c.result == MndConnect::Ok;
    return c;
}

std::optional<std::vector<MonadoDevice>> FakeMonadoSource::poll() {
    pollCalls++;
    if (!connected) {
        return std::nullopt;
    }
    std::optional<std::vector<MonadoDevice>> p = std::vector<MonadoDevice>{};
    if (!polls.empty()) {
        p = polls.front();
        if (polls.size() > 1) {
            polls.erase(polls.begin());
        }
    }
    if (!p) {
        connected = false;
    }
    return p;
}

void FakeMonadoSource::disconnect() {
    disconnectCalls++;
    connected = false;
}

std::vector<MonadoDevice> fakeMonadoDevices() {
    MonadoDevice hmd;
    hmd.index = 0;
    hmd.name = QStringLiteral("Pimax Crystal Super");
    hmd.head = true;
    MonadoDevice l;
    l.index = 1;
    l.name = QStringLiteral("Pimax Crystal Super Left Controller");
    l.role = VrRole::Left;
    l.batterySupported = true;
    l.batteryPresent = true;
    l.charging = true;
    l.battery = 0.64F;
    MonadoDevice r = l;
    r.index = 2;
    r.name = QStringLiteral("Pimax Crystal Super Right Controller");
    r.role = VrRole::Right;
    r.batteryPresent = false;
    r.charging = false;
    r.battery = 0;
    return {hmd, l, r};
}

// ---------------------------------------------------------------- the link

MonadoLink::MonadoLink(MonadoSource& source, Clock clock, Exists exists)
    : source_(source), clock_(std::move(clock)),
      exists_(exists ? std::move(exists) : Exists([](const QString& p) { return QFileInfo::exists(p); })) {}

MonadoLink::~MonadoLink() {
    stop();
}

void MonadoLink::stop() {
    if (connected_) {
        source_.disconnect();
        connected_ = false;
    }
}

MonadoStatus MonadoLink::tick(const MonadoTarget& t) {
    MonadoStatus s;
    s.library = t.library;
    if (!t.wanted) {
        stop();
        last_ = s;
        return s;
    }
    if (t.library.isEmpty()) {
        stop();
        s.state = MndLinkState::NoLibrary;
        s.error = QStringLiteral("libmonado.so was not found in the Monado build next to monado-service.");
        last_ = s;
        return s;
    }
    if (!t.socketPath.isEmpty() && !exists_(t.socketPath)) {
        stop(); // Monado went away (or is not up yet): try at once when its socket is back
        lastAttemptMs_.reset();
        s.state = MndLinkState::NotRunning;
        last_ = s;
        return s;
    }
    const qint64 now = clock_();
    if (!connectedLib_.isEmpty() && connectedLib_ != t.library) {
        stop(); // another Monado build: connect again, at once
        connectedLib_.clear();
        lastAttemptMs_.reset();
    }
    if (connected_) {
        if (auto d = source_.poll()) {
            s.state = MndLinkState::Connected;
            s.devices = std::move(*d);
            last_ = s;
            return s;
        }
        stop();
        lastAttemptMs_ = now;
        s.state = MndLinkState::Waiting;
        s.error = QStringLiteral("Monado stopped answering.");
        last_ = s;
        return s;
    }
    if (lastAttemptMs_ && now - *lastAttemptMs_ < kMndRetryMs) {
        s.state = last_.state == MndLinkState::NoLibrary ? MndLinkState::NoLibrary : MndLinkState::Waiting;
        s.error = last_.error;
        return s;
    }
    lastAttemptMs_ = now;
    const MndConnect c = source_.connect(t.library);
    switch (c.result) {
    case MndConnect::Ok:
        connected_ = true;
        connectedLib_ = t.library;
        if (auto d = source_.poll()) {
            s.state = MndLinkState::Connected;
            s.devices = std::move(*d);
        } else {
            stop();
            s.state = MndLinkState::Waiting;
            s.error = QStringLiteral("Monado stopped answering.");
        }
        break;
    case MndConnect::NoLibrary:
        s.state = MndLinkState::NoLibrary;
        s.error = c.error;
        break;
    case MndConnect::Failed:
        s.state = MndLinkState::Waiting;
        s.error = c.error;
        break;
    }
    last_ = s;
    return s;
}

// ---------------------------------------------------------------- the worker

MonadoWorker::MonadoWorker(std::unique_ptr<MonadoSource> source, Deliver deliver, int intervalMs)
    : source_(std::move(source)), deliver_(std::move(deliver)), intervalMs_(intervalMs) {
    thread_ = std::jthread([this](std::stop_token st) {
        MonadoLink link(*source_, [] {
            return static_cast<qint64>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::steady_clock::now().time_since_epoch())
                                           .count());
        });
        MonadoTarget last;
        bool first = true;
        while (!st.stop_requested()) {
            MonadoTarget t;
            {
                std::lock_guard lk(m_);
                t = target_;
                changed_ = false;
            }
            // nothing to do while not wanted: deliver only the change to Off
            if (t.wanted || first || !(t == last)) {
                deliver_(link.tick(t));
            }
            first = false;
            last = t;
            std::unique_lock lk(m_);
            cv_.wait_for(lk, st, std::chrono::milliseconds(intervalMs_), [this] { return changed_; });
        }
        link.stop(); // mnd_root_destroy on the thread that connected
    });
}

MonadoWorker::~MonadoWorker() {
    thread_.request_stop();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void MonadoWorker::setTarget(const MonadoTarget& t) {
    {
        std::lock_guard lk(m_);
        if (target_ == t) {
            return;
        }
        target_ = t;
        changed_ = true;
    }
    cv_.notify_all();
}

// ---------------------------------------------------------------- words

namespace {

ControllerRow monadoRow(const MonadoDevice* d) {
    ControllerRow r;
    if (!d) {
        return r;
    }
    r.present = true;
    r.name = d->name.isEmpty() ? QStringLiteral("Controller") : d->name;
    if (!d->batterySupported) {
        r.chip = {QStringLiteral("LISTED"), ChipKind::Info};
        r.detail = QStringLiteral("Monado does not report its state");
        return r;
    }
    if (!d->batteryPresent) {
        r.chip = {QStringLiteral("OFF"), ChipKind::Off};
        r.detail = QStringLiteral("Turn it on");
        return r;
    }
    r.chip = {QStringLiteral("ON"), ChipKind::Info};
    r.batteryPercent = std::clamp(static_cast<int>(d->battery * 100.0F + 0.5F), 0, 100);
    r.charging = d->charging;
    r.batteryText = QStringLiteral("%1 %%2").arg(r.batteryPercent).arg(d->charging ? QStringLiteral(" charging") : QString());
    return r;
}

} // namespace

ControllersView monadoControllersView(const MonadoStatus& s) {
    ControllersView v;
    switch (s.state) {
    case MndLinkState::Off:
        v.header = {QStringLiteral("MONADO OFF"), ChipKind::Off};
        v.message = QStringLiteral("Start tracking (Tracking page) to see the controllers.");
        return v;
    case MndLinkState::NoLibrary:
        v.header = {QStringLiteral("NO LIBMONADO"), ChipKind::Off};
        v.message = (s.error.isEmpty() ? QStringLiteral("libmonado was not found.") : s.error) +
                    QStringLiteral(" The controllers cannot be read.");
        return v;
    case MndLinkState::NotRunning:
        v.header = {QStringLiteral("MONADO STARTING"), ChipKind::Off};
        v.message = QStringLiteral("Waiting for Monado...");
        return v;
    case MndLinkState::Waiting:
        v.header = {QStringLiteral("CONNECTING"), ChipKind::Off};
        v.message = s.error.isEmpty() ? QStringLiteral("Connecting to Monado...")
                                      : QStringLiteral("Connecting to Monado... (%1)").arg(s.error);
        return v;
    case MndLinkState::Connected: break;
    }
    const MonadoDevice* left = nullptr;
    const MonadoDevice* right = nullptr;
    for (const MonadoDevice& d : s.devices) {
        if (d.role == VrRole::Left && !left) {
            left = &d;
        } else if (d.role == VrRole::Right && !right) {
            right = &d;
        } else if (!d.head && d.batterySupported) {
            v.extra << QStringLiteral("%1: %2").arg(d.name, d.batteryPresent ? QStringLiteral("on") : QStringLiteral("off"));
        }
    }
    v.left = monadoRow(left);
    v.right = monadoRow(right);
    const auto on = [](const ControllerRow& r) { return r.present && r.chip.kind == ChipKind::Info; };
    const int n = (on(v.left) ? 1 : 0) + (on(v.right) ? 1 : 0);
    v.header = {QStringLiteral("MONADO: %1/2 ON").arg(n), n == 2 ? ChipKind::Info : n == 1 ? ChipKind::Warn : ChipKind::Off};
    if (n == 0) {
        v.message = QStringLiteral("Turn the controllers on.");
    }
    return v;
}

} // namespace pibridge::app
