// SPDX-License-Identifier: MIT
// PiBridge: the desktop app (binary `pibridge`).
//   pibridge                   the app
//   pibridge --fake            fixed fake state, temporary settings dirs, no commands run (also PIBRIDGE_FAKE_STATE=1)
//   pibridge --screenshots DIR render every page with the fake state into DIR/*.png and quit
// The app never runs a command at start; commands only run when a button is clicked.
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QLockFile>
#include <QSocketNotifier>
#include <QScrollArea>
#include <QScrollBar>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <csignal>
#include <cstdio>
#include <sys/signalfd.h>
#include <unistd.h>
#include <functional>
#include <memory>

#include "core/runner.h"
#include "core/state.h"
#include "core/system.h"
#include "mainwindow.h"
#include "core/conf.h"
#include "core/inside_out.h"
#include "core/paths.h"
#include "pages/dashboard.h"
#include "pages/help_window.h"
#include "pages/record_card.h"
#include "ui/brand.h"
#include "ui/icons.h"
#include "ui/theme.h"

namespace {

bool writeFile(const QString& path, const QByteArray& data) {
    QDir().mkpath(QFileInfo(path).absolutePath());
    QFile f(path);
    return f.open(QIODevice::WriteOnly | QIODevice::Truncate) && f.write(data) == data.size();
}

// Fake mode: every XDG dir points into `tmp`, so nothing of the user's is read or written; a few files make the
// pages look as with a real install (the layer switched on, one game profile, the last game's status).
void setUpFakeHome(const QString& tmp) {
    for (const char* var : {"XDG_CONFIG_HOME", "XDG_STATE_HOME", "XDG_DATA_HOME", "XDG_RUNTIME_DIR", "XDG_CACHE_HOME"}) {
        const QString d = tmp + QLatin1Char('/') + QString::fromLatin1(var).toLower();
        QDir().mkpath(d);
        qputenv(var, QFile::encodeName(d));
    }
    qputenv("XDG_DATA_DIRS", QFile::encodeName(tmp + QStringLiteral("/none"))); // no system-wide layer either
    qputenv("XDG_CONFIG_DIRS", QFile::encodeName(tmp + QStringLiteral("/none")));
    const QString data = tmp + QStringLiteral("/xdg_data_home");
    writeFile(data + QStringLiteral("/openxr/1/api_layers/implicit.d/XR_APILAYER_ARTSCOUT_quad_views_vulkan.json"),
              "{\"file_format_version\": \"1.0.0\"}\n");
    writeFile(tmp + QStringLiteral("/xdg_config_home/qv-vulkan/settings.conf"),
              "peripheral_density=0.20\nfocus_density=1.00\ngaze_width=0.35\ngaze_height=0.35\n"
              "\n[exe:Falcon_BMS]\nperipheral_density=0.30\nfocus_density=1.25\ngaze_width=0.40\ngaze_height=0.40\n");
    writeFile(tmp + QStringLiteral("/xdg_state_home/qv-vulkan/status.txt"),
              "app=Falcon BMS\nexe=Falcon_BMS.exe\nruntime=SteamVR/OpenXR\nsystem=Crystal Super\nsession=ended\n"
              "eye_tracking=available\nperipheral=710x768\nfocus=1244x1344\ncanvas=3552x3840\n"
              "recommended=3552x3840\ncompose_gpu_ms=0.41\n");
}

} // namespace

int main(int argc, char** argv) {
    QString shotsDir;
    bool fake = qEnvironmentVariableIntValue("PIBRIDGE_FAKE_STATE") != 0;
    QString iconsDir;
    for (int i = 1; i < argc; i++) {
        const QByteArray a = argv[i];
        if (a == "--fake") {
            fake = true;
        } else if (a == "--screenshots" && i + 1 < argc) {
            shotsDir = QFile::decodeName(argv[++i]);
            fake = true;
        } else if (a == "--export-icons" && i + 1 < argc) {
            iconsDir = QFile::decodeName(argv[++i]);
            qputenv("QT_QPA_PLATFORM", "offscreen");
        }
    }
    if (!shotsDir.isEmpty() && qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        qputenv("QT_QPA_PLATFORM", "offscreen");
    }
    std::unique_ptr<QTemporaryDir> tmp;
    if (fake) {
        tmp = std::make_unique<QTemporaryDir>(QDir::tempPath() + QStringLiteral("/pibridge-fake-XXXXXX"));
        setUpFakeHome(tmp->path());
    }

    QApplication app(argc, argv);
    if (!iconsDir.isEmpty()) {
        // write the faceted-gem app icon (icons.cpp appIcon()) as PNGs for the desktop/KDE theme, so no rasterizer is
        // needed to regenerate data/icons/pibridge-NN.png from the SVG.
        QDir().mkpath(iconsDir);
        const QIcon ic = pibridge::ui::appIcon();
        for (const int s : {16, 24, 32, 48, 64, 128, 256, 512}) {
            const QPixmap pm = ic.pixmap(QSize(s, s));
            if (!pm.save(iconsDir + QStringLiteral("/pibridge-%1.png").arg(s))) {
                std::fprintf(stderr, "pibridge: could not write the %dpx icon to %s\n", s, qUtf8Printable(iconsDir));
            }
        }
        std::printf("wrote the gem icons to %s\n", qUtf8Printable(iconsDir));
        return 0;
    }
    QApplication::setApplicationName(QStringLiteral("pibridge"));
    QApplication::setApplicationDisplayName(pibridge::ui::productName());
    QApplication::setApplicationVersion(QStringLiteral(PIBRIDGE_VERSION));
    QApplication::setDesktopFileName(QStringLiteral("pibridge"));
    QApplication::setWindowIcon(pibridge::ui::appIcon()); // the pibridgeed gem (icons.cpp), not the installed theme icon
    pibridge::ui::applyTheme(app);

    pibridge::app::StateSource source(fake);
    pibridge::app::FakeRunner fakeRunner;
    pibridge::app::ProcessRunner processRunner;
    pibridge::app::Runner& runner = fake ? static_cast<pibridge::app::Runner&>(fakeRunner) : processRunner;
    pibridge::app::Actions::ToolFinder finder = fake ? pibridge::app::Actions::ToolFinder([](const QString& n) {
        return QStringLiteral("/fake/") + n;
    })
                                             : pibridge::app::Actions::ToolFinder(&pibridge::app::findTool);
    pibridge::app::MainWindow w(source, runner, finder);

    if (shotsDir.isEmpty()) {
        // one PiBridge at a time (live 2026-10-19: a forgotten second PiBridge put SteamVR's drivers back every 2 s and
        // broke the other one's START TRACKING): a second start only brings the running window to the front
        std::unique_ptr<QLockFile> lock;
        std::unique_ptr<QSocketNotifier> raise;
        int sfd = -1;
        if (!fake) {
            const QString dir = pibridge::app::statusDir();
            QDir().mkpath(dir);
            lock = std::make_unique<QLockFile>(dir + QStringLiteral("/pibridge-app.lock"));
            lock->setStaleLockTime(0);
            if (!lock->tryLock(100)) {
                qint64 pid = 0;
                QString host, appName;
                if (lock->getLockInfo(&pid, &host, &appName) && pid > 0 && ::kill(static_cast<pid_t>(pid), SIGUSR1) == 0) {
                    std::printf("pibridge: already running (PID %lld): its window is brought to the front\n",
                                static_cast<long long>(pid));
                    return 0;
                }
                lock->removeStaleLockFile();
                if (!lock->tryLock(100)) {
                    std::fprintf(stderr, "pibridge: another PiBridge holds %s\n", qPrintable(lock->fileName()));
                    return 1;
                }
            }
            sigset_t set;
            sigemptyset(&set);
            sigaddset(&set, SIGUSR1);
            pthread_sigmask(SIG_BLOCK, &set, nullptr);
            sfd = ::signalfd(-1, &set, SFD_NONBLOCK | SFD_CLOEXEC);
            if (sfd >= 0) {
                raise = std::make_unique<QSocketNotifier>(sfd, QSocketNotifier::Read);
                QObject::connect(raise.get(), &QSocketNotifier::activated, &w, [&w, sfd] {
                    signalfd_siginfo si{};
                    while (::read(sfd, &si, sizeof si) == static_cast<ssize_t>(sizeof si)) {
                    }
                    w.showNormal();
                    w.raise();
                    w.activateWindow();
                });
            }
        }
        w.show();
        const int rc = app.exec();
        if (sfd >= 0) {
            ::close(sfd);
        }
        return rc;
    }

    // Screenshots: every page at a fixed size, the dashboard in both fake states.
    QDir().mkpath(shotsDir);
    w.resize(1600, 1000);
    w.show();
    struct Shot {
        int page;
        bool active;
        QString name;
        bool scrolledDown = false;
        std::function<void(pibridge::app::MainWindow&)> prepare = {};
    };
    const auto dashboard = [](pibridge::app::MainWindow& win) {
        return static_cast<pibridge::app::DashboardPage*>(win.page(pibridge::app::MainWindow::Dashboard));
    };
    const auto reshow = [](pibridge::app::MainWindow& win, int page) {
        win.showPage(pibridge::app::MainWindow::About);
        win.showPage(page);
    };
    const auto settle = [](int ms) {
        QElapsedTimer t;
        t.start();
        while (t.elapsed() < ms) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
            QThread::msleep(10);
        }
    };
    const auto setConf = [](std::initializer_list<std::pair<const char*, const char*>> kv) {
        pibridge::app::Conf c(pibridge::app::confPath());
        for (const auto& [k, v] : kv) {
            c.set(k, v);
        }
    };
    const std::vector<Shot> shots{
        {pibridge::app::MainWindow::Dashboard, true, QStringLiteral("01-dashboard")},
        {pibridge::app::MainWindow::Dashboard, false, QStringLiteral("02-dashboard-idle")},
        {pibridge::app::MainWindow::Apps, true, QStringLiteral("06-apps")},
        {pibridge::app::MainWindow::Apps, false, QStringLiteral("06b-apps-steamvr-off")},
        {pibridge::app::MainWindow::Performance, true, QStringLiteral("03-performance-quadviews")},
        {pibridge::app::MainWindow::Performance, true, QStringLiteral("03b-performance-quadviews-lower"), true},
        {pibridge::app::MainWindow::Tracking, false, QStringLiteral("09-tracking")},
        {pibridge::app::MainWindow::Tracking, false, QStringLiteral("09b-tracking-lower"), true},
        {pibridge::app::MainWindow::Tracking, false, QStringLiteral("09c-tracking-inside-out-ready"), false,
         [&](pibridge::app::MainWindow& win) {
             setConf({{"tracking_mode", "inside-out"}});
             reshow(win, pibridge::app::MainWindow::Tracking);
         }},
        {pibridge::app::MainWindow::Apps, false, QStringLiteral("06c-apps-inside-out-steamvr-off")},
        {pibridge::app::MainWindow::Tracking, false, QStringLiteral("09d-tracking-running"), false,
         [&](pibridge::app::MainWindow& win) { win.trackingSession().demoRun(); }},
        {pibridge::app::MainWindow::Dashboard, false, QStringLiteral("09e-dashboard-tracking-running")},
        {pibridge::app::MainWindow::Room, false, QStringLiteral("08c-room-inside-out"), false,
         [&](pibridge::app::MainWindow& win) {
             win.trackingSession().stop();
             setConf({{"io_anchor", "on"}});
             reshow(win, pibridge::app::MainWindow::Room);
         }},
        // the Record card while a recording runs, and after one ended (pibridge-record, docs/record.md)
        {pibridge::app::MainWindow::Dashboard, false, QStringLiteral("09h-dashboard-recording"), false,
         [&](pibridge::app::MainWindow& win) {
             pibridge::app::RecordInputs in;
             in.recorderRunning = in.directRunning = true;
             in.status.state = "recording";
             in.status.file = "/fake/home/Videos/pimax/pimax-20261019-201500.mp4";
             in.status.eyes = "left";
             in.status.fps = 30;
             in.status.quality = "high";
             in.status.encoder = "h264_nvenc";
             in.status.audio = "output";
             in.status.width = 1168;
             in.status.height = 1080;
             in.status.frames = 30 * 83;
             dashboard(win)->recordCard()->demo(in);
         }},
        {pibridge::app::MainWindow::Dashboard, false, QStringLiteral("09i-dashboard-recording-saved"), false,
         [&](pibridge::app::MainWindow& win) {
             pibridge::app::RecordInputs in;
             in.directRunning = true;
             in.status.state = "idle";
             in.status.file = "/fake/home/Videos/pimax/pimax-20261019-201500.mp4";
             in.status.fps = 30;
             in.status.frames = 30 * 95;
             dashboard(win)->recordCard()->demo(in);
         }},
        {pibridge::app::MainWindow::Tracking, true, QStringLiteral("09f-tracking-steamvr-some-off"), false,
         [&](pibridge::app::MainWindow& win) {
             setConf({{"io_slam", "off"}, {"io_cam_ae", "off"}, {"io_ctrl_angle", "45"}});
             reshow(win, pibridge::app::MainWindow::Tracking);
         }},
        {pibridge::app::MainWindow::Room, false, QStringLiteral("08-room"), false,
         [&](pibridge::app::MainWindow& win) {
             setConf({{"tracking_mode", "lighthouse"}});
             reshow(win, pibridge::app::MainWindow::Room);
         }},
        {pibridge::app::MainWindow::Room, true, QStringLiteral("08b-room-steamvr-running")},
        {pibridge::app::MainWindow::Logs, true, QStringLiteral("04-logs")},
        {pibridge::app::MainWindow::About, true, QStringLiteral("05-about")},
        {pibridge::app::MainWindow::Display, true, QStringLiteral("11-display")},
        {pibridge::app::MainWindow::Recording, true, QStringLiteral("12-recording")},
    };
    int failed = 0;
    for (const auto& s : shots) {
        source.setFakeState(pibridge::app::fakeState(s.active));
        w.showPage(s.page);
        if (s.prepare) {
            s.prepare(w);
        }
        if (auto* sa = w.page(s.page)->findChild<QScrollArea*>()) {
            QCoreApplication::processEvents();
            sa->verticalScrollBar()->setValue(s.scrolledDown ? sa->verticalScrollBar()->maximum() : 0);
        }
        for (int i = 0; i < 5; i++) {
            QCoreApplication::processEvents();
        }
        const QString file = shotsDir + QLatin1Char('/') + s.name + QStringLiteral(".png");
        if (!w.grab().save(file)) {
            std::fprintf(stderr, "could not save %s\n", qPrintable(file));
            failed++;
        } else {
            std::printf("%s\n", qPrintable(file));
        }
    }
    // the help window (pages/help_window.h) on the user guide
    {
        pibridge::app::HelpWindow help(&w);
        help.open(QStringLiteral("USER-GUIDE.md"));
        for (int i = 0; i < 5; i++) {
            QCoreApplication::processEvents();
        }
        const QString file = shotsDir + QStringLiteral("/20-help.png");
        if (!help.grab().save(file)) {
            std::fprintf(stderr, "could not save %s\n", qPrintable(file));
            failed++;
        }
    }
    if (!fakeRunner.calls.empty()) { // the screenshot mode must never run anything
        std::fprintf(stderr, "screenshot mode issued %zu commands\n", fakeRunner.calls.size());
        failed++;
    }
    return failed == 0 ? 0 : 1;
}
