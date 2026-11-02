// SPDX-License-Identifier: MIT
// pibridge-desktop: the desktop in the headset (docs/desktop.md).
//   pibridge-desktop run [--idle-exit S] [--fps N]   the service (the overlays start it; one at a time)
//   pibridge-desktop allow [--timeout S]             ask KDE once, on the desktop, and keep its permission
//   pibridge-desktop forget                          drop the permission (KDE asks again next time)
//   pibridge-desktop status                          the permission and what the service does
#include "pibridge_desktop.h"
#include "portal.h"
#include "service.h"

#include <QCoreApplication>
#include <QFile>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/file.h>
#include <unistd.h>

using namespace pibridge;
using namespace pibridge::desktopsvc;

namespace {

int usage() {
    std::fprintf(stderr, "usage: pibridge-desktop run [--idle-exit S] [--fps N]\n"
                         "       pibridge-desktop allow [--timeout S]\n"
                         "       pibridge-desktop forget\n"
                         "       pibridge-desktop status\n");
    return 2;
}

const char* stateWords(desktop::State s) {
    switch (s) {
    case desktop::State::Off: return "off";
    case desktop::State::Starting: return "starting";
    case desktop::State::Running: return "running";
    case desktop::State::NeedsPermission: return "needs permission";
    case desktop::State::Failed: return "failed";
    }
    return "?";
}

int intArg(int argc, char** argv, const char* name, int def) {
    for (int i = 2; i + 1 < argc; i++) {
        if (std::strcmp(argv[i], name) == 0) {
            return std::atoi(argv[i + 1]);
        }
    }
    return def;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        return usage();
    }
    const std::string cmd = argv[1];
    if (cmd == "status") {
        const bool allowed = !readToken().isEmpty();
        std::printf("permission: %s\n", allowed ? "allowed" : "not allowed yet (pibridge-desktop allow)");
        desktop::Link link;
        if (link.open()) {
            const bool up = desktop::fresh(link.header()->serviceBeatNs.load(), desktop::monoNs());
            std::printf("service: %s", up ? stateWords(link.state()) : "not running");
            if (up) {
                std::printf(" (%s)", link.text().c_str());
            }
            std::printf("\n");
            if (up) {
                for (const auto& m : link.monitors()) {
                    std::printf("  %s: %ux%u at %d,%d (picture %ux%u)\n", m.name.c_str(), m.logicalW, m.logicalH, m.x, m.y,
                                m.width, m.height);
                }
            }
        }
        return 0;
    }
    if (cmd == "forget") {
        QFile::remove(tokenPath());
        std::printf("The desktop permission is forgotten; KDE asks again at the next `pibridge-desktop allow`.\n");
        return 0;
    }
    QCoreApplication app(argc, argv);
    if (cmd == "allow") {
        // ask on the desktop (or restore silently if allowed already), keep the token, show what is shared
        Portal portal;
        std::printf("A question from KDE about sharing the screens and the pointer appears on the desktop: allow it "
                    "(and choose to remember it).\n");
        std::fflush(stdout);
        PortalResult r = portal.open(readToken(), intArg(argc, argv, "--timeout", 120) * 1000);
        if (!r.restoreToken.isEmpty()) {
            writeToken(r.restoreToken);
        }
        if (r.pipewireFd >= 0) {
            ::close(r.pipewireFd);
        }
        portal.close();
        if (!r.ok) {
            std::fprintf(stderr, "Not allowed: %s\n", qPrintable(r.error));
            return 1;
        }
        if (r.restoreToken.isEmpty()) {
            std::fprintf(stderr, "Allowed for now, but KDE gave no permission to keep: it will ask again.\n");
            return 1;
        }
        std::printf("Allowed. Shared monitors:\n");
        for (std::size_t i = 0; i < r.streams.size(); i++) {
            std::printf("  Monitor %zu: %dx%d at %d,%d\n", i + 1, r.streams[i].w, r.streams[i].h, r.streams[i].x,
                        r.streams[i].y);
        }
        return 0;
    }
    if (cmd != "run") {
        return usage();
    }
    // one service at a time: the overlays start it whenever the desktop is opened
    const std::string lockPath = (status::runtime_dir() / "desktop.lock").string();
    const int lock = ::open(lockPath.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (lock < 0 || ::flock(lock, LOCK_EX | LOCK_NB) != 0) {
        std::fprintf(stderr, "pibridge-desktop: already running\n");
        return 0;
    }
    desktop::Link link;
    if (!link.open()) {
        std::fprintf(stderr, "pibridge-desktop: %s\n", link.error().c_str());
        return 1;
    }
    RunOptions opt;
    opt.idleExitS = intArg(argc, argv, "--idle-exit", opt.idleExitS);
    opt.fps = intArg(argc, argv, "--fps", opt.fps);
    Service svc(link, opt);
    svc.start();
    return QCoreApplication::exec();
}
