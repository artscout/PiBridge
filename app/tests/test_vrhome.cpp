// SPDX-License-Identifier: MIT
// The VR home in the app without a headset: the window picture's size and the pointer's way back into the window
// (core/vr_panel.h), Poly Haven's answer, the conversion of a downloaded picture into what pibridge-home reads and the
// thumbnail (core/vr_backgrounds.h), and the shared file between the app and pibridge-home (common/pibridge_vrpanel.h).
#include "pibridge_test.h"

#include "backgrounds.h"
#include "core/vr_backgrounds.h"
#include "core/vr_panel.h"
#include "pibridge_vrpanel.h"

#include <QCoreApplication>
#include <QFile>
#include <QImage>
#include <QTemporaryDir>

using namespace pibridge::app;

TEST(picture_size_keeps_the_aspect_and_the_limit) {
    CHECK(pictureSize({1500, 920}) == QSize(1500, 920));
    CHECK(pictureSize({4096, 2000}) == QSize(2048, 1000));
    CHECK(pictureSize({}) == QSize());
}

TEST(pointer_maps_back_into_the_window) {
    // the picture is the window at half size: picture pixel 100 is window pixel ~201
    const QPoint p = pointerToWindow(100, 50, {1024, 500}, {2048, 1000});
    CHECK(p.x() == 201 && p.y() == 101);
    const QPoint edge = pointerToWindow(5000, 5000, {1024, 500}, {2048, 1000});
    CHECK(edge.x() == 2047 && edge.y() == 999);
}

TEST(poly_haven_answer) {
    const QByteArray j = R"({"hdri":{},"tonemapped":{"url":"https://dl.polyhaven.org/x/sunflowers.jpg","size":123}})";
    CHECK(BackgroundLibrary::tonemappedUrl(j) == QStringLiteral("https://dl.polyhaven.org/x/sunflowers.jpg"));
    CHECK(BackgroundLibrary::tonemappedUrl("not json").isEmpty());
    CHECK(BackgroundLibrary::tonemappedUrl(R"({"hdri":{}})").isEmpty());
}

TEST(a_downloaded_picture_becomes_the_homes_file_and_a_thumbnail) {
    QTemporaryDir d;
    const QString jpg = d.path() + QStringLiteral("/in.jpg");
    QImage pano(1024, 512, QImage::Format_RGB32);
    pano.fill(QColor(200, 120, 40));
    CHECK(pano.save(jpg, "JPG"));
    const QString ppm = d.path() + QStringLiteral("/x.ppm"), th = d.path() + QStringLiteral("/thumbs/x.png");
    CHECK(BackgroundLibrary::convert(jpg, ppm, th, 512).isEmpty());
    QFile f(ppm);
    CHECK(f.open(QIODevice::ReadOnly));
    CHECK(f.readLine() == "P6\n" && f.readLine() == "512 256\n" && f.readLine() == "255\n");
    CHECK(f.size() == 512 * 256 * 3 + 15);
    CHECK(QImage(th).size() == QSize(320, 160));
    // a flat photo is refused
    QImage flat(800, 600, QImage::Format_RGB32);
    flat.fill(Qt::gray);
    CHECK(flat.save(jpg, "JPG"));
    CHECK(BackgroundLibrary::convert(jpg, ppm, th).contains(QStringLiteral("2:1")));
    CHECK(!BackgroundLibrary::convert(d.path() + QStringLiteral("/none.jpg"), ppm, th).isEmpty());
}

TEST(the_fake_library_downloads_nothing) {
    BackgroundLibrary lib(QStringLiteral("/nonexistent"), true);
    CHECK(!lib.download(QStringLiteral("birchwood")));
    CHECK(lib.downloaded(QStringLiteral("sunflowers")) && !lib.thumb(QStringLiteral("sunflowers")).isNull());
    CHECK(!lib.downloaded(QStringLiteral("birchwood")));
}

TEST(the_catalogue_names_free_pictures) {
    for (const auto& p : pibridge::home::bg::catalogue()) {
        CHECK(p.licence == "CC0" && !p.author.empty() && !p.title.empty());
        CHECK(pibridge::home::bg::directJpgUrl(p.id).starts_with("https://dl.polyhaven.org/"));
    }
}

TEST(the_shared_file_round_trip) {
    QTemporaryDir d;
    pibridge::vrpanel::Link app, home;
    const std::string path = (d.path() + QStringLiteral("/vrpanel.shm")).toStdString();
    CHECK(app.open(path) && home.open(path));
    QImage img(300, 200, QImage::Format_ARGB32);
    img.fill(QColor(10, 20, 30));
    CHECK(app.writeFrame(img.constBits(), 300, 200, static_cast<std::size_t>(img.bytesPerLine())));
    std::vector<std::uint8_t> got;
    std::uint32_t w = 0, h = 0;
    const auto seq = home.readFrame(0, got, w, h);
    CHECK(seq && w == 300 && h == 200);
    CHECK(got.size() == 300u * 200u * 4u && got[0] == 30 && got[1] == 20 && got[2] == 10); // B G R in memory
    home.header()->pointer.store(pibridge::vrpanel::packPointer({true, 12, 34, 1}));
    const auto p = pibridge::vrpanel::unpackPointer(app.header()->pointer.load());
    CHECK(p.on && p.x == 12 && p.y == 34 && p.buttons == 1);
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv); // the image plugins (JPEG)
    return pibridge::test::run_all(argc, argv);
}
