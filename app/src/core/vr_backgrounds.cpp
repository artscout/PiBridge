// SPDX-License-Identifier: MIT
#include "vr_backgrounds.h"

#include "backgrounds.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QLinearGradient>
#include <QPointer>
#include <QProcess>
#include <QSaveFile>
#include <thread>

namespace pibridge::app {

namespace {
const char* kAgent = "pibridge-backgrounds/0.1 (personal VR use)";
}

BackgroundLibrary::BackgroundLibrary(QString dir, bool fake, QObject* parent)
    : QObject(parent), dir_(std::move(dir)), fake_(fake) {}

BackgroundLibrary::~BackgroundLibrary() {
    if (proc_ != nullptr) {
        proc_->kill();
        proc_->waitForFinished(1000);
    }
}

bool BackgroundLibrary::downloaded(const QString& id) const {
    if (fake_) {
        return id == QLatin1String("sunflowers") || id == QLatin1String("venice_sunset") || id == QLatin1String("alps_field");
    }
    return home::bg::downloaded(dir_.toStdString(), id.toStdString());
}

QString BackgroundLibrary::thumbPath(const QString& id) const {
    return QString::fromStdString(home::bg::thumbPath(dir_.toStdString(), id.toStdString()).string());
}

QImage BackgroundLibrary::thumb(const QString& id) const {
    if (fake_) {
        if (!downloaded(id)) {
            return {};
        }
        // a made-up landscape per picture for the screenshots
        QImage img(320, 160, QImage::Format_RGB32);
        QPainter p(&img);
        const int hue = static_cast<int>(qHash(id) % 360);
        QLinearGradient g(0, 0, 0, 160);
        g.setColorAt(0, QColor::fromHsv(hue, 120, 200));
        g.setColorAt(0.55, QColor::fromHsv((hue + 30) % 360, 90, 240));
        g.setColorAt(0.56, QColor::fromHsv((hue + 90) % 360, 140, 120));
        g.setColorAt(1, QColor::fromHsv((hue + 90) % 360, 160, 60));
        p.fillRect(img.rect(), g);
        return img;
    }
    QImage img(thumbPath(id));
    return img;
}

QString BackgroundLibrary::tonemappedUrl(const QByteArray& apiJson) {
    const QJsonDocument d = QJsonDocument::fromJson(apiJson);
    return d.object().value(QStringLiteral("tonemapped")).toObject().value(QStringLiteral("url")).toString();
}

QString BackgroundLibrary::convert(const QString& jpg, const QString& ppm, const QString& thumb, int maxW) {
    QImage img(jpg);
    if (img.isNull()) {
        return QStringLiteral("the downloaded file is not a picture");
    }
    if (img.width() < 512 || std::abs(img.width() - 2 * img.height()) > img.width() / 50) {
        return QStringLiteral("the picture is not a 360-degree (2:1) panorama");
    }
    if (img.width() > maxW) {
        img = img.scaled(maxW, maxW / 2, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    }
    img = img.convertToFormat(QImage::Format_RGB888);
    QSaveFile out(ppm);
    if (!out.open(QIODevice::WriteOnly)) {
        return QStringLiteral("cannot write ") + ppm;
    }
    out.write(QStringLiteral("P6\n%1 %2\n255\n").arg(img.width()).arg(img.height()).toLatin1());
    for (int y = 0; y < img.height(); y++) {
        out.write(reinterpret_cast<const char*>(img.constScanLine(y)), static_cast<qint64>(img.width()) * 3);
    }
    if (!out.commit()) {
        return QStringLiteral("cannot write ") + ppm;
    }
    QDir().mkpath(QFileInfo(thumb).absolutePath());
    if (!img.scaled(320, 160, Qt::IgnoreAspectRatio, Qt::SmoothTransformation).save(thumb, "PNG")) {
        return QStringLiteral("cannot write the thumbnail");
    }
    return {};
}

bool BackgroundLibrary::download(const QString& id) {
    if (fake_ || busy() || home::bg::find(id.toStdString()) == nullptr) {
        return false;
    }
    QDir().mkpath(dir_ + QStringLiteral("/.part"));
    busyId_ = id;
    emit progress(id, QStringLiteral("Asking Poly Haven for the picture ..."));
    proc_ = new QProcess(this);
    connect(proc_, &QProcess::finished, this, [this, id](int code, QProcess::ExitStatus st) {
        const QByteArray json = proc_->readAllStandardOutput();
        proc_->deleteLater();
        proc_ = nullptr;
        QString url = code == 0 && st == QProcess::NormalExit ? tonemappedUrl(json) : QString();
        if (url.isEmpty()) {
            url = QString::fromStdString(home::bg::directJpgUrl(id.toStdString())); // the usual place of that file
        }
        fetch(id, url);
    });
    connect(proc_, &QProcess::errorOccurred, this, [this, id](QProcess::ProcessError e) {
        if (e == QProcess::FailedToStart) {
            proc_->deleteLater();
            proc_ = nullptr;
            done(id, false, QStringLiteral("curl is not installed: the pictures cannot be downloaded."));
        }
    });
    proc_->start(QStringLiteral("curl"), {QStringLiteral("-sfL"), QStringLiteral("--max-time"), QStringLiteral("20"),
                                          QStringLiteral("-A"), QString::fromLatin1(kAgent),
                                          QString::fromStdString(home::bg::apiUrl(id.toStdString()))});
    return true;
}

void BackgroundLibrary::fetch(const QString& id, const QString& url) {
    const QString part = dir_ + QStringLiteral("/.part/") + id + QStringLiteral(".jpg");
    emit progress(id, QStringLiteral("Downloading (about 10 to 30 MB) ..."));
    proc_ = new QProcess(this);
    connect(proc_, &QProcess::finished, this, [this, id, part](int code, QProcess::ExitStatus st) {
        proc_->deleteLater();
        proc_ = nullptr;
        if (code != 0 || st != QProcess::NormalExit) {
            done(id, false, QStringLiteral("The download failed (curl code %1). Check the internet connection and try again: "
                                           "it goes on where it stopped.").arg(code));
            return;
        }
        emit progress(id, QStringLiteral("Preparing the picture ..."));
        const QString ppm = QString::fromStdString(home::bg::picturePath(dir_.toStdString(), id.toStdString()).string());
        const QString th = thumbPath(id);
        QPointer<BackgroundLibrary> self(this);
        std::thread([self, id, part, ppm, th] {
            const QString err = convert(part, ppm, th);
            if (err.isEmpty()) {
                QFile::remove(part);
            }
            if (self) {
                QMetaObject::invokeMethod(self.data(), [self, id, err] {
                    if (self) {
                        self->done(id, err.isEmpty(), err.isEmpty() ? QStringLiteral("Ready.") : QStringLiteral("Not usable: ") + err + QLatin1Char('.'));
                    }
                }, Qt::QueuedConnection);
            }
        }).detach();
    });
    proc_->start(QStringLiteral("curl"), {QStringLiteral("-fL"), QStringLiteral("--retry"), QStringLiteral("2"), QStringLiteral("-C"),
                                          QStringLiteral("-"), QStringLiteral("--max-filesize"), QStringLiteral("80000000"),
                                          QStringLiteral("-A"), QString::fromLatin1(kAgent), QStringLiteral("-o"), part, url});
}

void BackgroundLibrary::done(const QString& id, bool ok, const QString& words) {
    busyId_.clear();
    emit finished(id, ok, words);
}

} // namespace pibridge::app
