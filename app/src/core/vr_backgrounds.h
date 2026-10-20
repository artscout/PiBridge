// SPDX-License-Identifier: MIT
// The VR home's backgrounds in the app (Home page; docs/home.md "Backgrounds"): which pictures of the list
// (tools/home/src/backgrounds.h, Poly Haven CC0) are downloaded, their thumbnails, and the download on first use into
// ~/.local/share/pibridge/backgrounds. A download is three steps, each a leaf that only reports in plain words when it
// fails: Poly Haven's API names the tonemapped 8K JPG (curl), the JPG is fetched (curl, resumable), then it is turned
// into what pibridge-home reads (a 4096 x 2048 PPM) and a 320 x 160 PNG thumbnail on a worker thread (Qt's JPEG reader).
// Nothing runs in the fake mode.
#pragma once

#include <QImage>
#include <QObject>
#include <QString>
#include <memory>

class QProcess;

namespace pibridge::app {

class BackgroundLibrary : public QObject {
    Q_OBJECT
public:
    BackgroundLibrary(QString dir, bool fake, QObject* parent = nullptr);
    ~BackgroundLibrary() override;
    const QString& dir() const { return dir_; }
    bool downloaded(const QString& id) const;
    QString thumbPath(const QString& id) const;
    /// The thumbnail (a generated one in the fake mode), null when there is none.
    QImage thumb(const QString& id) const;
    bool busy() const { return !busyId_.isEmpty(); }
    const QString& busyId() const { return busyId_; }
    /// Start the download of `id` (one at a time); false when busy, unknown or in the fake mode.
    bool download(const QString& id);

    /// The picture file -> the PPM pibridge-home reads (width at most `maxW`, 2:1) and the thumbnail; plain words on
    /// failure. Runs on any thread (no widgets).
    static QString convert(const QString& jpg, const QString& ppm, const QString& thumb, int maxW = 4096);
    /// tonemapped.url of Poly Haven's /files/<id> answer; empty when it has none.
    static QString tonemappedUrl(const QByteArray& apiJson);

signals:
    void progress(const QString& id, const QString& words);
    void finished(const QString& id, bool ok, const QString& words);

private:
    void fetch(const QString& id, const QString& url);
    void done(const QString& id, bool ok, const QString& words);

    QString dir_;
    bool fake_;
    QString busyId_;
    QProcess* proc_ = nullptr;
};

} // namespace pibridge::app
