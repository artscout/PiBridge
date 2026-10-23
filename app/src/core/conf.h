// SPDX-License-Identifier: MIT
// The app's writer of the shared settings file ~/.config/pibridge/pibridge.conf (key=value lines, # comments).
// The daemons own the parser (pibridge::hmd::Settings); this side only changes the lines of the keys the user switched and
// keeps every other line, comment and unknown key exactly as it was, so a hand-edited file survives the app.
#pragma once

#include <QString>
#include <optional>
#include <string>
#include <string_view>

namespace pibridge::app {

// The value of the last live (not commented) `key=` line, trimmed; nullopt when there is none.
std::optional<std::string> confGet(std::string_view text, std::string_view key);
// The text with every live `key=` line set to `value` (a trailing " # comment" is kept), or `key=value` appended
// when the key has no live line. Commented lines ("#key=...") are left alone. Ends with a newline.
std::string confSet(std::string_view text, std::string_view key, std::string_view value);
// A number with a point in every locale and no trailing zeros ("90", "0.5").
std::string confNumber(double v);
// "on"/"1"/"true"/"yes" -> true, "off"/"0"/"false"/"no" -> false, else `def`.
bool confBool(const std::optional<std::string>& v, bool def);

// The settings file on disk, with the defaults of docs/CONVENTIONS.md.
class Conf {
public:
    // `fallback`: read instead while `path` does not exist (the file of PiBridge's earlier name); a set() then writes
    // its text with the change to `path`, so the settings move over on the first change. It is never written.
    explicit Conf(QString path, QString fallback = {});
    const QString& path() const { return path_; }
    // Re-read the file (missing: empty, every key at its default).
    void reload();
    std::string text() const { return text_; }
    QString value(std::string_view key, const QString& def) const;
    bool flag(std::string_view key, bool def) const;
    int refreshHz() const;
    bool eyeTracking() const { return flag("eye_tracking", true); }
    bool audioToHeadset() const { return flag("audio_to_headset", true); }
    bool micToHeadset() const { return flag("mic_to_headset", true); }
    // Set one key and write the file atomically (temp file + rename); false when it could not be written.
    bool set(std::string_view key, std::string_view value);

private:
    QString path_;
    QString fallback_;
    std::string text_;
};

} // namespace pibridge::app
