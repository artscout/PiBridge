// SPDX-License-Identifier: MIT
// The custom widgets of the look: round toggle, outlined status chip, card with title and chip, segmented buttons,
// long tick slider with "-" / "+" and a value box, the yellow "needs a game restart" dot, and small helpers.
#pragma once

#include <QAbstractButton>
#include <QFrame>
#include <QLabel>
#include <QSlider>
#include <QStringList>
#include <QWidget>
#include <optional>
#include <vector>

#include "../core/status.h"

class QButtonGroup;
class QHBoxLayout;
class QPushButton;
class QSpinBox;
class QVBoxLayout;

namespace pibridge::ui {

// An uppercase, letter-spaced label.
QLabel* capsLabel(const QString& text, int px = 13, QWidget* parent = nullptr);
// A plain dim body text (word-wrapped).
QLabel* dimLabel(const QString& text, QWidget* parent = nullptr);
// An outlined toolbar button ("LAUNCH STEAMVR").
QPushButton* toolButton(const QString& text, QWidget* parent = nullptr);
// A filled indigo button ("IMPORT").
QPushButton* accentButton(const QString& text, QWidget* parent = nullptr);
// A text button in the accent colour ("CALIBRATE").
QPushButton* linkButton(const QString& text, QWidget* parent = nullptr);
// A 1 px horizontal line.
QFrame* rule(QWidget* parent = nullptr);

class Toggle : public QAbstractButton {
    Q_OBJECT
public:
    explicit Toggle(QWidget* parent = nullptr);
    QSize sizeHint() const override { return {40, 22}; }
    // Set without emitting toggled() (for showing the state that is on disk).
    void setCheckedQuiet(bool on);

protected:
    void paintEvent(QPaintEvent*) override;
};

class Chip : public QWidget {
    Q_OBJECT
public:
    explicit Chip(QWidget* parent = nullptr);
    void set(const QString& text, pibridge::app::ChipKind kind);
    void set(const pibridge::app::ChipText& c) { set(c.text, c.kind); }
    const QString& text() const { return text_; }
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override { return sizeHint(); }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QString text_;
    pibridge::app::ChipKind kind_ = pibridge::app::ChipKind::Off;
};

// The yellow dot that marks a setting the game must be restarted for.
class RestartDot : public QWidget {
public:
    explicit RestartDot(QWidget* parent = nullptr);
    QSize sizeHint() const override { return {10, 10}; }

protected:
    void paintEvent(QPaintEvent*) override;
};

// A small round "?" badge that carries a hint. Unlike a bare Qt tooltip it is visible and discoverable, like
// omarchyvr's hover-hints: hovering shows the text and clicking it pops the same text (so it can be read off a
// screen or in the headset, not only on hover). A muted grey circle with an accent "?", clearly distinct from the
// yellow "needs a restart" dot.
class HelpBadge : public QWidget {
public:
    explicit HelpBadge(const QString& text, QWidget* parent = nullptr);
    QSize sizeHint() const override { return {16, 16}; }

protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;

private:
    QString text_;
};

// A HelpBadge returned as a plain QWidget*, for the shared row helpers and page layouts.
QWidget* helpBadge(const QString& text);

class Card : public QFrame {
    Q_OBJECT
public:
    explicit Card(const QString& title, QWidget* parent = nullptr);
    QVBoxLayout* body() const { return body_; }
    QHBoxLayout* header() const { return header_; }
    Chip* chip() const { return chip_; }
    QLabel* titleLabel() const { return title_; }

private:
    QLabel* title_;
    Chip* chip_;
    QHBoxLayout* header_;
    QVBoxLayout* body_;
};

class Segmented : public QWidget {
    Q_OBJECT
public:
    explicit Segmented(const QStringList& items, QWidget* parent = nullptr);
    // -1: none checked.
    void setCurrent(int index);
    int current() const;
    void setItemEnabled(int index, bool on);

signals:
    void activated(int index); // a click by the user only

private:
    QButtonGroup* group_;
    std::vector<QPushButton*> buttons_;
};

// A slider drawn as a row of thin ticks, filled in the accent colour up to the value.
class TickSlider : public QSlider {
public:
    explicit TickSlider(QWidget* parent = nullptr);
    QSize sizeHint() const override { return {300, 18}; }

protected:
    void paintEvent(QPaintEvent*) override;
};

// LABEL (with an optional restart dot) above:  - |||||||||:::::::: +  [ 42 ] %
class SliderRow : public QWidget {
    Q_OBJECT
public:
    SliderRow(const QString& label, int min, int max, const QString& unit, bool restartDot, const QString& tip,
              QWidget* parent = nullptr);
    int value() const;
    void setValueQuiet(int v);

signals:
    void valueChanged(int v);

private:
    TickSlider* slider_;
    QSpinBox* box_;
    bool quiet_ = false;
};

// Label left, widget right, in one row (the rows of Quick settings). When `tip` is non-empty a "?" help badge is
// shown next to the label (a visible, clickable hint, like omarchyvr's hover-hints).
QWidget* settingRow(const QString& label, QWidget* right, QWidget* extra = nullptr, bool restartDot = false,
                    const QString& tip = {});

// Our own simple line drawing of a headset.
class HeadsetArt : public QWidget {
public:
    explicit HeadsetArt(QWidget* parent = nullptr);
    void setLit(bool lit);
    QSize sizeHint() const override { return {300, 200}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    bool lit_ = false;
};

// A small battery: outline, fill to the level (amber at 20 % and below, red at 10 %), a bolt while charging.
class Battery : public QWidget {
public:
    explicit Battery(QWidget* parent = nullptr);
    // percent < 0: nothing drawn.
    void set(int percent, bool charging);
    QSize sizeHint() const override { return {34, 16}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    int percent_ = -1;
    bool charging_ = false;
};

// A small view of the gaze: a circle of +-45 degrees with the gaze dot (grey "no gaze" when there is none).
class GazeDot : public QWidget {
public:
    explicit GazeDot(QWidget* parent = nullptr);
    void setGaze(std::optional<std::pair<float, float>> tangent);
    QSize sizeHint() const override { return {120, 120}; }

protected:
    void paintEvent(QPaintEvent*) override;

private:
    std::optional<std::pair<float, float>> g_;
};

} // namespace pibridge::ui
