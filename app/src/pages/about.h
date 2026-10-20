// SPDX-License-Identifier: MIT
// About: version, licence, origin, and the files the app reads and writes.
#pragma once

#include <QWidget>

namespace pibridge::app {

class AboutPage : public QWidget {
    Q_OBJECT
public:
    explicit AboutPage(QWidget* parent = nullptr);
};

} // namespace pibridge::app
