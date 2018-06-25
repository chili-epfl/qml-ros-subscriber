/*
 * Copyright (C) 2018 EPFL
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see http://www.gnu.org/licenses/.
 */

/**
 * @file RosSubscriber.h
 * @brief Object that exposes the QMLRos plugin components as QML objects
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#ifndef QMLROSSUBSCRIBERPLUGIN_H
#define QMLROSSUBSCRIBERPLUGIN_H

#include <QQmlExtensionPlugin>
#include <qqml.h>

class RosSubscriberPlugin : public QQmlExtensionPlugin {
    /* *INDENT-OFF* */
    Q_OBJECT
    /* *INDENT-ON* */
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QQmlExtensionInterface")

public:
    void registerTypes(const char* uri);
};

#endif /* QMLROSSUBSCRIBERPLUGIN_H */
