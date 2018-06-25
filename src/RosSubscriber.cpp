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
 * @file RosSubscriber.cpp
 * @brief QML wrapper source for RosSubscriber
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#include "RosSubscriber.h"

#ifdef Q_OS_ANDROID
#include <android/log.h>
#else
#include <cstdio>
#endif
#include <stdarg.h>

#include <QNetworkInterface>
#include <QImage>

#include <boost/function.hpp> 
#include <boost/bind.hpp> 

static void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
#ifdef Q_OS_ANDROID
    __android_log_vprint(ANDROID_LOG_INFO, "RosSubscriber", msg, args);
#else
    vprintf(msg, args);
#endif
    va_end(args);
}

static QString getDeviceIpAddress() {
    QList<QHostAddress> list = QNetworkInterface::allAddresses();

    for(int i = 0; i < list.count(); ++i) {
      if(!list[i].isLoopback()) {
          if (list[i].protocol() == QAbstractSocket::IPv4Protocol)
            return list[i].toString();
      }
    }

    return "";
}

RosSubscriber::RosSubscriber(QQuickItem* parent)
: QQuickItem(parent) {
    status = "Idle";
    masterIp = "192.168.1.100";
}

RosSubscriber::~RosSubscriber() {
    stopNode();
}

void RosSubscriber::startNode() {
    if (status == "Running")
        return;

    QString nodeIp = getDeviceIpAddress();
    QString sanitizedNodeIp = QString(nodeIp).replace('.', '_');
    QByteArray tmp = nodeIp.toUtf8();
    log("Node IP: %s", tmp.data());

    int argc = 3;
    QByteArray master = QString("__master:=http://" + masterIp + ":11311").toUtf8();
    QByteArray ip = QString("__ip:=" + nodeIp).toUtf8();
    char *argv[argc] = { "cellulo_qml_plugin", master.data(), ip.data() };

    log("Initializing ROS");
    for (int i = 0; i < argc; ++i) {
        log("Argument %i: %s", i, argv[i]);
    }

    QString nodeName("cellulo_" + sanitizedNodeIp);
    ros::init(argc, &argv[0], nodeName.toStdString());

    if (ros::master::check()) {
        log("ROS master found");
    } else {
        log("No ROS master");
    }

    log(ros::master::getURI().c_str());

    nodeHandle.reset(new ros::NodeHandle());

    status = "Running";
    emit RosSubscriber::statusChanged();

    log("Node started");
}

void RosSubscriber::stopNode() {
    if (status == "Idle")
        return;

    subscribers.clear();
    delete nodeHandle.release();
    ros::shutdown();
    status = "Idle";

    emit statusChanged();
}

void RosSubscriber::imageCallback(const std::string &topic, const sensor_msgs::CompressedImageConstPtr& msg) {
    log("Received new image on topic %s", topic.c_str());
    QImage image;
    image.loadFromData(msg->data.data(), msg->data.size(), msg->format.c_str());
    log("Image size: %d x %d", image.width(), image.height());
    receivedMessages[QString(topic.c_str())] = QVariant(image);

    emit receivedMessagesChanged();
}

void RosSubscriber::subscribe(const QString &topic, int queueSize, MessageType type) {
    std::string _topic = topic.toStdString();
    auto it = subscribers.find(_topic);
    if (it != subscribers.end()) {
        log("Already subscribing to topic %s!", _topic.c_str());
        return;
    }

    if (type == COMPRESSED_IMAGE) {
        boost::function<void(const sensor_msgs::CompressedImageConstPtr&)> f(boost::bind(&RosSubscriber::imageCallback, this, _topic, _1));
        subscribers.emplace(
            _topic,
            std::unique_ptr<ros::Subscriber>(new ros::Subscriber(nodeHandle->subscribe(_topic, queueSize, f)))
        );
    }
    else {
        log("Unsupported message type!");
    }
}

void RosSubscriber::unsubscribe(const QString &topic) {
    std::string _topic = topic.toStdString();
    auto it = subscribers.find(_topic);
    if (it != subscribers.end()) {
        log("Unsibscribing from topic %s!", _topic.c_str());
        subscribers.erase(_topic);
    }
}
