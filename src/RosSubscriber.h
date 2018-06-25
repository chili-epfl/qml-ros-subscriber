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
 * @brief QML wrapper header for RosSubscriber
 * @author Florian Zimmermann
 * @date 2018-03-26
 */

#ifndef ROSNODE_H
#define ROSNODE_H

#include <QQuaternion>
#include <QQuickItem>

#include <memory>
#include <string>
#include <unordered_map>

#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

class RosSubscriber : public QQuickItem {
    /* *INDENT-OFF* */
    Q_OBJECT
    /* *INDENT-ON* */

    Q_PROPERTY(QString status READ getStatus NOTIFY statusChanged)
    Q_PROPERTY(QString masterIp READ getMasterIp WRITE setMasterIp NOTIFY masterIpChanged)
    Q_PROPERTY(QVariantMap receivedMessages READ getReceivedMessages NOTIFY receivedMessagesChanged)

public:
    enum MessageType {
        COMPRESSED_IMAGE
    };
    Q_ENUM(MessageType)
    /**
     * @brief Creates a new RosSubscriber with the given QML parent
     *
     * @param parent The QML parent
     */
    RosSubscriber(QQuickItem* parent = 0);

    /**
     * @brief Destroys this RosSubscriber
     */
    ~RosSubscriber();

    /**
     * @brief Gets this ROS node's status
     *
     * @return This ROS node's status
     */
    const QString &getStatus() const { return status; }

    /**
     * @brief Gets the ROS master's IP address
     *
     * @return The ROS master's IP address
     */
    const QString &getMasterIp() const { return masterIp; }

    const QVariantMap &getReceivedMessages() const { return receivedMessages; }

    /**
     * @brief Sets the ROS master's IP address
     *
     * @param The ROS master's IP address
     */
    void setMasterIp(const QString &masterIp) { this->masterIp = masterIp; }

public slots:
    /**
     * @brief Initializes the ROS node
     */
    void startNode();

    /**
     * @brief Kills the ROS node
     */
    void stopNode();

    void spinOnce() { ros::spinOnce(); }

    void subscribe(const QString &topic, int queueSize, MessageType type);
    void unsubscribe(const QString &topic);

signals:
    /**
     * @brief Emitted when this ROS node's status changes
     */
    void statusChanged();

    /**
     * @brief Emitted when the ROS master's IP address changes
     */
    void masterIpChanged();

    void receivedMessagesChanged();

private:
    void imageCallback(const std::string &topic, const sensor_msgs::CompressedImageConstPtr& msg);

    QString status;                   ///< Status of this ROS node
    QString masterIp;                 ///< IP address of ROS master

    std::unique_ptr<ros::NodeHandle> nodeHandle;
    std::unordered_map<std::string, std::unique_ptr<ros::Subscriber>> subscribers;

    QVariantMap receivedMessages;
};

#endif /* ROSNODE_H */
