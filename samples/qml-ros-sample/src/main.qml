import QtQuick 2.2
import QtQuick.Window 2.1
import QtQuick.Controls 1.2

import ch.epfl.chili.ros.subscriber 1.0

ApplicationWindow {
    id: window
    visible: true

    Timer {
        running: true
        interval: 100
        repeat: true
        onTriggered: {
            if (rosSubscriber.status == "Running") {
                rosSubscriber.spinOnce()
            }
        }
    }

    RosSubscriber{
        id: rosSubscriber
        masterIp: masterIpField.text

        onStatusChanged: {
            status.text = rosSubscriber.status
            if (rosSubscriber.status == "Idle")
                status.color = "gray";
            else
                status.color = "blue";
        }

        onReceivedMessagesChanged: {
            imageItem.setImage(rosSubscriber.receivedMessages["/usb_cam/image_raw/compressed"])
        }
    }

    Row{
        GroupBox{
            title: "RosSubscriber"

            Column{
                Label{
                    text: "ROS master IP address"
                }
                TextField{
                    id: masterIpField
                    text: "192.168.1.100"
                }

                Button{
                    text: "Start"
                    onClicked: {
                        rosSubscriber.startNode();
                        rosSubscriber.subscribe("/usb_cam/image_raw/compressed", 10, RosSubscriber.CompressedImage)
                    }
                }

                Button{
                    text: "Stop"
                    onClicked: rosSubscriber.stopNode();
                }

                Label{
                    text: "Status:"
                }

                Text{
                    id: status
                    text: rosSubscriber.status
                    color: "gray"
                }
            }
        }

        GroupBox{
            title: "Image"

            Column{
                ImageItem {
                    id: imageItem
                    width: 1280
                    height: 960
                }
            }
        }
    }
}
