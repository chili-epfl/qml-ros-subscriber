qml-ros-subscriber
===================

QML plugin for subscribing to messages from a ROS master. For now only image messages are supported, but it can easily be extended.
Tested with Qt 5.10.0 on the following:

  - Android 6.0.1 (arm-v7) built with SDK API 18 and NDK r10e on ArchLinux host

Prerequisites
-------------
 - ROS libraries and their headers. If building for ARM, ROS libraries cross-compiled for arm-v7 and their headers. Follow the instructions [here](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) and adapt the `.pro` file using the `lib` and `include` directories found in `roscpp_android/output/target`.

build
-----

```
    $ mkdir build && cd build
    $ qmake ..
    $ make install
```

Make sure to use the correct `qmake` for your architecture.
