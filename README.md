qml-ros
===================

QML plugin for subscribing to messages from a ROS master. Tested with Qt 5.10.0 on
the following:

  - Android 6.0.1 (arm-v7) built with SDK API 18 and NDK r10e on ArchLinux host

Prerequisites
-------------

 - ROS libraries cross-compiled for arm-v7 and their headers. Follow the instructions [here](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) and adapt the `.pro` file using the `lib` and `include` directories found in `roscpp_android/output/target`.

build
-----

```
    $ mkdir build && cd build
    $ qmake ..
    $ make install
```

Make sure to use the `qmake` binary from the arm-v7 Qt installation.
