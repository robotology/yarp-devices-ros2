![YARP logo](doc/images/yarp-robot-24.png?raw=true "yarp-devices-ros2")
Yarp devices for ROS2
=====================

This repository contains the YARP devices and utilities for ROS2.

:construction: This repository is currently work in progress. :construction:
:construction: The software contained is this repository is currently under testing. :construction: APIs may change without any warning. :construction: This code should be not used before its first official release :construction:

Documentation
-------------

Documentation of the individual devices is provided in the official Yarp documentation page:
[![YARP documentation](https://img.shields.io/badge/Documentation-yarp.it-19c2d8.svg)](https://yarp.it/latest/group__dev__impl.html)


Installation
-------------

### Build with ROS msgs compiled in separate colcon workspace

~~~bash
# Compile the colcon workspace containing the required messages and services
(cd ros2_interfaces_ws && colcon build --packages-select map2d_nws_ros2_msgs)

# Make the workspace available
. ros2_interfaces_ws/install/setup.bash

# Configure and compile
cmake -S. -Bbuild
cmake --build build
~~~

### Build with pure CMake commands

~~~
# Configure, compile and install
cmake -S. -Bbuild -DCMAKE_INSTALL_PREFIX=<install_prefix>
cmake --build build
cmake --build build --target install

# Make ROS msgs available in [ament index](https://github.com/ament/ament_index)
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:<install_prefix>
~~~

CI Status
---------

:construction: This repository is currently work in progress. :construction:

[![Build Status](https://github.com/robotology/yarp-devices-ros2/workflows/CI%20Workflow/badge.svg)](https://github.com/robotology/yarp-devices-ros2/actions?query=workflow%3A%22CI+Workflow%22)

License
---------

:construction: This repository is currently work in progress. :construction:

Maintainers
--------------
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/randaz81.png" width="40">](https://github.com/randaz81) | [@randaz81](https://github.com/randaz81) |
| [<img src="https://github.com/elandini84.png" width="40">](https://github.com/elandini84) | [@elandini84](https://github.com/elandini84) |
