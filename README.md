![YARP logo](doc/images/yarp-robot-24.png?raw=true "yarp-devices-ros2")
Yarp devices for ROS2
=====================

This repository contains the YARP devices and utilities for ROS2.

Documentation
-------------

Documentation of the individual devices is provided in the official Yarp documentation page:
[![YARP documentation](https://img.shields.io/badge/Documentation-yarp.it-19c2d8.svg)](https://yarp.it/latest/group__dev__impl.html)


Installation
-------------

### Build with pure CMake commands

~~~
# Configure, compile and install
cmake -S. -Bbuild -DCMAKE_INSTALL_PREFIX=<install_prefix>
cmake --build build
cmake --build build --target install

# Make YARP devices available
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<install_prefix>/share/yarp

# Make ROS msgs available in [ament index](https://github.com/ament/ament_index)
export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:<install_prefix>
~~~


### Build with ROS msgs compiled in separate colcon workspace

~~~bash
# Compile the colcon workspace containing the required messages and services
(cd ros2_interfaces_ws && colcon build)

# Make the workspace available
. ros2_interfaces_ws/install/setup.bash

# Configure and compile
cmake -S. -Bbuild -DYARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs:BOOL=ON -DYARP_ROS2_USE_SYSTEM_yarp_control_msgs:BOOL=ON -DCMAKE_INSTALL_PREFIX=<install_prefix>
cmake --build build
cmake --build build --target install

# Make YARP devices available
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<install_prefix>/share/yarp
~~~

CI Status
---------

[![Build Status](https://github.com/robotology/yarp-devices-ros2/workflows/CI%20Workflow/badge.svg)](https://github.com/robotology/yarp-devices-ros2/actions?query=workflow%3A%22CI+Workflow%22)

License
---------

[![License](https://img.shields.io/badge/license-BSD--3--Clause%20%2B%20others-19c2d8.svg)](https://github.com/robotology/yarp-devices-ros2/blob/master/LICENSES)

Maintainers
--------------
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/randaz81.png" width="40">](https://github.com/randaz81) | [@randaz81](https://github.com/randaz81) |
| [<img src="https://github.com/elandini84.png" width="40">](https://github.com/elandini84) | [@elandini84](https://github.com/elandini84) |
