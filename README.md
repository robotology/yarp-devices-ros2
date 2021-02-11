yarp_ros2
---------

This is a work in progress.

This repository contains the YARP devices and utilities for ROS2.

### Build


~~~bash
# Compile the colcon workspace containing the required messages and services
(cd ros2_interfaces_ws && colcon build --packages-select map2d_nws_ros2_msgs)

# Make the workspace available
. ros2_interfaces_ws/install/setup.bash

# Configure and compile
cmake -S. -Bbuild
cmake --build build
~~~
