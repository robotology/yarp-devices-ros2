#!/bin/bash
set -e

if [[ -n "$ROS_DISTRO" ]]; then

  # setup ros2 environment
  source "/opt/ros/$ROS_DISTRO/setup.bash"

  # setup cer-sim environment
  if [[ $ROS_VERSION = 2 ]]; then
    source "/home/user1/cer-sim/colcon_ws/install/setup.bash"
  elif [[ $ROS_VERSION = 1 ]]; then
    source "/home/user1/cer-sim/catkin_ws/devel/setup.bash"
  fi

  # setup yarp-ros2 environment
  if [[ $ROS_VERSION = 2 ]]; then
    source "/home/user1/yarp-ros2/ros2_interfaces_ws/install/setup.bash"
  fi

fi

exec "$@"
