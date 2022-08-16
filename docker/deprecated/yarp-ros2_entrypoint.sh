#!/bin/bash
set -e

USERNAME=@USERNAME@

echo "ROS_DISTRO = $ROS_DISTRO"

if [[ -n "$ROS_DISTRO" ]]; then

  # setup ros2 environment
  if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "Loading /opt/ros/$ROS_DISTRO/setup.bash"
    source "/opt/ros/$ROS_DISTRO/setup.bash"
  else
    echo "WARNING: File /opt/ros/$ROS_DISTRO/setup.bash not found"
  fi

  echo "ROS_VERSION = $ROS_VERSION"

  # setup cer-sim environment
  if [[ $ROS_VERSION = 2 ]]; then
    if [ -f "/home/${USERNAME}/cer-sim/colcon_ws/install/setup.bash" ]; then
      echo "Loading /home/${USERNAME}/cer-sim/colcon_ws/install/setup.bash"
      source "/home/${USERNAME}/cer-sim/colcon_ws/install/setup.bash"
    else
      echo "WARNING: File /home/${USERNAME}/cer-sim/colcon_ws/install/setup.bash"
    fi
  elif [[ $ROS_VERSION = 1 ]]; then
    if [ -f "/home/${USERNAME}/cer-sim/catkin_ws/devel/setup.bash" ]; then
      echo "Loading /home/${USERNAME}/cer-sim/catkin_ws/devel/setup.bash"
      source "/home/${USERNAME}/cer-sim/catkin_ws/devel/setup.bash"
    else
      echo "WARNING: File /home/${USERNAME}/cer-sim/colcon_ws/install/setup.bash"
    fi
  fi

  # setup yarp-ros2 environment
  if [[ $ROS_VERSION = 2 ]]; then
    if [ -f "/home/${USERNAME}/yarp-ros2/ros2_interfaces_ws/install/setup.bash" ]; then
      echo "Loading /home/${USERNAME}/yarp-ros2/ros2_interfaces_ws/install/setup.bash"
      source "/home/${USERNAME}/yarp-ros2/ros2_interfaces_ws/install/setup.bash"
    else
      echo "WARNING: File /home/${USERNAME}/yarp-ros2/ros2_interfaces_ws/install/setup.bash"
    fi
  fi

fi

exec "$@"
