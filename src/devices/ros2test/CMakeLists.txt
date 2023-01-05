# Copyright (C) 2006-2023 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(ros2test
  CATEGORY device
  TYPE Ros2Test
  INCLUDE Ros2Test.h
  INTERNAL ON
)

yarp_add_plugin(yarp_ros2test)

target_sources(yarp_ros2test
  PRIVATE
    Ros2Test.cpp
    Ros2Test.h
)

target_link_libraries(yarp_ros2test
  PRIVATE
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
)

yarp_install(
  TARGETS yarp_ros2test
  EXPORT yarp-device-ros2test
  COMPONENT yarp-device-ros2test
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)

set_property(TARGET yarp_ros2test PROPERTY FOLDER "Plugins/Device")
