# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(ros2PubTest
  CATEGORY device
  TYPE Ros2PubTest
  INCLUDE Ros2PubTest.h
  INTERNAL ON
)

yarp_add_plugin(yarp_ros2PubTest)

target_sources(yarp_ros2PubTest
  PRIVATE
    Ros2Test.cpp
    Ros2Test.h
)

target_link_libraries(yarp_ros2PubTest
  PRIVATE
    YARP::YARP_os
    YARP::YARP_dev
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
)

yarp_install(
  TARGETS yarp_ros2PubTest
  EXPORT yarp-device-ros2PubTest
  COMPONENT yarp-device-ros2PubTest
  LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
  ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
  YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
)

set_property(TARGET yarp_ros2PubTest PROPERTY FOLDER "Plugins/Device")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()
