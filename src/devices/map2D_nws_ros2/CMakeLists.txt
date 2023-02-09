# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(nav_msgs REQUIRED)
find_package(test_msgs REQUIRED)
if(YARP_ROS2_USE_SYSTEM_map2d_nws_ros2_msgs)
  find_package(map2d_nws_ros2_msgs REQUIRED)
endif()
find_package(visualization_msgs REQUIRED)

yarp_prepare_plugin(map2D_nws_ros2
  CATEGORY device
  TYPE Map2D_nws_ros2
  INCLUDE Map2D_nws_ros2.h
  EXTRA_CONFIG WRAPPER=map2D_nws_ros2
  INTERNAL ON
)

if(NOT SKIP_map2D_nws_ros2)
  yarp_add_plugin(yarp_map2D_nws_ros2)

  target_sources(yarp_map2D_nws_ros2
    PRIVATE
      Map2D_nws_ros2.cpp
      Map2D_nws_ros2.h
  )

  target_sources(yarp_localization2D_nws_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)

  target_include_directories(yarp_localization2D_nws_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)

  target_link_libraries(yarp_map2D_nws_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      std_msgs::std_msgs__rosidl_typesupport_cpp
      nav_msgs::nav_msgs__rosidl_typesupport_cpp
      test_msgs::test_msgs__rosidl_typesupport_cpp
      visualization_msgs::visualization_msgs__rosidl_typesupport_cpp
      map2d_nws_ros2_msgs::map2d_nws_ros2_msgs__rosidl_typesupport_cpp
      Ros2Utils
  )

  yarp_install(
    TARGETS yarp_map2D_nws_ros2
    EXPORT yarp-device-map2D_nws_ros2
    COMPONENT yarp-device-map2D_nws_ros2
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set_property(TARGET yarp_map2D_nws_ros2 PROPERTY FOLDER "Plugins/Device")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()
endif()
