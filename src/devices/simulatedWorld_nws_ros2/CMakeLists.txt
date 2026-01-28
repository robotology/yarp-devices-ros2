# SPDX-FileCopyrightText: 2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

if(YARP_ROS2_USE_SYSTEM_simulated_world_nws_ros2_msgs)
  find_package(simulated_world_nws_ros2_msgs REQUIRED)
endif()

yarp_prepare_plugin(simulatedWorld_nws_ros2
  CATEGORY device
  TYPE SimulatedWorld_nws_ros2
  INCLUDE SimulatedWorld_nws_ros2.h
  EXTRA_CONFIG WRAPPER=simulatedWorld_nws_ros2
  INTERNAL ON
)

if(NOT SKIP_simulatedWorld_nws_ros2)
  yarp_add_plugin(yarp_simulatedWorld_nws_ros2)
  generateDeviceParamsParser(SimulatedWorld_nws_ros2 simulatedWorld_nws_ros2)

  target_sources(yarp_simulatedWorld_nws_ros2
    PRIVATE
      SimulatedWorld_nws_ros2.cpp
      SimulatedWorld_nws_ros2.h
      SimulatedWorld_nws_ros2_ParamsParser.h
      SimulatedWorld_nws_ros2_ParamsParser.cpp
  )

  target_sources(yarp_simulatedWorld_nws_ros2 PRIVATE $<TARGET_OBJECTS:Ros2Utils>)
  target_include_directories(yarp_simulatedWorld_nws_ros2 PRIVATE $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)

  target_link_libraries(yarp_simulatedWorld_nws_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      simulated_world_nws_ros2_msgs::simulated_world_nws_ros2_msgs__rosidl_typesupport_cpp
      Ros2Utils
  )

  yarp_install(
    TARGETS yarp_simulatedWorld_nws_ros2
    EXPORT yarp-device-simulatedWorld_nws_ros2
    COMPONENT yarp-device-simulatedWorld_nws_ros2
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set_property(TARGET yarp_simulatedWorld_nws_ros2 PROPERTY FOLDER "Plugins/Device")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()
endif()
