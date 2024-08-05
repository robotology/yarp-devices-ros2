# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(frameTransformGet_nwc_ros2
  CATEGORY device
  TYPE FrameTransformGet_nwc_ros2
  INCLUDE frameTransformGet_nwc_ros2.h
  INTERNAL ON
)

if(NOT SKIP_frameTransformGet_nwc_ros2)
  yarp_add_plugin(yarp_frameTransformGet_nwc_ros2)
  generateDeviceParamsParser(FrameTransformGet_nwc_ros2 frameTransformGet_nwc_ros2)

  target_sources(yarp_frameTransformGet_nwc_ros2
    PRIVATE
      frameTransformGet_nwc_ros2.cpp
      frameTransformGet_nwc_ros2.h
      FrameTransformGet_nwc_ros2_ParamsParser.h
      FrameTransformGet_nwc_ros2_ParamsParser.cpp
  )

  target_include_directories(yarp_frameTransformGet_nwc_ros2 PRIVATE
                             $<TARGET_PROPERTY:Ros2Utils,INTERFACE_INCLUDE_DIRECTORIES>)

  target_link_libraries(yarp_frameTransformGet_nwc_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      std_msgs::std_msgs__rosidl_typesupport_cpp
      geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
      tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
      Ros2Utils
  )

  yarp_install(
    TARGETS yarp_frameTransformGet_nwc_ros2
    EXPORT yarp-device-rframeTransformGet_nwc_ros2
    COMPONENT yarp-device-frameTransformGet_nwc_ros2
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  if(YARP_COMPILE_TESTS)
      add_subdirectory(tests)
  endif()

  set_property(TARGET yarp_frameTransformGet_nwc_ros2 PROPERTY FOLDER "Plugins/Device")
endif()
