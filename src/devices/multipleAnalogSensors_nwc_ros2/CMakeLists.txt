# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(imu_nwc_ros2
  CATEGORY device
  TYPE Imu_nwc_ros2
  INCLUDE Imu_nwc_ros2.h
  DEPENDS "TARGET YARP::YARP_math"
  DEFAULT ON
)

if(ENABLE_imu_nwc_ros2)
  yarp_add_plugin(yarp_imu_nwc_ros2)

  target_sources(yarp_imu_nwc_ros2
    PRIVATE
      Imu_nwc_ros2.cpp
      Imu_nwc_ros2.h
      GenericSensor_nwc_ros2.h
  )

  target_link_libraries(yarp_imu_nwc_ros2
    PRIVATE
      YARP::YARP_os
      YARP::YARP_sig
      YARP::YARP_dev
      rclcpp::rclcpp
      sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
      tf2::tf2
      Ros2Utils
  )

  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
    YARP_os
    YARP_sig
    YARP_dev
  )

  yarp_install(
    TARGETS yarp_imu_nwc_ros2
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_imu_nwc_ros2 PROPERTY FOLDER "Plugins/Device")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()

endif()
