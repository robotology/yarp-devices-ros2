# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(RESNAME ros2_control_demo)

file(GLOB apps ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${apps}    DESTINATION ${YARP-DEVICES-ROS2_APPLICATIONS_INSTALL_DIR})