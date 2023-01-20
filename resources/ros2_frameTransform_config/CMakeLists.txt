# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(RESNAME ros2_frameTransform_config)

file(GLOB ft_config      ${CMAKE_CURRENT_SOURCE_DIR}/*.xml)

yarp_install(FILES ${ft_config} DESTINATION ${YARP-DEVICES-ROS2_CONTEXTS_INSTALL_DIR}/${RESNAME})
