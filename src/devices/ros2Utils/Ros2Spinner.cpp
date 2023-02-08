/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Ros2Spinner.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

namespace {
YARP_LOG_COMPONENT(ROS2SPINNER, "yarp.ros2.Ros2Spinner")
}

Ros2Spinner::Ros2Spinner(std::shared_ptr<rclcpp::Node> input_node) :
m_node(input_node)
{}

void Ros2Spinner::run()
{
    if(!m_spun)
    {
        m_spun = true;
        rclcpp::spin(m_node);
    }
}

Ros2Spinner::~Ros2Spinner()
{
    if(m_spun)
    {
        rclcpp::shutdown();
        m_spun = false;
    }
}
