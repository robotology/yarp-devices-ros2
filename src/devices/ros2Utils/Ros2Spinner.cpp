/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
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
    if(!m_spun)  //This is just a temporary solution.
    {
        rclcpp::spin(m_node);
        m_spun = true;
    }
}

void Ros2Spinner::threadRelease()
{
    if(m_spun)
    {
        rclcpp::shutdown();
        m_spun = false;
    }
}