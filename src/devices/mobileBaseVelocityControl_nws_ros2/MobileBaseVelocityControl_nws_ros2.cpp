/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _USE_MATH_DEFINES
#include "MobileBaseVelocityControl_nws_ros2.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/INavigation2D.h>
#include <Ros2Utils.h>

#include <cmath>
#include <mutex>
/*! \file MobileBaseVelocityControl_nws_ros2.cpp */

using std::placeholders::_1;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace yarp::os;
using namespace yarp::sig;

namespace {
    YARP_LOG_COMPONENT(MOBVEL_NWS_ROS2, "yarp.device.MobileBaseVelocityControl_nws_ros2")
}

bool MobileBaseVelocityControl_nws_ros2::open(yarp::os::Searchable& config)
{
    parseParams(config);
    m_node = NodeCreator::createNode(m_node_name);
    m_ros2_subscriber = m_node->create_subscription<geometry_msgs::msg::Twist>(m_topic_name,
                                                                                10,
                                                                                std::bind(&MobileBaseVelocityControl_nws_ros2::twist_callback, this, _1));

    if (!m_ros2_subscriber)
    {
        yCError(MOBVEL_NWS_ROS2) << " opening " << m_topic_name << " Topic, check your ROS2 network configuration\n";
        return false;
    }

    m_spinner = new Ros2Spinner(m_node);
    m_spinner->start();

    yCInfo(MOBVEL_NWS_ROS2) << "Waiting for device to attach";

    return true;
}

void MobileBaseVelocityControl_nws_ros2::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->m_iNavVel->applyVelocityCommand(msg->linear.x,
                                          msg->linear.y,
                                          msg->angular.z * 180 / M_PI);

}

bool MobileBaseVelocityControl_nws_ros2::detach()
{
    m_iNavVel = nullptr;
    return true;
}

bool MobileBaseVelocityControl_nws_ros2::attach(PolyDriver* driver)
{
    if (driver->isValid())
    {
        driver->view(m_iNavVel);
    }

    if (nullptr == m_iNavVel)
    {
        yCError(MOBVEL_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    yCInfo(MOBVEL_NWS_ROS2) << "Device attached";

    return true;
}


bool MobileBaseVelocityControl_nws_ros2::close()
{
    yCInfo(MOBVEL_NWS_ROS2, "closing...");
    delete m_spinner;
    yCInfo(MOBVEL_NWS_ROS2, "closed");
    return true;
}
