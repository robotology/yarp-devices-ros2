/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define _USE_MATH_DEFINES
#include "MobileBaseVelocityControl_nws_ros2.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <yarp/dev/INavigation2D.h>

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

//------------------------------------------------------------------------------------------------------------------------------


bool MobileBaseVelocityControl_nws_ros2::open(yarp::os::Searchable& config)
{
    if (config.check("node_name"))
    {
        m_ros2_node_name = config.find("node_name").asString();
    }

    if (config.check("topic_name"))
    {
        m_ros2_topic_name = config.find("topic_name").asString();
    }
    try {
        if (!rclcpp::ok()) {
            rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
        }
        m_node = std::make_shared<rclcpp::Node>(m_ros2_node_name);
    } catch (const std::exception& e) {
        std::cout << const_cast<char *>(e.what());
        return false;
    }
    m_ros2_subscriber = m_node->create_subscription<geometry_msgs::msg::Twist>(m_ros2_topic_name,
                                                                                10,
                                                                                std::bind(&MobileBaseVelocityControl_nws_ros2::twist_callback, this, _1));

    if (!m_ros2_subscriber)
    {
        yCError(MOBVEL_NWS_ROS2) << " opening " << m_ros2_topic_name << " Topic, check your ROS2 network configuration\n";
        return false;
    }

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

    return true;
}


bool MobileBaseVelocityControl_nws_ros2::close()
{
    yCInfo(MOBVEL_NWS_ROS2, "shutting down");
    rclcpp::shutdown();
    return true;
}

void MobileBaseVelocityControl_nws_ros2::run()
{
    yCInfo(MOBVEL_NWS_ROS2, "starting");
    rclcpp::spin(m_node);
}