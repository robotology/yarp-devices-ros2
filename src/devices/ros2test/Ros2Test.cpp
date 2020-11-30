/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Ros2Test.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std::chrono_literals;

YARP_LOG_COMPONENT(ROS2TEST, "yarp.ros2.ros2test", yarp::os::Log::TraceType);


Ros2Init::Ros2Init()
{
    rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    node = std::make_shared<rclcpp::Node>("yarprobotinterface_node");
}

Ros2Init& Ros2Init::get()
{
    static Ros2Init instance;
    return instance;
}


Ros2Test::Ros2Test() :
        yarp::os::PeriodicThread(0.5)
{
}

bool Ros2Test::open(yarp::os::Searchable& config)
{
    yCTrace(ROS2TEST);
    m_topic = config.check("topic", yarp::os::Value("ros2test_topic"), "Name of the ROS topic").asString();
    yCInfo(ROS2TEST, "Ros2Test::open - %s", m_topic.c_str());

    m_publisher = Ros2Init::get().node->create_publisher<std_msgs::msg::String>(m_topic, 10);

    start();
    return true;
}

bool Ros2Test::close()
{
    yCTrace(ROS2TEST);
    yCInfo(ROS2TEST, "Ros2Test::close");
    return true;
}

void Ros2Test::run()
{
    yCTrace(ROS2TEST);
    auto message = std_msgs::msg::String();
    message.data = "Hello, " + m_topic + "! " + std::to_string(m_count++);
    yCInfo(ROS2TEST, "Publishing: '%s'", message.data.c_str());
    m_publisher->publish(message);
}
