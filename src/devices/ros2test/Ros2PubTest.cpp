/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Ros2Test.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std::chrono_literals;

YARP_LOG_COMPONENT(ROS2PUBTEST, "yarp.ros2.ros2PubTest", yarp::os::Log::TraceType);

//----------------------------------------------------------------------

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

//----------------------------------------------------------------------

Ros2PubTest::Ros2PubTest() :
        yarp::os::PeriodicThread(0.5)
{
}

bool Ros2PubTest::open(yarp::os::Searchable& config)
{
    m_publisher = Ros2Init::get().node->create_publisher<std_msgs::msg::String>("/pub_test_topic", 10);
    start();
    return true;
}

bool Ros2PubTest::close()
{
    yCTrace(ROS2PUBTEST);
    yCInfo(ROS2PUBTEST, "Ros2PubTest::close");
    return true;
}

void Ros2PubTest::run()
{
    yCTrace(ROS2PUBTEST);
    auto message = std_msgs::msg::String();
    message.data = "Hello, " + m_topic + "! " + std::to_string(m_count++);
    yCInfo(ROS2PUBTEST, "Publishing: '%s'", message.data.c_str());
    m_publisher->publish(message);
}
