/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Ros2PublishTest.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;


YARP_LOG_COMPONENT(ROS2PUBLISHTEST, "yarp.ros2.ros2PublishTest", yarp::os::Log::TraceType);

MinimalPublisher::MinimalPublisher(std::string name, std::string topic)
: Node(name)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);
}

void MinimalPublisher::publish(std::string messageString) {
    auto message = std_msgs::msg::String();
    message.data = messageString;
    publisher_->publish(message);
}

Ros2PublishTest::Ros2PublishTest():
PeriodicThread(0.033)
{
}

bool Ros2PublishTest::open(yarp::os::Searchable& config)
{
    yCTrace(ROS2PUBLISHTEST);
    if (config.check("node_name"))
    {
        m_node_name = config.find("node_name").asString();
    } else {
        yCError(ROS2PUBLISHTEST) << "missing node_name parameter";
        return false;
    }

    if (config.check("topic_name"))
    {
        m_topic = config.find("topic_name").asString();
    } else {
        yCError(ROS2PUBLISHTEST) << "missing topic_name parameter";
        return false;
    }


    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    minimalPublisher = new MinimalPublisher(m_node_name, m_topic) ;

    start();
    return true;
}

bool Ros2PublishTest::close()
{
    yCInfo(ROS2PUBLISHTEST, "Ros2PublishTest::close");
    rclcpp::shutdown();
    return true;
}

void Ros2PublishTest::run()
{
    minimalPublisher->publish("Hello from " + m_topic + "! " + m_node_name);
}
