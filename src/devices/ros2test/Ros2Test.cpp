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

YARP_LOG_COMPONENT(ROS2TEST, "yarp.ros2.ros2test");

MinimalPublisher::MinimalPublisher() :
        Node("ros2test_node"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("ros2test_topic", 10);
    timer_ = this->create_wall_timer(
        500ms,
        std::bind(&MinimalPublisher::timer_callback, this)
    );
}

void MinimalPublisher::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

bool Ros2Test::open(yarp::os::Searchable& config)
{
    yCInfo(ROS2TEST, "Ros2Test::open");
    YARP_UNUSED(config);
    rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    m_pub = std::make_shared<MinimalPublisher>();
    rclcpp::spin(m_pub);
    return true;
}

bool Ros2Test::close()
{
    yCInfo(ROS2TEST, "Ros2Test::close");
    return true;
}
