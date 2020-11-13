/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_ROS2_ROS2TEST_H
#define YARP_ROS2_ROS2TEST_H

#include <yarp/dev/DeviceDriver.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalPublisher :
    public rclcpp::Node
{
public:
    MinimalPublisher();

  private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
  };


class Ros2Test :
        public yarp::dev::DeviceDriver
{
public:
    Ros2Test() = default;
    Ros2Test(const Ros2Test&) = delete;
    Ros2Test(Ros2Test&&) noexcept = delete;
    Ros2Test& operator=(const Ros2Test&) = delete;
    Ros2Test& operator=(Ros2Test&&) noexcept = delete;
    ~Ros2Test() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

private:
    std::shared_ptr<MinimalPublisher> m_pub;
};

#endif // YARP_ROS2_ROS2TEST_H
