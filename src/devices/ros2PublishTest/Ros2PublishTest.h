/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_ROS2_ROS2PUBLISHTEST_H
#define YARP_ROS2_ROS2PUBLISHTEST_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(std::string name, std::string topic);
    void publish(std::string messageString);

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

class Ros2PublishTest :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread
{
public:
    Ros2PublishTest();
    Ros2PublishTest(const Ros2PublishTest&) = delete;
    Ros2PublishTest(Ros2PublishTest&&) noexcept = delete;
    Ros2PublishTest& operator=(const Ros2PublishTest&) = delete;
    Ros2PublishTest& operator=(Ros2PublishTest&&) noexcept = delete;
    ~Ros2PublishTest() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:

    MinimalPublisher* minimalPublisher;
    std::string m_node_name;
    std::string m_topic;
};

#endif // YARP_ROS2_ROS2PUBLISHTEST_H
