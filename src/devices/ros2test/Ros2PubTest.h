/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_ROS2PUBTEST_H
#define YARP_ROS2_ROS2PUBTEST_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>

//----------------------------------------------------------------------

class Ros2Init
{
public:
    Ros2Init();
    std::shared_ptr<rclcpp::Node> node;
    static Ros2Init& get();
};

//----------------------------------------------------------------------

class MinimalPublisher
{
public:
    MinimalPublisher(const std::string& topicname);
};

//----------------------------------------------------------------------

class Ros2PubTest :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread
{
public:
    Ros2PubTest();
    Ros2PubTest(const Ros2PubTest&) = delete;
    Ros2PubTest(Ros2PubTest&&) noexcept = delete;
    Ros2PubTest& operator=(const Ros2PubTest&) = delete;
    Ros2PubTest& operator=(Ros2PubTest&&) noexcept = delete;
    ~Ros2PubTest() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    size_t m_count {0};
};

#endif // YARP_ROS2_ROS2PUBTEST_H
