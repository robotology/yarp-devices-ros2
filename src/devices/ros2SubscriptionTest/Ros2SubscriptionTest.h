/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_ROS2_ROS2SUBSCRIPTIONTEST_H
#define YARP_ROS2_ROS2SUBSCRIPTIONTEST_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/Thread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex>

//class Ros2Init
//{
//public:
//    Ros2Init();
//
//    std::shared_ptr<rclcpp::Node> node;
//
//    static Ros2Init& get();
//};

/*****
 * in ros2 non puoi inizializzare i nomi dei nodi con lo /
 * e non possono contenere caratteri alfanumerici diversi da _
 *
 * puoi chiamare i nodi con lo stesso nome ma per ros2 sono due nodi diversi
 * try catch, purtroppo non esiste un modo per controllare se e' gia' inizializzato....
 * esisteva ma loo hanno deprecato. si puo' controllare se ci sono alternative
 */

class MinimalPublisher
{
public:
    MinimalPublisher(const std::string& topicname);
};


class Ros2SubscriptionTest :
        public yarp::dev::DeviceDriver,
        public yarp::os::Thread
{
public:
    Ros2SubscriptionTest();
    Ros2SubscriptionTest(const Ros2SubscriptionTest&) = delete;
    Ros2SubscriptionTest(Ros2SubscriptionTest&&) noexcept = delete;
    Ros2SubscriptionTest& operator=(const Ros2SubscriptionTest&) = delete;
    Ros2SubscriptionTest& operator=(Ros2SubscriptionTest&&) noexcept = delete;
    ~Ros2SubscriptionTest() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void local_callback(const std_msgs::msg::String::SharedPtr msg) const;

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;

    std::string m_topic;
    std::string m_node_name;
    size_t m_count {0};

};

#endif // YARP_ROS2_ROS2SUBSCRIPTIONTEST_H
