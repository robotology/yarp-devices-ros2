/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Ros2SubscriptionTest.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

YARP_LOG_COMPONENT(ROS2SUBSCRIPTIONTEST, "yarp.ros2.ros2SubscriptionTest", yarp::os::Log::TraceType);

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber(std::string name, Ros2SubscriptionTest *subscriptionTest, std::string topic)
    : Node(name)
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            topic, 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
        m_subscriptionTest = subscriptionTest;
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        m_subscriptionTest->local_callback(msg);
    }
    Ros2SubscriptionTest *m_subscriptionTest;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};





//Ros2Init::Ros2Init()
//{
//    rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
//    node = std::make_shared<rclcpp::Node>("yarprobotinterface_node");
//}
//
//Ros2Init& Ros2Init::get()
//{
//    static Ros2Init instance;
//    return instance;
//}


Ros2SubscriptionTest::Ros2SubscriptionTest()
{
}

bool Ros2SubscriptionTest::open(yarp::os::Searchable& config)
{
    if (config.check("node_name"))
    {
        m_node_name = config.find("node_name").asString();
    } else {
        yCError(ROS2SUBSCRIPTIONTEST) << "missing node_name parameter";
        return false;
    }

    if (config.check("topic_name"))
    {
        m_topic = config.find("topic_name").asString();
    } else {
        yCError(ROS2SUBSCRIPTIONTEST) << "missing topic_name parameter";
        return false;
    }
    start();
    return true;
}

bool Ros2SubscriptionTest::close()
{
    yCTrace(ROS2SUBSCRIPTIONTEST);
    yCInfo(ROS2SUBSCRIPTIONTEST, "Ros2SubscriptionTest::close");
    return true;
}

void Ros2SubscriptionTest::local_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    yInfo() << "hello, I'm " + m_node_name + m_topic + ", I heard: '%s'" << msg->data.c_str();
}

void Ros2SubscriptionTest::run()
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    rclcpp::spin(std::make_shared<MinimalSubscriber>(m_node_name, this, m_topic));
    rclcpp::shutdown();
}
