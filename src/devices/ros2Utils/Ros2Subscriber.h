/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ROS2_SUBSCRIBER_H
#define ROS2_SUBSCRIBER_H

#include <iostream>
#include <cstring>

#include <yarp/os/LogStream.h>

#include <yarp/os/LogComponent.h>
#include <rclcpp/rclcpp.hpp>



template <class CallbackClass, typename MsgClass>
    class Ros2Subscriber
{
public:
    Ros2Subscriber(rclcpp::Node::SharedPtr node, CallbackClass* callbackClass);

    void subscribe_to_topic(std::string topic_name);
private:
    rclcpp::Node::SharedPtr m_node;
    CallbackClass* m_callbackClass;
    std::vector<typename rclcpp::Subscription<MsgClass>::SharedPtr> m_subscription;
};

template <class CallbackClass, typename MsgClass>
Ros2Subscriber<CallbackClass, MsgClass>::Ros2Subscriber(rclcpp::Node::SharedPtr node, CallbackClass* callbackClass)
{
    m_node = node;
    m_callbackClass = callbackClass;
}


template <class CallbackClass, typename MsgClass>
void Ros2Subscriber<CallbackClass, MsgClass>::subscribe_to_topic(std::string topic_name) {
    yInfo() << "Ros2Subscriber creating topic: " << topic_name;
    m_subscription.push_back(m_node->create_subscription<MsgClass>(
        topic_name,
        10,
        [this, topic_name](const typename MsgClass::SharedPtr msg) {
            m_callbackClass->callback(msg, topic_name);
        }));
}
#endif //ROS2_SUBSCRIBER_H
