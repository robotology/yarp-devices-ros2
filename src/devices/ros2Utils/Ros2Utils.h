/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_ROS2UTILS_H
#define YARP_ROS2_ROS2UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

class NodeCreator
{
public:
    static rclcpp::Node::SharedPtr createNode(std::string name);
    static rclcpp::Node::SharedPtr createNode(std::string name, rclcpp::NodeOptions& node_options);
};


builtin_interfaces::msg::Time ros2TimeFromYarp(double yarpTime);

#endif // YARP_ROS2_ROS2UTILS_H
