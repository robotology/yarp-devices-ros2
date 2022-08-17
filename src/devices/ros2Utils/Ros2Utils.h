/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
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
