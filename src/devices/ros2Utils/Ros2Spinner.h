/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <rclcpp/rclcpp.hpp>
#include <yarp/os/Thread.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>

class Ros2Spinner : public yarp::os::Thread
{
private:
    bool m_spun{false};
    std::shared_ptr<rclcpp::Node> m_node;
public:
    Ros2Spinner(std::shared_ptr<rclcpp::Node> input_node);
    ~Ros2Spinner();

    void run() override;
};
