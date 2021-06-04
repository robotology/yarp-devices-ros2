/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
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
    ~Ros2Spinner()=default;

    void run();
    void threadRelease() override;
};
