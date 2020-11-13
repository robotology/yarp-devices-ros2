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
};

#endif // YARP_ROS2_ROS2TEST_H
