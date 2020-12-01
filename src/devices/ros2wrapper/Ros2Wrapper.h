/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_ROS2_ROS2WRAPPER_H
#define YARP_ROS2_ROS2WRAPPER_H

#include <../ros2test/ros2test.h>
#include <yarp/dev/IMultipleWrapper.h>

class Ros2Wrapper :
        public Ros2Test,
        public yarp::dev::IMultipleWrapper
{
public:
    Ros2Wrapper();
    Ros2Wrapper(const Ros2Wrapper&) = delete;
    Ros2Wrapper(Ros2Wrapper&&) noexcept = delete;
    Ros2Wrapper& operator=(const Ros2Wrapper&) = delete;
    Ros2Wrapper& operator=(Ros2Wrapper&&) noexcept = delete;
    ~Ros2Wrapper() override = default;

    //IMultipleWrapper
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;
    
    void attach(yarp::dev::IRangefinder2D *s);
    void detach();
    
    // DeviceDriver
    //bool open(yarp::os::Searchable& config) override;
    //bool close() override;

    // PeriodicThread
    //void run() override;

private:
    yarp::dev::IRangefinder2D *iDevice;
};

#endif // YARP_ROS2_ROS2TEST_H
