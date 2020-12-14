/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_ROS2_ROS2WRAPPER_H
#define YARP_ROS2_ROS2WRAPPER_H

#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <mutex>


class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};


class Ros2Wrapper :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
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
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:
	yarp::dev::PolyDriver m_driver;
    yarp::dev::IRangefinder2D *iDevice;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_publisher;
    std::string m_topic;
    bool isDeviceOwned = false;
    
    double minAngle, maxAngle;
    double minDistance, maxDistance;
    double resolution;
    std::string frame_id;
    std::string sensorId;
};

#endif // YARP_ROS2_ROS2TEST_H
