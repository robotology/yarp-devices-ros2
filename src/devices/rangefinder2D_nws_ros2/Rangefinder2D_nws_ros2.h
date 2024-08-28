/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_RANGEFINDER2D_NWS_ROS2_H
#define YARP_ROS2_RANGEFINDER2D_NWS_ROS2_H

#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "Rangefinder2D_nws_ros2_ParamsParser.h"

#include <mutex>

/**
 *  @ingroup dev_impl_nws_ros2 dev_impl_lidar
 *
 * \brief `Rangefinder2D_nws_ros2`:  A Network grabber for 2D Rangefinder devices
 *
 *  Documentation to be added
 *
 */
class Rangefinder2D_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
        public yarp::dev::WrapperSingle,
        Rangefinder2D_nws_ros2_ParamsParser
{
public:
    Rangefinder2D_nws_ros2();
    Rangefinder2D_nws_ros2(const Rangefinder2D_nws_ros2&) = delete;
    Rangefinder2D_nws_ros2(Rangefinder2D_nws_ros2&&) noexcept = delete;
    Rangefinder2D_nws_ros2& operator=(const Rangefinder2D_nws_ros2&) = delete;
    Rangefinder2D_nws_ros2& operator=(Rangefinder2D_nws_ros2&&) noexcept = delete;
    ~Rangefinder2D_nws_ros2() override = default;

    //WrapperSingle
    bool attach(yarp::dev::PolyDriver* driver) override;
    bool detach() override;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:
    yarp::dev::PolyDriver m_driver;
    yarp::dev::IRangefinder2D *m_iDevice =nullptr;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_publisher;
    bool m_isDeviceOwned = false;

    double m_minAngle, m_maxAngle;
    double m_minDistance, m_maxDistance;
    double m_resolution;
};

#endif // YARP_ROS2_ROS2TEST_H
