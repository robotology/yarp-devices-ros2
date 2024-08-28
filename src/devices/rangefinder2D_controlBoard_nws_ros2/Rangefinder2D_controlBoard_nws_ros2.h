/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_RANGEFINDER2D_CONTROLBOARD_NWS_ROS2_H
#define YARP_ROS2_RANGEFINDER2D_CONTROLBOARD_NWS_ROS2_H

#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IAxisInfo.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "Rangefinder2D_controlBoard_nws_ros2_ParamsParser.h"

#include <mutex>

/**
 *  @ingroup dev_impl_nws_ros2
 *
 * \brief `Rangefinder2D_nws_ros2`:  A Network grabber that simultaneously publishes the joint states and the measurements from a 2DRangefinder.
 *
 * This device was developed for testing purposes only with fake/simulated controllers.
 * No documentation is provided for this device. Please do not use it on a real robot.
 *
 */
class Rangefinder2D_controlBoard_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
        public yarp::dev::WrapperSingle,
        Rangefinder2D_controlBoard_nws_ros2_ParamsParser
{
public:
    Rangefinder2D_controlBoard_nws_ros2();
    Rangefinder2D_controlBoard_nws_ros2(const Rangefinder2D_controlBoard_nws_ros2&) = delete;
    Rangefinder2D_controlBoard_nws_ros2(Rangefinder2D_controlBoard_nws_ros2&&) noexcept = delete;
    Rangefinder2D_controlBoard_nws_ros2& operator=(const Rangefinder2D_controlBoard_nws_ros2&) = delete;
    Rangefinder2D_controlBoard_nws_ros2& operator=(Rangefinder2D_controlBoard_nws_ros2&&) noexcept = delete;
    ~Rangefinder2D_controlBoard_nws_ros2() override = default;

    //WrapperSingle
    bool attach(yarp::dev::PolyDriver* driver) override;
    bool detach() override;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    bool setDevice(yarp::dev::DeviceDriver* device);
    bool updateAxisName();

    // PeriodicThread
    void run() override;

private:
    yarp::dev::PolyDriver m_driver;
    yarp::dev::DeviceDriver* m_driver_cb =nullptr;
    yarp::dev::IRangefinder2D *m_iLidar =nullptr;
    yarp::dev::IPositionControl* iPositionControl{nullptr};
    yarp::dev::IEncodersTimed* iEncodersTimed{nullptr};
    yarp::dev::ITorqueControl* iTorqueControl{nullptr};
    yarp::dev::IAxisInfo* iAxisInfo{nullptr};
    std::vector<std::string> jointNames;
    size_t subdevice_joints {0};


    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_publisher_laser;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher_joint;
    bool m_isDeviceReady = false;
    yarp::sig::Vector m_times;

    double m_minAngle, m_maxAngle;
    double m_minDistance, m_maxDistance;
    double m_resolution;
    sensor_msgs::msg::JointState m_ros_struct;
};

#endif // YARP_ROS2_RANGEFINDER2D_CONTROLBOARD_NWS_ROS2_H
