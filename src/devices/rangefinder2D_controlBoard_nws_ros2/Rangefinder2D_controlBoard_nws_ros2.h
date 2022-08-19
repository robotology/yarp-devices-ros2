/*
 * Copyright (C) 2006-2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
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
        public yarp::dev::WrapperSingle
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

    bool setDevice(yarp::dev::DeviceDriver* device, bool owned);
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
    bool m_isDeviceOwned_laser = false;
    yarp::sig::Vector m_times;

    double m_minAngle, m_maxAngle;
    double m_minDistance, m_maxDistance;
    double m_resolution;
    double m_period;
    bool   m_subdevice_owned_cb = false;
    sensor_msgs::msg::JointState m_ros_struct;

    std::string m_topic;
    std::string m_topic_cb;
    std::string m_node_name;
    std::string m_frame_id;
};

#endif // YARP_ROS2_RANGEFINDER2D_CONTROLBOARD_NWS_ROS2_H
