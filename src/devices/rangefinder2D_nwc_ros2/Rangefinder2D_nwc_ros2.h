/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_RANGEFINDER2D_NWC_ROS2_H
#define YARP_ROS2_RANGEFINDER2D_NWC_ROS2_H

#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/DeviceDriver.h>
#include <Ros2Spinner.h>


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <Ros2Subscriber.h>

#include "Rangefinder2D_nwc_ros2_ParamsParser.h"

#include <mutex>

/**
 *  @ingroup dev_impl_nwc_ros2 dev_impl_lidar
 *
 * \brief `Rangefinder2D_nwc_ros2`:  A Network grabber for 2D Rangefinder devices
 *
 *  Documentation to be added
 *
 */
class Rangefinder2D_nwc_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IRangefinder2D,
        Rangefinder2D_nwc_ros2_ParamsParser
{
public:
    Rangefinder2D_nwc_ros2();
    Rangefinder2D_nwc_ros2(const Rangefinder2D_nwc_ros2&) = delete;
    Rangefinder2D_nwc_ros2(Rangefinder2D_nwc_ros2&&) noexcept = delete;
    Rangefinder2D_nwc_ros2& operator=(const Rangefinder2D_nwc_ros2&) = delete;
    Rangefinder2D_nwc_ros2& operator=(Rangefinder2D_nwc_ros2&&) noexcept = delete;
    ~Rangefinder2D_nwc_ros2() override = default;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // Spinner
    Ros2Spinner*            m_spinner{nullptr};

    // ROS2 Topic Callback
    void callback(sensor_msgs::msg::LaserScan::SharedPtr msg, std::string topic);

public:
    // IRangeFinder2D
    yarp::dev::ReturnValue getLaserMeasurement(std::vector<yarp::sig::LaserMeasurementData> &data, double* timestamp = nullptr) override;
    yarp::dev::ReturnValue getRawData(yarp::sig::Vector &data, double* timestamp = nullptr) override;
    yarp::dev::ReturnValue getDeviceStatus(Device_status& status) override;
    yarp::dev::ReturnValue getDistanceRange(double& min, double& max) override;
    yarp::dev::ReturnValue setDistanceRange(double min, double max) override;
    yarp::dev::ReturnValue getScanLimits(double& min, double& max) override;
    yarp::dev::ReturnValue setScanLimits(double min, double max) override;
    yarp::dev::ReturnValue getHorizontalResolution(double& step) override;
    yarp::dev::ReturnValue setHorizontalResolution(double step) override;
    yarp::dev::ReturnValue getScanRate(double& rate) override;
    yarp::dev::ReturnValue setScanRate(double rate) override;
    yarp::dev::ReturnValue getDeviceInfo(std::string &device_info) override;

private:
    rclcpp::Node::SharedPtr m_node;
    Ros2Subscriber<Rangefinder2D_nwc_ros2,sensor_msgs::msg::LaserScan>* m_subscriber;

    bool   m_verbose = false;
    bool   m_data_valid = false;
    std::mutex m_mutex;
    double m_minAngle, m_maxAngle;
    double m_minDistance, m_maxDistance;
    double m_resolution;
    double m_period;
    double m_timestamp;
    yarp::sig::Vector m_data;
    std::string m_frame_id;
};

#endif // YARP_ROS2_RANGEFINDER2D_NWC_ROS2_H
