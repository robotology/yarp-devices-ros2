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
        public yarp::dev::IRangefinder2D
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
    bool getLaserMeasurement(std::vector<yarp::dev::LaserMeasurementData> &data, double* timestamp = nullptr) override;
    bool getRawData(yarp::sig::Vector &data, double* timestamp = nullptr) override;
    bool getDeviceStatus(Device_status& status) override;
    bool getDistanceRange(double& min, double& max) override;
    bool setDistanceRange(double min, double max) override;
    bool getScanLimits(double& min, double& max) override;
    bool setScanLimits(double min, double max) override;
    bool getHorizontalResolution(double& step) override;
    bool setHorizontalResolution(double step) override;
    bool getScanRate(double& rate) override;
    bool setScanRate(double rate) override;
    bool getDeviceInfo(std::string &device_info) override;

private:
    std::string m_topic_name;
    std::string m_node_name;
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
