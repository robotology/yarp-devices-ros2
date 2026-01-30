/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "Rangefinder2D_nwc_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>


#include <cmath>
#include <iostream>
#include <Ros2Utils.h>


using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;


YARP_LOG_COMPONENT(RANGEFINDER2D_NWC_ROS2, "yarp.ros2.rangefinder2D_nwc_ros2", yarp::os::Log::TraceType);


Rangefinder2D_nwc_ros2::Rangefinder2D_nwc_ros2()
{
}

bool Rangefinder2D_nwc_ros2::open(yarp::os::Searchable &config)
{
    parseParams(config);
    m_verbose = m_verbose_on == 1;

    if(m_namespace.empty()) {
        m_node = NodeCreator::createNode(m_node_name);
    } else {
        m_node = NodeCreator::createNode(m_node_name, m_namespace);
    }
    if( m_node == nullptr) {
        yCError(RANGEFINDER2D_NWC_ROS2, "Could not create node %s", m_node_name.c_str());
        return false;
    }
    m_subscriber= new Ros2Subscriber<Rangefinder2D_nwc_ros2, sensor_msgs::msg::LaserScan>(m_node, this);
    m_subscriber->subscribe_to_topic(m_topic_name);

    m_spinner = new Ros2Spinner(m_node);
    m_spinner->start();

    yCInfo(RANGEFINDER2D_NWC_ROS2) << "opened";

    return true;
}

bool Rangefinder2D_nwc_ros2::close()
{
    yCInfo(RANGEFINDER2D_NWC_ROS2, "closing...");
    delete m_subscriber;
    delete m_spinner;
    yCInfo(RANGEFINDER2D_NWC_ROS2, "closed");
    return true;
}

void Rangefinder2D_nwc_ros2::callback(sensor_msgs::msg::LaserScan::SharedPtr msg, std::string topic)
{
    yCTrace(RANGEFINDER2D_NWC_ROS2, "callback LaserScan");
    std::lock_guard<std::mutex> data_guard(m_mutex);

    if (m_data_valid==false)
    {
        m_minAngle = msg->angle_min;
        m_maxAngle = msg->angle_max;
        m_minDistance = msg->range_min;
        m_maxDistance = msg->range_max;
        m_resolution = msg->angle_increment;
        m_data.resize(msg->ranges.size());
        m_data_valid = true;
    }
    if (msg->ranges.size() == m_data.size())
    {
        for (size_t i=0; i<m_data.size(); i++)
        {
            m_data[i] = msg->ranges[i];
        }
    }
    else
    {
        yCError(RANGEFINDER2D_NWC_ROS2, "Data vector changed its size?");
    }
    m_timestamp = (msg->header.stamp.sec + (msg->header.stamp.nanosec / 1000000000));
}

ReturnValue Rangefinder2D_nwc_ros2::getLaserMeasurement(std::vector<yarp::sig::LaserMeasurementData> &data, double* timestamp)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    *timestamp = m_timestamp;
    return ReturnValue_ok;
}

ReturnValue Rangefinder2D_nwc_ros2::getRawData(yarp::sig::Vector &data, double* timestamp)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    data = m_data;
    *timestamp = m_timestamp;
    return ReturnValue_ok;
}

ReturnValue Rangefinder2D_nwc_ros2::getDeviceStatus(Device_status& status)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    return ReturnValue_ok;
}

ReturnValue Rangefinder2D_nwc_ros2::getDistanceRange(double& min, double& max)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    min = m_minDistance;
    max = m_maxDistance;
    return ReturnValue_ok;
}

ReturnValue Rangefinder2D_nwc_ros2::setDistanceRange(double min, double max)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    yCTrace(RANGEFINDER2D_NWC_ROS2, "setDistanceRange not implemented");
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue Rangefinder2D_nwc_ros2::getScanLimits(double& min, double& max)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    min = m_minAngle;
    max = m_maxAngle;
    return ReturnValue_ok;
}

ReturnValue Rangefinder2D_nwc_ros2::setScanLimits(double min, double max)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    yCTrace(RANGEFINDER2D_NWC_ROS2, "setScanLimits not implemented");
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue Rangefinder2D_nwc_ros2::getHorizontalResolution(double& step)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    step = m_resolution;
    return ReturnValue_ok;
}

ReturnValue Rangefinder2D_nwc_ros2::setHorizontalResolution(double step)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    yCTrace(RANGEFINDER2D_NWC_ROS2, "setHorizontalResolution not implemented");
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue Rangefinder2D_nwc_ros2::getScanRate(double& rate)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    return ReturnValue_ok;
}

ReturnValue Rangefinder2D_nwc_ros2::setScanRate(double rate)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    yCTrace(RANGEFINDER2D_NWC_ROS2, "setScanRate not implemented");
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue Rangefinder2D_nwc_ros2::getDeviceInfo(std::string &device_info)
{
    if (!m_data_valid) {return ReturnValue::return_code::return_value_error_generic;}
    std::lock_guard<std::mutex> data_guard(m_mutex);
    return ReturnValue_ok;
}
