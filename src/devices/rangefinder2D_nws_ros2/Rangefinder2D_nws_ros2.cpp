/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "Rangefinder2D_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>


#include <cmath>
#include <iostream>
#include <Ros2Utils.h>


using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;


YARP_LOG_COMPONENT(RANGEFINDER2D_NWS_ROS2, "yarp.ros2.rangefinder2D_nws_ros2", yarp::os::Log::TraceType);


Rangefinder2D_nws_ros2::Rangefinder2D_nws_ros2() :
        yarp::os::PeriodicThread(0.01)
{
}

bool Rangefinder2D_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{
    if (driver->isValid())
    {
        driver->view(m_iDevice);
    }

    //attach the hardware device
    if (nullptr == m_iDevice)
    {
        yCError(RANGEFINDER2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    //get information/parameters from the hardware device etc
    if(!m_iDevice->getDistanceRange(m_minDistance, m_maxDistance))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide min & max distance range.";
        return false;
    }

    if(!m_iDevice->getScanLimits(m_minAngle, m_maxAngle))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide min & max angle scan range.";
        return false;
    }

    if (!m_iDevice->getHorizontalResolution(m_resolution))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide horizontal resolution ";
        return false;
    }

   return true;
}

bool Rangefinder2D_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_iDevice = nullptr;
    return true;
}

void Rangefinder2D_nws_ros2::run()
{
    auto message = std_msgs::msg::String();

    if (m_iDevice!=nullptr)
    {
        bool ret = true;
        IRangefinder2D::Device_status status;
        yarp::sig::Vector ranges;
        double synchronized_timestamp = 0;
        ret &= m_iDevice->getRawData(ranges, &synchronized_timestamp);
        ret &= m_iDevice->getDeviceStatus(status);

        if (ret)
        {
            int ranges_size = ranges.size();

            sensor_msgs::msg::LaserScan rosData;

            if (!std::isnan(synchronized_timestamp))
            {
//                rosData.header.stamp(synchronized_timestamp);
                rosData.header.stamp.sec = int(synchronized_timestamp); // FIXME
                rosData.header.stamp.nanosec = static_cast<int>(1000000000UL * (synchronized_timestamp - int(synchronized_timestamp))); // FIXME
            }
            else
            {
                rosData.header.stamp.sec = int(yarp::os::Time::now()); // FIXME
                rosData.header.stamp.nanosec = static_cast<int>(1000000000UL * (yarp::os::Time::now() - int(yarp::os::Time::now()))); // FIXME
            }

            rosData.header.frame_id = m_frame_id;
            rosData.angle_min = m_minAngle * M_PI / 180.0;
            rosData.angle_max = m_maxAngle * M_PI / 180.0;
            rosData.angle_increment = m_resolution * M_PI / 180.0;
            rosData.time_increment = 0;             // all points in a single scan are considered took at the very same time
            rosData.scan_time = getPeriod();        // time elapsed between two successive readings
            rosData.range_min = m_minDistance;
            rosData.range_max = m_maxDistance;
            rosData.ranges.resize(ranges_size);
            rosData.intensities.resize(ranges_size);

            for (int i = 0; i < ranges_size; i++)
            {
                // in yarp, NaN is used when a scan value is missing. For example when the angular range of the rangefinder is smaller than 360.
                // is ros, NaN is not used. Hence this check replaces NaN with inf.
                if (std::isnan(ranges[i]))
                {
                   rosData.ranges[i] = std::numeric_limits<double>::infinity();
                   rosData.intensities[i] = 0.0;
                }
                else
                {
                   rosData.ranges[i] = ranges[i];
                   rosData.intensities[i] = 0.0;
                }
            }
            m_publisher->publish(rosData);
        }
        else
        {
            yCError(RANGEFINDER2D_NWS_ROS2, "Sensor returned error");
        }
    }
}

bool Rangefinder2D_nws_ros2::open(yarp::os::Searchable &config)
{
    parseParams(config);
    //create the topic
    m_node = NodeCreator::createNode(m_node_name);
    m_publisher = m_node->create_publisher<sensor_msgs::msg::LaserScan>(m_topic_name, 10);
    yCInfo(RANGEFINDER2D_NWS_ROS2, "Opened topic: %s", m_topic_name.c_str());

    yCInfo(RANGEFINDER2D_NWS_ROS2, "Waiting for device to attach");

    //start the publishig thread
    setPeriod(m_period);
    start();
    return true;
}

bool Rangefinder2D_nws_ros2::close()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    return true;
}
