/*
 * Copyright (C) 2006-2022 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "Rangefinder2D_controlBoard_nws_ros2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>


#include <cmath>
#include <iostream>
#include <Ros2Utils.h>


using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;


YARP_LOG_COMPONENT(RANGEFINDER2D_NWS_ROS2, "yarp.ros2.rangefinder2D_controlBoard_nws_ros2", yarp::os::Log::TraceType);


inline double convertDegreesToRadians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

Rangefinder2D_controlBoard_nws_ros2::Rangefinder2D_controlBoard_nws_ros2() :
        yarp::os::PeriodicThread(0.01)
{
}

bool Rangefinder2D_controlBoard_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{
    if (driver->isValid())
    {
        driver->view(m_iLidar);
    }

    if (!setDevice(driver))
    {
        return false;
    }

    //attach the hardware device
    if (nullptr == m_iLidar)
    {
        yCError(RANGEFINDER2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    //get information/parameters from the hardware device etc
    if(!m_iLidar->getDistanceRange(m_minDistance, m_maxDistance))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide min & max distance range.";
        return false;
    }

    if(!m_iLidar->getScanLimits(m_minAngle, m_maxAngle))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide min & max angle scan range.";
        return false;
    }

    if (!m_iLidar->getHorizontalResolution(m_resolution))
    {
        yCError(RANGEFINDER2D_NWS_ROS2) << "Laser device does not provide horizontal resolution ";
        return false;
    }

   m_isDeviceReady = true;
   return true;
}

bool Rangefinder2D_controlBoard_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_iLidar = nullptr;
    return true;
}

void Rangefinder2D_controlBoard_nws_ros2::run()
{
    if (!m_isDeviceReady) return;

    auto message = std_msgs::msg::String();
    sensor_msgs::msg::LaserScan rosData;

    if (m_iLidar!=nullptr)
    {
        bool ret = true;
        IRangefinder2D::Device_status status;
        yarp::sig::Vector ranges;
        double synchronized_timestamp = 0;
        ret &= m_iLidar->getRawData(ranges, &synchronized_timestamp);
        ret &= m_iLidar->getDeviceStatus(status);

        if (ret)
        {
            int ranges_size = ranges.size();

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
            m_publisher_laser->publish(rosData);
        }
        else
        {
            yCError(RANGEFINDER2D_NWS_ROS2, "Sensor returned error");
        }
    }

    //-----------------------------------------------------------motor part
    yCAssert(RANGEFINDER2D_NWS_ROS2, iEncodersTimed);
    yCAssert(RANGEFINDER2D_NWS_ROS2, iAxisInfo);

    bool positionsOk = iEncodersTimed->getEncodersTimed(m_ros_struct.position.data(), m_times.data());
    YARP_UNUSED(positionsOk);

    // Data from HW have been gathered few lines before
    JointTypeEnum jType;
    for (size_t i = 0; i < subdevice_joints; i++) {
        iAxisInfo->getJointType(i, jType);
        if (jType == VOCAB_JOINTTYPE_REVOLUTE) {
            m_ros_struct.position[i] = convertDegreesToRadians(m_ros_struct.position[i]);
            m_ros_struct.velocity[i] = convertDegreesToRadians(m_ros_struct.velocity[i]);
        }
    }

    m_ros_struct.name = jointNames;
    m_ros_struct.header.stamp.sec = rosData.header.stamp.sec;
    m_ros_struct.header.stamp.nanosec = rosData.header.stamp.nanosec;
    m_publisher_joint->publish(m_ros_struct);
}

bool Rangefinder2D_controlBoard_nws_ros2::setDevice(yarp::dev::DeviceDriver* driver)
{
    yCAssert(RANGEFINDER2D_NWS_ROS2, driver);

    // Save the pointer and subDeviceOwned
    m_driver_cb = driver;

    m_driver_cb->view(iPositionControl);
    if (!iPositionControl) {
        yCError(RANGEFINDER2D_NWS_ROS2, "<%s - %s>: IPositionControl interface was not found in attached device. Quitting",  m_node_name.c_str(), m_topic_cb.c_str());
        return false;
    }

    m_driver_cb->view(iEncodersTimed);
    if (!iEncodersTimed) {
        yCError(RANGEFINDER2D_NWS_ROS2, "<%s - %s>: IEncodersTimed interface was not found in attached device.. Quitting",  m_node_name.c_str(), m_topic_cb.c_str());
        return false;
    }

    m_driver_cb->view(iTorqueControl);
    if (!iTorqueControl) {
        yCWarning(RANGEFINDER2D_NWS_ROS2, "<%s - %s>: ITorqueControl interface was not found in attached device..",  m_node_name.c_str(), m_topic_cb.c_str());
    }

    m_driver_cb->view(iAxisInfo);
    if (!iAxisInfo) {
        yCError(RANGEFINDER2D_NWS_ROS2, "<%s - %s>: IAxisInfo interface was not found in attached device.. Quitting",  m_node_name.c_str(), m_topic_cb.c_str());
        return false;
    }

    // Get the number of controlled joints
    int tmp_axes;
    if (!iPositionControl->getAxes(&tmp_axes)) {
        yCError(RANGEFINDER2D_NWS_ROS2, "<%s - %s>: Failed to get axes number for attached device. ",  m_node_name.c_str(), m_topic_cb.c_str());
        return false;
    }
    if (tmp_axes <= 0) {
        yCError(RANGEFINDER2D_NWS_ROS2, "<%s - %s>: attached device has an invalid number of joints (%d)",  m_node_name.c_str(), m_topic_cb.c_str(), tmp_axes);
        return false;
    }

    subdevice_joints = static_cast<size_t>(tmp_axes);

    m_times.resize(subdevice_joints);
    m_ros_struct.name.resize(subdevice_joints);
    m_ros_struct.position.resize(subdevice_joints);
    m_ros_struct.velocity.resize(subdevice_joints);
    m_ros_struct.effort.resize(subdevice_joints);

    if (!updateAxisName()) {
        return false;
    }

    m_isDeviceReady = true;
    return true;
}

bool Rangefinder2D_controlBoard_nws_ros2::open(yarp::os::Searchable &config)
{
    //wrapper params
    m_topic    = config.check("topic_name_lidar",  yarp::os::Value("topic_name_lidar"), "Name of the ROS2 topic").asString();
    m_topic_cb   = config.check("topic_name_joint",  yarp::os::Value("topic_name_joint"), "Name of the ROS2 topic").asString();
    m_frame_id = config.check("frame_id",  yarp::os::Value("laser_frame"), "Name of the frameId").asString();
    m_node_name = config.check("node_name",  yarp::os::Value("laser_node"), "Name of the node").asString();
    m_period   = config.check("period", yarp::os::Value(0.010), "Period of the thread").asFloat64();

    //create the topic
    m_node = NodeCreator::createNode(m_node_name);
    m_publisher_laser = m_node->create_publisher<sensor_msgs::msg::LaserScan>(m_topic, 10);
    m_publisher_joint = m_node->create_publisher<sensor_msgs::msg::JointState>(m_topic_cb, 10);
    yCInfo(RANGEFINDER2D_NWS_ROS2, "Opened topic: %s", m_topic.c_str());

    yCError(RANGEFINDER2D_NWS_ROS2) << "Waiting for device to attach";

    //start the publishing thread
    setPeriod(m_period);
    start();
    return true;
}

bool Rangefinder2D_controlBoard_nws_ros2::close()
{
    return true;
}

bool Rangefinder2D_controlBoard_nws_ros2::updateAxisName()
{
    // IMPORTANT!! This function has to be called BEFORE the thread starts,
    // the name has to be correct right from the first message!!
    yCAssert(RANGEFINDER2D_NWS_ROS2, iAxisInfo);

    std::vector<std::string> tmpVect;
    for (size_t i = 0; i < subdevice_joints; i++) {
        std::string tmp;
        bool ret = iAxisInfo->getAxisName(i, tmp);
        if (!ret) {
            yCError(RANGEFINDER2D_NWS_ROS2, "Joint name for axis %zu not found!", i);
            return false;
        }
        tmpVect.emplace_back(tmp);
    }

    yCAssert(RANGEFINDER2D_NWS_ROS2, tmpVect.size() == subdevice_joints);

    jointNames = tmpVect;

    return true;
}
