/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "frameTransformSet_nwc_ros2.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORMSETNWCROS2, "yarp.device.frameTransformSet_nwc_ros2")
}

//------------------------------------------------------------------------------------------------------------------------------

FrameTransformSet_nwc_ros2::FrameTransformSet_nwc_ros2(double tperiod) :
PeriodicThread(tperiod)
{
}

bool FrameTransformSet_nwc_ros2::open(yarp::os::Searchable& config)
{
    if (!yarp::os::NetworkBase::checkNetwork()) {
        yCError(FRAMETRANSFORMSETNWCROS2,"Error! YARP Network is not initialized");
        return false;
    }

    parseParams(config);

    m_ftContainer.m_timeout = m_GENERAL_refresh_interval;
    setPeriod(m_GENERAL_period);
    m_node = NodeCreator::createNode(m_ROS2_ft_node);
    m_publisherFtTimed = m_node->create_publisher<tf2_msgs::msg::TFMessage>(m_ROS2_ft_topic, 10);
    m_publisherFtStatic = m_node->create_publisher<tf2_msgs::msg::TFMessage>(m_ROS2_ft_topic_static, rclcpp::QoS(10).reliable().durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));

    start();

    return true;
}

bool FrameTransformSet_nwc_ros2::close()
{
    yCTrace(FRAMETRANSFORMSETNWCROS2, "Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    return true;
}

void FrameTransformSet_nwc_ros2::run()
{
    std::lock_guard <std::mutex> lg(m_trf_mutex);
    if (!publishFrameTransforms())
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Error while publishing transforms");
    }
    return;
}

bool FrameTransformSet_nwc_ros2::setTransforms(const std::vector<yarp::math::FrameTransform>& transforms)
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    if(!m_ftContainer.setTransforms(transforms))
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Unable to set transforms");
        return false;
    }
    if(m_GENERAL_asynch_pub)
    {
        if (!publishFrameTransforms())
        {
            yCError(FRAMETRANSFORMSETNWCROS2,"Error while publishing transforms");
            return false;
        }
    }
    return true;
}

bool FrameTransformSet_nwc_ros2::setTransform(const yarp::math::FrameTransform& t)
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    if(!m_ftContainer.setTransform(t))
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Unable to set transform");
        return false;
    }
    if(m_GENERAL_asynch_pub)
    {
        if (!publishFrameTransforms())
        {
            yCError(FRAMETRANSFORMSETNWCROS2,"Error while publishing transforms");
            return false;
        }
    }
    return true;
}

void FrameTransformSet_nwc_ros2::ros2TimeFromYarp(double yarpTime, builtin_interfaces::msg::Time& ros2Time)
{
    uint64_t time;
    uint64_t sec_part;
    uint64_t nsec_part;
    sec_part = int(yarpTime); // (time / 1000000000UL);
    nsec_part = (yarpTime - sec_part)*1000000000UL;
    ros2Time.sec = sec_part;
    ros2Time.nanosec = (uint32_t)nsec_part;
}

void FrameTransformSet_nwc_ros2::yarpTransformToROS2Transform(const yarp::math::FrameTransform &input, geometry_msgs::msg::TransformStamped& output)
{
    builtin_interfaces::msg::Time tempStamp;
    std_msgs::msg::Header tempHeader;
    geometry_msgs::msg::Transform tempTf;
    geometry_msgs::msg::Vector3 tempTransl;
    geometry_msgs::msg::Quaternion tempRotation;

    tempTransl.x = input.translation.tX;
    tempTransl.y = input.translation.tY;
    tempTransl.z = input.translation.tZ;

    tempRotation.w = input.rotation.w();
    tempRotation.x = input.rotation.x();
    tempRotation.y = input.rotation.y();
    tempRotation.z = input.rotation.z();

    tempTf.rotation = tempRotation;
    tempTf.translation = tempTransl;

    ros2TimeFromYarp(input.isStatic ? yarp::os::Time::now() : input.timestamp,tempStamp);
    tempHeader.stamp = tempStamp;
    tempHeader.frame_id = input.src_frame_id;

    output.header = tempHeader;
    output.child_frame_id = input.dst_frame_id;
    output.transform = tempTf;
}

bool FrameTransformSet_nwc_ros2::publishFrameTransforms()
{
    if(!m_ftContainer.checkAndRemoveExpired())
    {
        yCError(FRAMETRANSFORMSETNWCROS2,"Unable to remove expired transforms");
        return false;
    }
    std::vector<yarp::math::FrameTransform> fromGet;
    m_ftContainer.getTransforms(fromGet);
    tf2_msgs::msg::TFMessage toPublish_timed;
    tf2_msgs::msg::TFMessage toPublish_static;
    std::vector<geometry_msgs::msg::TransformStamped> content_timed;
    std::vector<geometry_msgs::msg::TransformStamped> content_static;
    for(auto &ft : fromGet)
    {
        geometry_msgs::msg::TransformStamped tempTfs;
        yarpTransformToROS2Transform(ft,tempTfs);
        if(!ft.isStatic) {
            content_timed.push_back(tempTfs);
        }else {
            content_static.push_back(tempTfs);
        }
    }

    toPublish_timed.transforms = content_timed;
    toPublish_static.transforms = content_static;
    if (content_timed.size() > 0) {
        m_publisherFtTimed->publish(toPublish_timed);
    }
    if (content_static.size() > 0) {
        m_publisherFtStatic->publish(toPublish_static);
    }

    return true;
}

bool FrameTransformSet_nwc_ros2::deleteTransform(std::string t1, std::string t2)
{
    // Not yet implemented
    yCError(FRAMETRANSFORMSETNWCROS2, "deleteTransform not yet implemented");
    return false;
}

bool FrameTransformSet_nwc_ros2::clearAll()
{
    // Not yet implemented
    yCError(FRAMETRANSFORMSETNWCROS2, "clearAll not yet implemented");
    return false;
}
