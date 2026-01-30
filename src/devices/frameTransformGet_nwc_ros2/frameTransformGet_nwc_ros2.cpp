/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "frameTransformGet_nwc_ros2.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std::placeholders;

namespace {
YARP_LOG_COMPONENT(FRAMETRANSFORGETNWCROS2, "yarp.device.frameTransformGet_nwc_ros2")
}

bool FrameTransformGet_nwc_ros2::open(yarp::os::Searchable& config)
{
    if (!yarp::os::NetworkBase::checkNetwork()) {
        yCError(FRAMETRANSFORGETNWCROS2,"Error! YARP Network is not initialized");
        return false;
    }
    parseParams(config);

    m_ftContainer.m_timeout = m_GENERAL_refresh_interval;
    if(m_ROS2_namespace.empty())
    {
        m_node = NodeCreator::createNode(m_ROS2_ft_node);
    } else {
        m_node = NodeCreator::createNode(m_ROS2_ft_node, m_ROS2_namespace);
    }
    if(m_node == nullptr) {
        yCError(FRAMETRANSFORGETNWCROS2) << " opening " << m_ROS2_ft_node << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }
    m_subscriptionFtTimed = m_node->create_subscription<tf2_msgs::msg::TFMessage>(m_ROS2_ft_topic, 10,
                                                                                  std::bind(&FrameTransformGet_nwc_ros2::frameTransformTimedGet_callback,
                                                                                  this, _1));

    rclcpp::QoS qos(10);
    qos = qos.transient_local(); // This line and the previous are needed to reestablish the "latched" behaviour for the subscriptio
    m_subscriptionFtStatic = m_node->create_subscription<tf2_msgs::msg::TFMessage>(m_ROS2_ft_topic_static, qos,
                                                                                   std::bind(&FrameTransformGet_nwc_ros2::frameTransformStaticGet_callback,
                                                                                   this, _1));

    m_spinner = new Ros2Spinner(m_node);
    m_spinner->start();

    yCInfo(FRAMETRANSFORGETNWCROS2) << "opened";

    return true;
}

bool FrameTransformGet_nwc_ros2::close()
{
    yCInfo(FRAMETRANSFORGETNWCROS2, "closing...");
    delete m_spinner;
    yCInfo(FRAMETRANSFORGETNWCROS2, "closed");
    return true;
}

yarp::dev::ReturnValue FrameTransformGet_nwc_ros2::getTransforms(std::vector<yarp::math::FrameTransform>& transforms) const
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    if(!m_ftContainer.checkAndRemoveExpired())
    {
        yCError(FRAMETRANSFORGETNWCROS2,"Unable to remove expired transforms.");
        return yarp::dev::ReturnValue::return_code::return_value_error_generic;
    }
    auto ret = m_ftContainer.getTransforms(transforms);
    if(!ret)
    {
        yCError(FRAMETRANSFORGETNWCROS2,"Unable to get transforms. Error: %s",ret.toString().c_str());
    }
    return ret;
}

void FrameTransformGet_nwc_ros2::frameTransformTimedGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    yCTrace(FRAMETRANSFORGETNWCROS2);
    updateBuffer(msg->transforms,false);
}

void FrameTransformGet_nwc_ros2::frameTransformStaticGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_trf_mutex);
    yCTrace(FRAMETRANSFORGETNWCROS2);
    updateBuffer(msg->transforms,true);
}

void FrameTransformGet_nwc_ros2::ros2TransformToYARP(const geometry_msgs::msg::TransformStamped& input, yarp::math::FrameTransform& output, bool isStatic)
{
    output.translation.tX = input.transform.translation.x;
    output.translation.tY = input.transform.translation.y;
    output.translation.tZ = input.transform.translation.z;

    output.rotation.w() = input.transform.rotation.w;
    output.rotation.x() = input.transform.rotation.x;
    output.rotation.y() = input.transform.rotation.y;
    output.rotation.z() = input.transform.rotation.z;

    output.timestamp = yarpTimeFromRos2(input.header.stamp);
    output.src_frame_id = input.header.frame_id;
    output.dst_frame_id = input.child_frame_id;
    output.isStatic = isStatic;
}

bool FrameTransformGet_nwc_ros2::updateBuffer(const std::vector<geometry_msgs::msg::TransformStamped>& transforms, bool areStatic)
{
    for (auto& it : transforms)
    {
        yarp::math::FrameTransform tempFT;
        ros2TransformToYARP(it,tempFT,areStatic);
        m_ftContainer.setTransform(tempFT);
    }
    return true;
}
