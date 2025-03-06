/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_FRAMETRANSFORMGETNWCROS2_H
#define YARP_DEV_FRAMETRANSFORMGETNWCROS2_H


#include <yarp/os/Network.h>
#include <yarp/dev/IFrameTransformStorage.h>
#include <yarp/sig/Vector.h>
#include <Ros2Spinner.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <yarp/dev/FrameTransformContainer.h>
#include <Ros2Utils.h>
#include <mutex>
#include <map>
#include "FrameTransformGet_nwc_ros2_ParamsParser.h"

#define ROS2NODENAME "tfNodeGet"
#define ROS2TOPICNAME_TF "/tf"
#define ROS2TOPICNAME_TF_STATIC "/tf_static"



/**
 * @ingroup dev_impl_nwc_ros2
 *
 * @brief `frameTransformGet_nwc_ros2`: A ros network wrapper client that receives frame transforms from a ros2 topic and makes them available through an IFrameTransformStorageGet interface. See \subpage FrameTransform for additional info.
 *
 * \section FrameTransformGet_nwc_ros2_device_parameters Parameters
 *
 *   Parameters required by this device are shown in class: FrameTransformGet_nwc_ros2_ParamsParser
 *
 * **N.B.** pay attention to the difference between **tf** and **ft**
 *
 * \section FrameTransformGet_nwc_ros2_device_example Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device frameTransformGet_nwc_ros2
 * [GENERAL]
 * period 0.05
 * refresh_interval 0.2
 * [ROS]
 * ft_topic /tf
 * ft_topic_static /tf_static
 * ft_node /tfNodeGet
 * \endcode
 */


class FrameTransformGet_nwc_ros2 :
    public yarp::dev::DeviceDriver,
    public yarp::dev::IFrameTransformStorageGet,
    FrameTransformGet_nwc_ros2_ParamsParser
{
public:
    ~FrameTransformGet_nwc_ros2()=default;

    //DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // Spinner
    Ros2Spinner*            m_spinner{nullptr};

    //IFrameTransformStorageGet interface
    yarp::dev::ReturnValue getTransforms(std::vector<yarp::math::FrameTransform>& transforms) const override;

    //Subscription callback
    void frameTransformTimedGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void frameTransformStaticGet_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    //own
    void ros2TransformToYARP(const geometry_msgs::msg::TransformStamped& input, yarp::math::FrameTransform& output, bool isStatic);
    bool updateBuffer(const std::vector<geometry_msgs::msg::TransformStamped>& transforms, bool areStatic);
    bool setTransform(const yarp::math::FrameTransform& transform);

private:
    mutable std::mutex                                                    m_trf_mutex;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr             m_subscriptionFtTimed;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr             m_subscriptionFtStatic;
    rclcpp::Node::SharedPtr                                               m_node;
    yarp::dev::FrameTransformContainer                                    m_ftContainer;
};

#endif // YARP_DEV_FRAMETRANSFORMGETNWCROS2_H
