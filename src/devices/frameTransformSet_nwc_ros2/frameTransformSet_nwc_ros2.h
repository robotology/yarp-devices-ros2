/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_FRAMETRANSFORMSETNWCROS2_H
#define YARP_DEV_FRAMETRANSFORMSETNWCROS2_H


#include <yarp/os/Network.h>
#include <yarp/dev/IFrameTransformStorage.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <FrameTransformContainer.h>
#include <Ros2Utils.h>
#include <mutex>
#include <map>

#define ROS2NODENAME "tfNodeSet"
#define ROS2TOPICNAME_TF "/tf"
#define ROS2TOPICNAME_TF_STATIC "/tf_static"

/**
 * @ingroup dev_impl_nwc_ros2
 *
 * @brief `frameTransformSet_nwc_ros2`: A ros network wrapper client that receives frame transforms from a ros2 topic and makes them available through an IFrameTransformStorageGet interface. See \subpage FrameTransform for additional info.
 *
 * \section FrameTransformSet_nwc_ros2_device_parameters Parameters
 *
 *   Parameters required by this device are:
 * | Parameter name | SubParameter         | Type    | Units          | Default Value         | Required     | Description                                    -------            |
 * |:--------------:|:--------------------:|:-------:|:--------------:|:---------------------:|:-----------: |:-----------------------------------------------------------------:|
 * | GENERAL        |      -               | group   | -              | -                     | No           |                                                                   |
 * | -              | refresh_interval     | double  | seconds        | 0.1                   | No           | The time interval outside which timed ft will be deleted          |
 * | ROS2           |      -               | group   | -              | -                     | No           |                                                                   |
 * | -              | ft_node              | string  | -              | /tfNodeGet            | No           | The of the ROS2 node                                              |
 * | -              | ft_topic             | string  | -              | /tf                   | No           | The name of the ROS2 topic from which fts will be received        |
 * | -              | ft_topic_static      | string  | -              | /tf_static            | No           | The name of the ROS2 topic from which static fts will be received |

 * **N.B.** pay attention to the difference between **tf** and **ft**
 *
 * \section FrameTransformSet_nwc_ros2_device_example Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device frameTransformSet_nwc_ros2
 * [GENERAL]
 * period 0.05
 * refresh_interval 0.2
 * [ROS]
 * ft_topic /tf
 * ft_topic_static /tf_static
 * ft_node /tfNodeGet
 * \endcode
 */


class FrameTransformSet_nwc_ros2 :
    public yarp::dev::DeviceDriver,
    public yarp::os::PeriodicThread,
    public yarp::dev::IFrameTransformStorageSet
{
public:
    FrameTransformSet_nwc_ros2(double tperiod=0.010);
    ~FrameTransformSet_nwc_ros2()=default;

    //DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    //periodicThread
    void run() override;

    //IFrameTransformStorageSet interface
    bool setTransforms(const std::vector<yarp::math::FrameTransform>& transforms) override;
    bool setTransform(const yarp::math::FrameTransform& transform) override;
    bool deleteTransform(std::string t1, std::string t2) override;
    bool clearAll() override;

    //own
    bool publishFrameTransforms();
    void ros2TimeFromYarp(double yarpTime, builtin_interfaces::msg::Time& ros2Time);
    void yarpTransformToROS2Transform(const yarp::math::FrameTransform &input, geometry_msgs::msg::TransformStamped& output);

private:
    mutable std::mutex                                                  m_trf_mutex;
    std::string                                                         m_ftNodeName{ROS2NODENAME};
    std::string                                                         m_ftTopic{ROS2TOPICNAME_TF};
    std::string                                                         m_ftTopicStatic{ROS2TOPICNAME_TF};
    double                                                              m_period;
    double                                                              m_refreshInterval{0.1};
    bool                                                                m_asynchPub{true};
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr              m_publisherFtTimed;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr              m_publisherFtStatic;
    FrameTransformContainer                                             m_ftContainer;
    rclcpp::Node::SharedPtr                                             m_node;
};

#endif // YARP_DEV_FRAMETRANSFORMSETNWCROS2_H
