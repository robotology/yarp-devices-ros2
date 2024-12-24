/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_ODOMETRY2D_NWS_ROS2_H
#define YARP_ROS2_ODOMETRY2D_NWS_ROS2_H
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <yarp/dev/IOdometry2D.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <tf2_msgs/msg/tf_message.hpp>

#include "Odometry2D_nws_ros2_ParamsParser.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#define DEG2RAD M_PI/180.0
#define DEFAULT_THREAD_PERIOD 0.02 //s

/**
 * @ingroup dev_impl_nws_ros2 dev_impl_navigation
 *
 * \section Odometry2D_nws_ros2_parameters Device description
 * \brief `Odometry2D_nws_ros2`: A ros2 nws to get odometry and publish it on a ros2 topic.
 * The attached device must implement a `yarp::dev::Nav2D::IOdometry2D` interface.
 *
 * Parameters required by this device are shown in class: `Odometry2D_nws_ros2_ParamsParser`.
 *
 * Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device odometry2D_nws_ros2
 * period 0.02
 * node_name odometry_ros2
 * topic_name /odometry
 * odom_frame odom
 * base_frame base
 *
 * subdevice fakeOdometry
 * \endcode
 *
 * example of xml file with a fake odometer
 *
 * \code{.unparsed}
 * <?xml version="1.0" encoding="UTF-8" ?>
 * <!DOCTYPE robot PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">
 * <robot name="fakeOdometry" build="2" portprefix="test" xmlns:xi="http://www.w3.org/2001/XInclude">
 *   <devices>
 *     <device xmlns:xi="http://www.w3.org/2001/XInclude" name="fakeOdometry_device" type="fakeOdometry">
 *     </device>
 *     <device xmlns:xi="http://www.w3.org/2001/XInclude" name="odometry2D_nws_ros2" type="odometry2D_nws_ros2">
 *       <param name="node_name"> odometry_ros2 </param>
 *       <param name="topic_name"> /odometry </param>
 *       <param name="odom_frame">odom</param>
 *       <param name="base_frame">base</param>
 *       <action phase="startup" level="5" type="attach">
 *         <paramlist name="networks">
 *           <elem name="subdevice_odometry"> fakeOdometry_device </elem>
 *         </paramlist>
 *       </action>
 *       <action phase="shutdown" level="5" type="detach" />
 *     </device>
 *   </devices>
 * </robot>
 * \endcode
 *
 * example of command via terminal.
 *
 * \code{.unparsed}
 * yarpdev --device odometry2D_nws_ros2 --node_name odometry_ros2 --topic_name /odometry --odom_frame odom --base_frame base --subdevice fakeOdometry
 * \endcode
 */


class Odometry2D_nws_ros2 :
        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        Odometry2D_nws_ros2_ParamsParser
{
public:
    Odometry2D_nws_ros2();
    ~Odometry2D_nws_ros2();

    // DeviceDriver
    bool open(yarp::os::Searchable &params) override;
    bool close() override;

    // WrapperSingle
    bool attach(yarp::dev::PolyDriver* driver) override;
    bool detach() override;

    // PeriodicThread
    bool threadInit() override;
    void threadRelease() override;
    void run() override;


private:
    // stamp count for timestamp
    yarp::os::Stamp m_timeStamp;

    //ros2 node
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr   m_ros2Publisher_odometry;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr  m_publisher_tf;

    //interfaces
    yarp::dev::PolyDriver m_driver;
    yarp::dev::Nav2D::IOdometry2D *m_odometry2D_interface{nullptr};
};

#endif // YARP_ROS2_ODOMETRY2D_NWS_ROS2_H
