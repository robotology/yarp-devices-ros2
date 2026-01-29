/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// For M_PI
#define _USE_MATH_DEFINES
#include "Odometry2D_nws_ros2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <cmath>
#include <Ros2Utils.h>
#include <yarp/dev/OdometryData.h>

YARP_LOG_COMPONENT(ODOMETRY2D_NWS_ROS2, "yarp.devices.Odometry2D_nws_ros2")

Odometry2D_nws_ros2::Odometry2D_nws_ros2() : yarp::os::PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}

Odometry2D_nws_ros2::~Odometry2D_nws_ros2()
{
    m_odometry2D_interface = nullptr;
}


bool Odometry2D_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{

    if (driver->isValid())
    {
        driver->view(m_odometry2D_interface);
    } else {
        yCError(ODOMETRY2D_NWS_ROS2) << "not valid driver";
    }

    if (m_odometry2D_interface == nullptr)
    {
        yCError(ODOMETRY2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    yCInfo(ODOMETRY2D_NWS_ROS2, "Attach complete");
    PeriodicThread::setPeriod(m_period);
    return PeriodicThread::start();
}


bool Odometry2D_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_odometry2D_interface = nullptr;
    return true;
}

bool Odometry2D_nws_ros2::threadInit()
{
    return true;
}

bool Odometry2D_nws_ros2::open(yarp::os::Searchable &config)
{
    parseParams(config);
   if (m_node_name[0] == '/') {
        yCError(ODOMETRY2D_NWS_ROS2) << "node_name parameter cannot begin with '/'";
        return false;
    }
    if (m_topic_name[0] != '/') {
        yCError(ODOMETRY2D_NWS_ROS2) << "missing initial / in topic_name parameter";
        return false;
    }

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    if(m_namespace.empty()) {
        m_node = NodeCreator::createNode(m_node_name, node_options);
    } else {
        m_node = NodeCreator::createNode(m_node_name, m_namespace, node_options);
    }
    if (m_node == nullptr) {
        yCError(ODOMETRY2D_NWS_ROS2) << " opening " << m_node_name << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }

    rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue( true ) );
    m_node->set_parameter( simTime );
    const std::string m_tf_topic ="/tf";
    m_publisher_tf   = m_node->create_publisher<tf2_msgs::msg::TFMessage>(m_tf_topic, 10);

    m_ros2Publisher_odometry = m_node->create_publisher<nav_msgs::msg::Odometry>(m_topic_name, 10);

    yCInfo(ODOMETRY2D_NWS_ROS2) << "Waiting for device to attach";
    return true;
}

void Odometry2D_nws_ros2::threadRelease()
{
}

void Odometry2D_nws_ros2::run()
{

    if (m_odometry2D_interface!=nullptr && m_ros2Publisher_odometry && m_publisher_tf) {
        yarp::dev::OdometryData odometryData;
        double synchronized_timestamp = 0;
        m_odometry2D_interface->getOdometry(odometryData, &synchronized_timestamp);

        if (std::isnan(synchronized_timestamp) == false)
        {
            m_timeStamp.update(synchronized_timestamp);
        }
        else
        {
            m_timeStamp.update(yarp::os::Time::now());
        }

        nav_msgs::msg::Odometry odometryMsg;
        odometryMsg.header.frame_id = m_odom_frame;
        odometryMsg.child_frame_id = m_base_frame;

        odometryMsg.pose.pose.position.x = odometryData.odom_x;
        odometryMsg.pose.pose.position.y = odometryData.odom_y;
        odometryMsg.pose.pose.position.z = 0.0;
        geometry_msgs::msg::Quaternion odom_quat;
        double halfYaw = odometryData.odom_theta * DEG2RAD * 0.5;
        double cosYaw = cos(halfYaw);
        double sinYaw = sin(halfYaw);
        odom_quat.x = 0;
        odom_quat.y = 0;
        odom_quat.z = sinYaw;
        odom_quat.w = cosYaw;
        odometryMsg.pose.pose.orientation = odom_quat;
        odometryMsg.twist.twist.linear.x = odometryData.base_vel_x;
        odometryMsg.twist.twist.linear.y = odometryData.base_vel_y;
        odometryMsg.twist.twist.linear.z = 0;
        odometryMsg.twist.twist.angular.x = 0;
        odometryMsg.twist.twist.angular.y = 0;
        odometryMsg.twist.twist.angular.z = odometryData.base_vel_theta * DEG2RAD;

        // tf publisher
        tf2_msgs::msg::TFMessage rosData;

        geometry_msgs::msg::TransformStamped tsData;
        tsData.child_frame_id = m_base_frame;
        tsData.header.frame_id = m_odom_frame;

        tsData.transform.rotation.x = 0;
        tsData.transform.rotation.y = 0;
        tsData.transform.rotation.z = sinYaw;
        tsData.transform.rotation.w = cosYaw;
        tsData.transform.translation.x = odometryData.odom_x;
        tsData.transform.translation.y = odometryData.odom_y;
        tsData.transform.translation.z = 0;

        odometryMsg.header.stamp.sec = int(m_timeStamp.getTime());
        odometryMsg.header.stamp.nanosec = int(1000000000UL * (m_timeStamp.getTime() - int(m_timeStamp.getTime())));

        tsData.header.stamp.sec = int(m_timeStamp.getTime());
        tsData.header.stamp.nanosec = int(1000000000UL * (m_timeStamp.getTime() - int(m_timeStamp.getTime())));

        if (rosData.transforms.size() == 0)
        {
            rosData.transforms.push_back(tsData);
        }
        else if (rosData.transforms.size() == 1)
        {
            rosData.transforms[0] = tsData;
        }
        else
        {
            yCWarning(ODOMETRY2D_NWS_ROS2) << "Size of /tf topic should be 1, instead it is:" << rosData.transforms.size();
        }

        if(m_ros2Publisher_odometry->get_subscription_count() > 0)
            m_ros2Publisher_odometry->publish(odometryMsg);

        m_publisher_tf->publish(rosData);


    } else{
        yCError(ODOMETRY2D_NWS_ROS2) << "the interface is not valid";
    }
}

bool Odometry2D_nws_ros2::close()
{
    yCTrace(ODOMETRY2D_NWS_ROS2);
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detach();
    return true;
}
