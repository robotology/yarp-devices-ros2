/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "Localization2D_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <Ros2Utils.h>

#include <cmath>

using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;


YARP_LOG_COMPONENT(LOCALIZATION2D_NWS_ROS2, "yarp.ros2.localization2D_nws_ros2", yarp::os::Log::TraceType);


Localization2D_nws_ros2::Localization2D_nws_ros2() :
        yarp::os::PeriodicThread(0.01)
{
    m_stats_time_last = yarp::os::Time::now();
}

bool Localization2D_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    if (poly->isValid())
    {
        poly->view(m_iLoc);
    }

    //attach the hardware device
    if (nullptr == m_iLoc)
    {
        yCError(LOCALIZATION2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

   return true;
}

bool Localization2D_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_iLoc = nullptr;

    return true;
}

void Localization2D_nws_ros2::run()
{
    double m_stats_time_curr = yarp::os::Time::now();
    if (m_stats_time_curr - m_stats_time_last > 5.0)
    {
        yCInfo(LOCALIZATION2D_NWS_ROS2) << "Running";
        m_stats_time_last = yarp::os::Time::now();
    }

    if (m_iLoc!=nullptr)
    {
        bool ret = m_iLoc->getLocalizationStatus(m_current_status);
        if (!ret)
        {
            yCError(LOCALIZATION2D_NWS_ROS2) << "getLocalizationStatus() failed";
        }

        if (m_current_status == LocalizationStatusEnum::localization_status_localized_ok)
        {
            bool ret2 = m_iLoc->getCurrentPosition(m_current_position);
            if (!ret2)
            {
                yCError(LOCALIZATION2D_NWS_ROS2) << "getCurrentPosition() failed";
            }
            else
            {
                m_loc_stamp.update();
            }
            bool ret3 = m_iLoc->getEstimatedOdometry(m_current_odometry);
            if (!ret3)
            {
                //yCError(LOCALIZATION2D_NWS_ROS2) << "getEstimatedOdometry() failed";
            }
            else
            {
                m_odom_stamp.update();
            }
        }
        else
        {
            yCWarning(LOCALIZATION2D_NWS_ROS2, "The system is not properly localized!");
        }

        if (m_publisher_odom->get_subscription_count() > 0)
        {
            publish_odometry_on_ROS_topic();
        }
        publish_odometry_on_TF_topic();
    }
}

bool Localization2D_nws_ros2::open(yarp::os::Searchable &config)
{
    if(m_node_name[0] == '/'){
        yCError(LOCALIZATION2D_NWS_ROS2) << "node_name cannot begin with an initial /";
        return false;
    }

    parseParams(config);

    //create the topics
    const std::string m_odom_topic ="/odom";
    const std::string m_tf_topic ="/tf";
    m_node = NodeCreator::createNode(m_node_name);

    m_publisher_odom = m_node->create_publisher<nav_msgs::msg::Odometry>(m_odom_topic, 10);
    m_publisher_tf   = m_node->create_publisher<tf2_msgs::msg::TFMessage>(m_tf_topic, 10);
    yCInfo(LOCALIZATION2D_NWS_ROS2, "Opened topics: %s, %s", m_odom_topic.c_str(), m_tf_topic.c_str());

    yCInfo(LOCALIZATION2D_NWS_ROS2) << "Waiting for device to attach";

    //start the publishig thread
    setPeriod(m_period);
    start();
    return true;
}

bool Localization2D_nws_ros2::close()
{
    yCTrace(LOCALIZATION2D_NWS_ROS2);
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detach();
    return true;
}

void Localization2D_nws_ros2::publish_odometry_on_TF_topic()
{
    tf2_msgs::msg::TFMessage rosData;

    geometry_msgs::msg::TransformStamped tsData;
    tsData.child_frame_id = m_ROS_child_frame_id;
    tsData.header.frame_id = m_ROS_parent_frame_id;
    tsData.header.stamp = m_node->get_clock()->now();   //@@@@@@@@@@@ CHECK HERE: simulation time?
    double halfYaw = m_current_odometry.odom_theta / 180.0 * M_PI * 0.5;
    double cosYaw = cos(halfYaw);
    double sinYaw = sin(halfYaw);
    tsData.transform.rotation.x = 0;
    tsData.transform.rotation.y = 0;
    tsData.transform.rotation.z = sinYaw;
    tsData.transform.rotation.w = cosYaw;
    tsData.transform.translation.x = m_current_odometry.odom_x;
    tsData.transform.translation.y = m_current_odometry.odom_y;
    tsData.transform.translation.z = 0;

    if (rosData.transforms.size() == 0)
    {
        rosData.transforms.push_back(tsData);
    }
    else
    {
        rosData.transforms[0] = tsData;
    }

    m_publisher_tf->publish(rosData);
}

void Localization2D_nws_ros2::publish_odometry_on_ROS_topic()
{
    nav_msgs::msg::Odometry rosData;

    rosData.header.frame_id = m_fixed_frame;
    rosData.header.stamp = m_node->get_clock()->now();   //@@@@@@@@@@@ CHECK HERE: simulation time?
    rosData.child_frame_id = m_robot_frame;

    rosData.pose.pose.position.x = m_current_odometry.odom_x;
    rosData.pose.pose.position.y = m_current_odometry.odom_y;
    rosData.pose.pose.position.z = 0;
    yarp::sig::Vector vecrpy(3);
    vecrpy[0] = 0;
    vecrpy[1] = 0;
    vecrpy[2] = m_current_odometry.odom_theta;
    yarp::sig::Matrix matrix = yarp::math::rpy2dcm(vecrpy);
    yarp::math::Quaternion q; q.fromRotationMatrix(matrix);
    rosData.pose.pose.orientation.x = q.x();
    rosData.pose.pose.orientation.y = q.y();
    rosData.pose.pose.orientation.z = q.z();
    rosData.pose.pose.orientation.w = q.w();
    //rosData.pose.covariance = 0;

    rosData.twist.twist.linear.x = m_current_odometry.base_vel_x;
    rosData.twist.twist.linear.y = m_current_odometry.base_vel_y;
    rosData.twist.twist.linear.z = 0;
    rosData.twist.twist.angular.x = 0;
    rosData.twist.twist.angular.y = 0;
    rosData.twist.twist.angular.z = m_current_odometry.base_vel_theta;
    //rosData.twist.covariance = 0;

    m_publisher_odom->publish(rosData);
}