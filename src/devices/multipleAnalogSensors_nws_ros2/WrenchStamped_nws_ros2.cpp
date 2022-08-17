/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "WrenchStamped_nws_ros2.h"

YARP_LOG_COMPONENT(GENERICSENSOR_NWS_ROS2, "yarp.device.WrenchStamped_nws_ros2")

bool WrenchStamped_nws_ros2::viewInterfaces()
{
    // View all the interfaces
    bool ok = m_poly->view(m_iFTsens);
    m_iFTsens->getSixAxisForceTorqueSensorFrameName(m_sens_index, m_framename);
    return ok;
}

void WrenchStamped_nws_ros2::run()
{
    if (m_publisher)
    {
        yarp::sig::Vector vecwrench(6);
        geometry_msgs::msg::WrenchStamped wrench_ros_data;
        m_iFTsens->getSixAxisForceTorqueSensorMeasure(m_sens_index, vecwrench, m_timestamp);
        wrench_ros_data.header.frame_id = m_framename;
        wrench_ros_data.header.stamp = ros2TimeFromYarp(m_timestamp);
        wrench_ros_data.wrench.force.x = vecwrench[0];
        wrench_ros_data.wrench.force.y = vecwrench[1];
        wrench_ros_data.wrench.force.z = vecwrench[2];
        wrench_ros_data.wrench.torque.x = vecwrench[4];
        wrench_ros_data.wrench.torque.y = vecwrench[5];
        wrench_ros_data.wrench.torque.z = vecwrench[6];
        m_publisher->publish(wrench_ros_data);
    }
}
