/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// We define and include this first to ensure
// that no transitivetely-included header already
// includes cmath withut defining _USE_MATH_DEFINES
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <cmath>

#include "Imu_nws_ros2.h"
#include <tf2/LinearMath/Quaternion.h>

YARP_LOG_COMPONENT(GENERICSENSOR_NWS_ROS2, "yarp.device.Imu_nws_ros2")

bool Imu_nws_ros2::viewInterfaces()
{
    // View all the interfaces
    bool ok = true;
    ok = m_poly->view(m_iThreeAxisGyroscopes);
    if (!ok) {
        yCError(GENERICSENSOR_NWS_ROS2) << "IThreeAxisGyroscopes interface is not available";
        return false;
    }
    ok = m_poly->view(m_iThreeAxisLinearAccelerometers);
    if (!ok) {
        yCError(GENERICSENSOR_NWS_ROS2) << "IThreeAxisLinearAccelerometers interface is not available";
        return false;
    }
    ok = m_poly->view(m_iThreeAxisMagnetometers);
    if (!ok) {
        yCError(GENERICSENSOR_NWS_ROS2) << "IThreeAxisMagnetometers interface is not available";
        return false;
    }
    ok = m_poly->view(m_iOrientationSensors);
    if (!ok) {
        yCError(GENERICSENSOR_NWS_ROS2) << "IOrientationSensors interface is not available";
        return false;
    }

    ok = m_iThreeAxisGyroscopes->getThreeAxisGyroscopeFrameName(m_sens_index, m_framename);
    return ok;
}

void Imu_nws_ros2::run()
{
    if (m_publisher)
    {
        yarp::sig::Vector vecgyr(3);
        yarp::sig::Vector vecacc(3);
        yarp::sig::Vector vecrpy(3);
        tf2::Quaternion quat;
        quat.setRPY(vecrpy[0] * M_PI / 180.0, vecrpy[1] * M_PI / 180.0, vecrpy[2] * M_PI / 180.0);
        quat.normalize();
        sensor_msgs::msg::Imu imu_ros_data;
        m_iThreeAxisGyroscopes->getThreeAxisGyroscopeMeasure(m_sens_index, vecgyr, m_timestamp);
        m_iThreeAxisLinearAccelerometers->getThreeAxisLinearAccelerometerMeasure(m_sens_index, vecacc, m_timestamp);
        m_iOrientationSensors->getOrientationSensorMeasureAsRollPitchYaw(m_sens_index, vecrpy, m_timestamp);
        imu_ros_data.header.frame_id = m_framename;
        imu_ros_data.header.stamp = ros2TimeFromYarp(m_timestamp);
        imu_ros_data.angular_velocity.x = vecgyr[0] * M_PI / 180.0;
        imu_ros_data.angular_velocity.y = vecgyr[1] * M_PI / 180.0;
        imu_ros_data.angular_velocity.z = vecgyr[2] * M_PI / 180.0;
        imu_ros_data.linear_acceleration.x = vecacc[0];
        imu_ros_data.linear_acceleration.y = vecacc[1];
        imu_ros_data.linear_acceleration.z = vecacc[2];
        imu_ros_data.orientation.x = quat.x();
        imu_ros_data.orientation.y = quat.y();
        imu_ros_data.orientation.z = quat.z();
        imu_ros_data.orientation.z = quat.w();
        m_publisher->publish(imu_ros_data);
    }
}
