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
    // View mandatory interfaces
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
    ok = m_poly->view(m_iOrientationSensors);
    if (!ok) {
        yCError(GENERICSENSOR_NWS_ROS2) << "IOrientationSensors interface is not available";
        return false;
    }

    // View optional interfaces
    bool mag_ok = m_poly->view(m_iThreeAxisMagnetometers);
    if (mag_ok)
    {
        std::string m_publisherMagName = m_publisherName + "_magnetic_field";
        yCInfo(GENERICSENSOR_NWS_ROS2) << "m_iThreeAxisMagnetometers interface available. Opening topic:" << m_publisherMagName;
        m_publisher_mag = m_node->create_publisher<sensor_msgs::msg::MagneticField>(m_publisherMagName,rclcpp::QoS(10));
    }
    else
    {
        yCWarning(GENERICSENSOR_NWS_ROS2) << "m_iThreeAxisMagnetometers interface is not available";
    }

    // Get frame Name
    ok = m_iThreeAxisGyroscopes->getThreeAxisGyroscopeFrameName(m_sens_index, m_framename);
    if (!ok) {
        yCError(GENERICSENSOR_NWS_ROS2) << "getThreeAxisGyroscopeFrameName() failed";
        return false;
    }

    return true;
}

void Imu_nws_ros2::run()
{
    if (m_publisher)
    {
        yarp::sig::Vector vecgyr(3);
        yarp::sig::Vector vecacc(3);
        yarp::sig::Vector vecrpy(3);

        m_iThreeAxisGyroscopes->getThreeAxisGyroscopeMeasure(m_sens_index, vecgyr, m_timestamp);
        m_iThreeAxisLinearAccelerometers->getThreeAxisLinearAccelerometerMeasure(m_sens_index, vecacc, m_timestamp);
        m_iOrientationSensors->getOrientationSensorMeasureAsRollPitchYaw(m_sens_index, vecrpy, m_timestamp);

        tf2::Quaternion quat;
        quat.setRPY(vecrpy[0] * M_PI / 180.0, vecrpy[1] * M_PI / 180.0, vecrpy[2] * M_PI / 180.0);
        quat.normalize();

        sensor_msgs::msg::Imu imu_ros_data;
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
        imu_ros_data.orientation.w = quat.w();
        m_publisher->publish(imu_ros_data);

        if (m_iThreeAxisMagnetometers)
        {
           yarp::sig::Vector vecmag(3);
           m_iThreeAxisMagnetometers->getThreeAxisMagnetometerMeasure(m_sens_index, vecmag, m_timestamp);
           sensor_msgs::msg::MagneticField mag_ros_data;
           mag_ros_data.header.frame_id = m_framename;
           mag_ros_data.header.stamp = ros2TimeFromYarp(m_timestamp);
           mag_ros_data.magnetic_field.x=vecmag[0];
           mag_ros_data.magnetic_field.y=vecmag[1];
           mag_ros_data.magnetic_field.z=vecmag[2];
           m_publisher_mag->publish(mag_ros_data);
        }
    }
}
