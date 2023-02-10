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

#include "Imu_nwc_ros2.h"
#include <tf2/LinearMath/Quaternion.h>

YARP_LOG_COMPONENT(GENERICSENSOR_NWC_ROS2, "yarp.device.imu_nwc_ros2")

void Imu_nwc_ros2::subscription_callback(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);
    if(m_internalStatus == yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ) { m_internalStatus = yarp::dev::MAS_status::MAS_OK; }
    yCInfo(GENERICSENSOR_NWC_ROS2) << "Imu data received at " << yarpTimeFromRos2(msg->header.stamp);
    m_currentData.header = msg->header;
    m_currentData.angular_velocity = msg->angular_velocity;
    m_currentData.angular_velocity_covariance = msg->angular_velocity_covariance;
    m_currentData.linear_acceleration = msg->linear_acceleration;
    m_currentData.linear_acceleration_covariance = msg->linear_acceleration_covariance;
    m_currentData.orientation = msg->orientation;
    m_currentData.orientation_covariance = msg->orientation_covariance;
}

// IThreeAxisLinearAccelerometers ------------------------------------------------------------------------------------------------- START //
bool Imu_nwc_ros2::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);

    YARP_UNUSED(sens_index);
    if(m_internalStatus == yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ)
    {
        yCError(GENERICSENSOR_NWC_ROS2) << "No data received yet";
        return false;
    }
    yarp::sig::Vector accelerations(3);
    timestamp = yarpTimeFromRos2(m_currentData.header.stamp);

    accelerations[0] = m_currentData.linear_acceleration.x;
    accelerations[1] = m_currentData.linear_acceleration.y;
    accelerations[2] = m_currentData.linear_acceleration.z;

    out = accelerations;

    return true;
}

bool Imu_nwc_ros2::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const
{
    YARP_UNUSED(sens_index);
    name = m_sensorName;

    return true;
}

bool Imu_nwc_ros2::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);

    YARP_UNUSED(sens_index);
    if(m_internalStatus == yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ)
    {
        yCError(GENERICSENSOR_NWC_ROS2) << "No data received yet";
        return false;
    }
    frameName = m_currentData.header.frame_id;

    return true;
}

size_t Imu_nwc_ros2::getNrOfThreeAxisLinearAccelerometers() const
{
    return 1;
}

yarp::dev::MAS_status Imu_nwc_ros2::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return m_internalStatus;
}
// IThreeAxisLinearAccelerometers --------------------------------------------------------------------------------------------------- END //

// IThreeAxisGyroscopes ----------------------------------------------------------------------------------------------------------- START //
bool Imu_nwc_ros2::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);

    YARP_UNUSED(sens_index);
    if(m_internalStatus == yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ)
    {
        yCError(GENERICSENSOR_NWC_ROS2) << "No data received yet";
        return false;
    }
    yarp::sig::Vector angularVelocities(3);
    timestamp = yarpTimeFromRos2(m_currentData.header.stamp);

    angularVelocities[0] = m_currentData.angular_velocity.x * 180.0 / M_PI;
    angularVelocities[1] = m_currentData.angular_velocity.y * 180.0 / M_PI;
    angularVelocities[2] = m_currentData.angular_velocity.z * 180.0 / M_PI;

    out = angularVelocities;

    return true;
}

bool Imu_nwc_ros2::getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const
{
    YARP_UNUSED(sens_index);
    name = m_sensorName;

    return true;
}

bool Imu_nwc_ros2::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);

    YARP_UNUSED(sens_index);
    if(m_internalStatus == yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ)
    {
        yCError(GENERICSENSOR_NWC_ROS2) << "No data received yet";
        return false;
    }
    frameName = m_currentData.header.frame_id;

    return true;
}

size_t Imu_nwc_ros2::getNrOfThreeAxisGyroscopes() const
{
    return 1;
}

yarp::dev::MAS_status Imu_nwc_ros2::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return m_internalStatus;
}
// IThreeAxisGyroscopes ------------------------------------------------------------------------------------------------------------- END //

// IOrientationSensors ------------------------------------------------------------------------------------------------------------ START //
bool Imu_nwc_ros2::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);

    YARP_UNUSED(sens_index);
    if(m_internalStatus == yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ)
    {
        yCError(GENERICSENSOR_NWC_ROS2) << "No data received yet";
        return false;
    }
    yarp::sig::Vector orient(3);
    tf2Scalar roll, pitch, yaw;
    tf2::Quaternion tempQuat;
    tempQuat.setX(m_currentData.orientation.x);
    tempQuat.setY(m_currentData.orientation.y);
    tempQuat.setZ(m_currentData.orientation.z);
    tempQuat.setW(m_currentData.orientation.w);
    tf2::Matrix3x3 tempMat(tempQuat);
    tempMat.getRPY(roll, pitch, yaw);

    timestamp = yarpTimeFromRos2(m_currentData.header.stamp);

    orient[0] = roll * 180.0 / M_PI;
    orient[1] = pitch * 180.0 / M_PI;
    orient[2] = yaw * 180.0 / M_PI;

    rpy = orient;

    return true;
}

bool Imu_nwc_ros2::getOrientationSensorName(size_t sens_index, std::string &name) const
{
    YARP_UNUSED(sens_index);
    name = m_sensorName;

    return true;
}

bool Imu_nwc_ros2::getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);

    YARP_UNUSED(sens_index);
    if(m_internalStatus == yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ)
    {
        yCError(GENERICSENSOR_NWC_ROS2) << "No data received yet";
        return false;
    }
    frameName = m_currentData.header.frame_id;

    return true;
}

size_t Imu_nwc_ros2::getNrOfOrientationSensors() const
{
    return 1;
}

yarp::dev::MAS_status Imu_nwc_ros2::getOrientationSensorStatus(size_t sens_index) const
{
    return m_internalStatus;
}
// IOrientationSensors -------------------------------------------------------------------------------------------------------------- END //
