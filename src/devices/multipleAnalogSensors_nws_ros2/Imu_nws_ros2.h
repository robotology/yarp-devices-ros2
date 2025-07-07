/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_IMU_NWS_ROS2_H
#define YARP_DEV_IMU_NWS_ROS2_H

#include "GenericSensor_nws_ros2.h"
#include <sensor_msgs/msg/imu.hpp>

/**
 * @ingroup dev_impl_nws_ros2
 *
 * \brief `IMU_nws_ros2`: This wrapper connects to a device and publishes a ROS topic of type sensor_msgs::Imu.
 *
 * | YARP device name |
 * |:-----------------:|
 * | `Imu_nws_ros2` |
 *
 * The parameters accepted by this device are shown in class: GenericSensor_nws_ros2_ParamsParser
 *
 */
class Imu_nws_ros2 : public GenericSensor_nws_ros2<sensor_msgs::msg::Imu>
{
    // Interface of the wrapped device
    yarp::dev::IThreeAxisLinearAccelerometers* m_iThreeAxisLinearAccelerometers{ nullptr };
    yarp::dev::IThreeAxisGyroscopes*           m_iThreeAxisGyroscopes{ nullptr };
    yarp::dev::IOrientationSensors*            m_iOrientationSensors{ nullptr };
    yarp::dev::IThreeAxisMagnetometers*        m_iThreeAxisMagnetometers{ nullptr };

public:
    using GenericSensor_nws_ros2<sensor_msgs::msg::Imu>::GenericSensor_nws_ros2;

    using GenericSensor_nws_ros2<sensor_msgs::msg::Imu>::open;
    using GenericSensor_nws_ros2<sensor_msgs::msg::Imu>::close;
    using GenericSensor_nws_ros2<sensor_msgs::msg::Imu>::attachAll;
    using GenericSensor_nws_ros2<sensor_msgs::msg::Imu>::detachAll;

    /* PeriodicRateThread methods */
    void run() override;

protected:
    bool viewInterfaces() override;
};

#endif // YARP_DEV_IMU_NWS_ROS2_H
