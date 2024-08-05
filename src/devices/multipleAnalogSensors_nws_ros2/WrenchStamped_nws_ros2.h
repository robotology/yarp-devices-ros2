/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_WRENCHSTAMPED_NWS_ROS2_H
#define YARP_DEV_WRENCHSTAMPED_NWS_ROS2_H

#include "GenericSensor_nws_ros2.h"
#include <geometry_msgs/msg/wrench_stamped.hpp>

    /**
 * @ingroup dev_impl_wrapper
 *
 * \brief `WrenchStamped_nws_ros2`: This wrapper connects to a device and publishes a ROS topic of type geometry_msgs::WrenchStamped.
 *
 * | YARP device name |
 * |:-----------------:|
 * | `WrenchStamped_nws_ros2` |
 *
 * The parameters accepted by this device are shown in class: GenericSensor_nws_ros2_ParamsParser
 *
 */
class WrenchStamped_nws_ros2 : public GenericSensor_nws_ros2<geometry_msgs::msg::WrenchStamped>
{
    // Interface of the wrapped device
    yarp::dev::ISixAxisForceTorqueSensors* m_iFTsens{ nullptr };

public:
    using GenericSensor_nws_ros2<geometry_msgs::msg::WrenchStamped>::GenericSensor_nws_ros2;

    using GenericSensor_nws_ros2<geometry_msgs::msg::WrenchStamped>::open;
    using GenericSensor_nws_ros2<geometry_msgs::msg::WrenchStamped>::close;
    using GenericSensor_nws_ros2<geometry_msgs::msg::WrenchStamped>::attachAll;
    using GenericSensor_nws_ros2<geometry_msgs::msg::WrenchStamped>::detachAll;

    /* PeriodicRateThread methods */
    void run() override;

protected:
    bool viewInterfaces() override;
};

#endif
