/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_MOBILEBASEVELOCITYCONTROL_NWS_ROS2
#define YARP_DEV_MOBILEBASEVELOCITYCONTROL_NWS_ROS2

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <Ros2Spinner.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/WrapperSingle.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

class Ros2InitMobVel
{
public:
    Ros2InitMobVel();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2InitMobVel& get();
};

 /**
  *  @ingroup dev_impl_nws_ros2 dev_impl_navigation
  *
  * \section MobileBaseVelocityControl_nws_ros2
  *
  * \brief `MobileBaseVelocityControl_nws_ros2`: A device which allows a client application to control the velocity of a mobile base from ros2.
  * The device opens a topic of type `::geometry_msgs::msg::Twist` to receive user commands
  *
  *  Parameters required by this device are:
  * | Parameter name | SubParameter   | Type    | Units          | Default Value                  | Required     | Description                                                       | Notes |
  * |:--------------:|:--------------:|:-------:|:--------------:|:------------------------------:|:------------:|:-----------------------------------------------------------------:|:-----:|
  * | node_name      |      -         | string  | -              | -                              | Yes           | Full name of the opened ros2 node                                |       |
  * | topic_name     |     -          | string  | -              | -                              | Yes           | Full name of the opened ros2 topic                               |       |
  */

class MobileBaseVelocityControl_nws_ros2 :
    public yarp::dev::DeviceDriver,
    public yarp::dev::WrapperSingle
{

public:
    MobileBaseVelocityControl_nws_ros2() = default;

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    bool detach() override;
    bool attach(yarp::dev::PolyDriver* driver) override;

private:
    // Spinner
    Ros2Spinner*                  m_spinner{nullptr};
    
    std::string                   m_node_name = "/mobileBase_VelControl_nws_ros2";
    std::string                   m_topic_name = "/velocity_input";
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_ros2_subscriber;
    rclcpp::Node::SharedPtr m_node;
    yarp::dev::Nav2D::INavigation2DVelocityActions* m_iNavVel = nullptr;
    yarp::dev::PolyDriver         m_subdev;
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

};

#endif // YARP_DEV_MOBILEBASEVELOCITYCONTROL_NWS_ROS2
