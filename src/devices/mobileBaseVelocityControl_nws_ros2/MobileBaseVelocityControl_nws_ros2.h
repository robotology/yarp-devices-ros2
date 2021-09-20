/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#ifndef YARP_DEV_MOBILEBASEVELOCITYCONTROL_NWS_ROS2
#define YARP_DEV_MOBILEBASEVELOCITYCONTROL_NWS_ROS2

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/WrapperSingle.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

 /**
  *  @ingroup dev_impl_network_clients dev_impl_navigation
  *
  * \section MobileBaseVelocityControl_nws_ros2
  *
  * \brief `MobileBaseVelocityControl_nws_ros2`: A device which allows a client application to control the velocity of a mobile base from ros2.
  * The device opens a topic of type `::geometry_msgs::msg::Twist` to receive user commands
  *
  *  Parameters required by this device are:
  * | Parameter name | SubParameter   | Type    | Units          | Default Value                  | Required     | Description                                                       | Notes |
  * |:--------------:|:--------------:|:-------:|:--------------:|:------------------------------:|:------------:|:-----------------------------------------------------------------:|:-----:|
  * | node_name      |      -         | string  | -              | /mobileBase_VelControl_nws_ros2 | No           | Full name of the opened ros2 node                                  |       |
  * | topic_name     |     -          | string  | -              | /velocity_input                | No           | Full name of the opened ros2 topic                                 |       |
  */

 class Ros2InitMobVel
 {
 public:
     Ros2InitMobVel();

     std::shared_ptr<rclcpp::Node> node;

     static Ros2InitMobVel& get();
 };

class MobileBaseVelocityControl_nws_ros2 :
    public yarp::dev::DeviceDriver,
    public yarp::os::Thread,
    public yarp::dev::WrapperSingle
{

protected:
    std::string                   m_ros2_node_name = "/mobileBase_VelControl_nws_ros2";
    std::string                   m_ros2_topic_name = "/velocity_input";
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_ros2_subscriber;
    rclcpp::Node::SharedPtr m_node;
    yarp::dev::Nav2D::INavigation2DVelocityActions* m_iNavVel = nullptr;

public:
    MobileBaseVelocityControl_nws_ros2() = default;

    /* DeviceDriver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

private:
    bool detach() override;
    bool attach(yarp::dev::PolyDriver* driver) override;
    void run() override;
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

};

#endif // YARP_DEV_MOBILEBASEVELOCITYCONTROL_NWS_ROS2
