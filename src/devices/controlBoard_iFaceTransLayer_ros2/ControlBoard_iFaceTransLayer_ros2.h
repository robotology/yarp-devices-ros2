/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


#ifndef YARP_ROS2_CONTROLBOARD_IFACETRANSLAYER_ROS2_H
#define YARP_ROS2_CONTROLBOARD_IFACETRANSLAYER_ROS2_H
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex>
#include <map>

#include <yarp/dev/IOdometry2D.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Property.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>

//Custom ros2 interfaces
#include <yarp_control_msgs/srv/get_control_modes.hpp>
#include <yarp_control_msgs/srv/set_control_modes.hpp>
#include <yarp_control_msgs/srv/get_available_control_modes.hpp>
#include <yarp_control_msgs/srv/get_joints_names.hpp>
#include <yarp_control_msgs/msg/position.hpp>
#include <yarp_control_msgs/msg/velocity.hpp>
#include <yarp_control_msgs/msg/position_direct.hpp>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif


/**
 *  @ingroup dev_impl_wrapper
 *
 * \brief `controlBoard_iFaceTransLayer_ros2`: A device to "translate" the controlBoard interface into ROS2 topics and esrvices.
 *
 * \section controlBoard_iFaceTransLayer_ros2_parameters Description of input parameters
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | name           |      -         | string  | -              |   -           | Yes                         | Device name used to create topics, services and node namse        | must not start with a leading '/' |
 * | subdevice      |      -         | string  | -              |   -           | No                          | name of the subdevice to instantiate                              | when used, parameters for the subdevice must be provided as well |
 *
 */

class ControlBoard_iFaceTransLayer_ros2 :
        public yarp::os::Thread,
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle
{
public:
    ControlBoard_iFaceTransLayer_ros2();
    ~ControlBoard_iFaceTransLayer_ros2();

    // yarp::dev::IWrapper
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // DeviceDriver
    bool open(yarp::os::Searchable &params) override;
    bool close() override;

    // Thread
    void run() override;


private:
    // Utilities
    bool messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values, const std::vector<double> &derivative);
    bool messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values);
    bool messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<std::string> &ref_values);
    bool namesCheck(const std::vector<std::string> &names);

    // Wrapper related
    bool setDevice(yarp::dev::DeviceDriver* device, bool owned);
    bool openAndAttachSubDevice(yarp::os::Property& prop);
    void closeDevice();
    void closePorts();

    // Service callbacks
    void getControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Request> request,
                                 std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Response> response);
    void setControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Request> request,
                                 std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Response> response);
    void getJointsNamesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Request> request,
                                std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Response> response);
    void getAvailableModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Request> request,
                                   std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Response> response);
    
    //Subscription callbacks
    void positionGet_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void velocityGet_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void positionTopic_callback(const yarp_control_msgs::msg::Position::SharedPtr msg);
    void positionDirectTopic_callback(const yarp_control_msgs::msg::PositionDirect::SharedPtr msg);
    void velocityTopic_callback(const yarp_control_msgs::msg::Velocity::SharedPtr msg);

    // Generic
    mutable std::mutex m_cmdMutex;

    // parameters from configuration
    std::string m_posTopicName;
    std::string m_posDirTopicName;
    std::string m_velTopicName;
    std::string m_rosPosTopicName;
    std::string m_rosVelTopicName;
    std::string m_getModesSrvName;
    std::string m_setModesSrvName;
    std::string m_getJointsNamesSrvName;
    std::string m_getAvailableModesSrvName;
    std::string m_nodeName;
    std::string m_name;
    bool m_subdevice_owned {false};
    size_t m_subdevice_joints {0};
    bool m_subdevice_ready = false;
    std::map<std::string,size_t> m_quickJointRef;

    // Driver
    yarp::dev::DeviceDriver*     m_subdevice_ptr{nullptr};
    yarp::dev::IAxisInfo*        m_iAxisInfo{nullptr};
    yarp::dev::IControlMode*     m_iControlMode{nullptr};
    yarp::dev::IPositionControl* m_iPositionControl{nullptr};
    yarp::dev::IVelocityControl* m_iVelocityControl{nullptr};
    yarp::dev::IPositionDirect*  m_iPositionDirect{nullptr};

    // Ros2 related attributes
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<yarp_control_msgs::msg::Position>::SharedPtr            m_posSubscription;
    rclcpp::Subscription<yarp_control_msgs::msg::PositionDirect>::SharedPtr      m_posDirectSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr            m_rosPosSubscription;
    rclcpp::Subscription<yarp_control_msgs::msg::Velocity>::SharedPtr            m_velSubscription;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr            m_rosVelSubscription;
    rclcpp::Service<yarp_control_msgs::srv::GetJointsNames>::SharedPtr           m_getJointsNamesSrv;
    rclcpp::Service<yarp_control_msgs::srv::GetControlModes>::SharedPtr          m_getControlModesSrv;
    rclcpp::Service<yarp_control_msgs::srv::SetControlModes>::SharedPtr          m_setControlModesSrv;
    rclcpp::Service<yarp_control_msgs::srv::GetAvailableControlModes>::SharedPtr m_getAvailableModesSrv;
};

#endif // YARP_ROS2_CONTROLBOARD_IFACETRANSLAYER_ROS2_H
