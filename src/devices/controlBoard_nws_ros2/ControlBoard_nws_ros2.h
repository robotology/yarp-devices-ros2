/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_CONTROLBOARD_NWS_ROS2_H
#define YARP_DEV_CONTROLBOARD_NWS_ROS2_H

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IAxisInfo.h>
#include <Ros2Spinner.h>

#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map>

//Custom ros2 interfaces
#include <yarp_control_msgs/srv/get_control_modes.hpp>
#include <yarp_control_msgs/srv/set_control_modes.hpp>
#include <yarp_control_msgs/srv/get_available_control_modes.hpp>
#include <yarp_control_msgs/srv/get_joints_names.hpp>
#include <yarp_control_msgs/msg/position.hpp>
#include <yarp_control_msgs/msg/velocity.hpp>
#include <yarp_control_msgs/msg/position_direct.hpp>

#include <mutex>



/**
 *  @ingroup dev_impl_nws_ros2
 *
 * \brief `controlBoard_nws_ros`: A controlBoard network wrapper server for ROS2.
 *
 * \section controlBoard_nws_ros_device_parameters Description of input parameters
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | node_name      |      -         | string  | -              |   -           | Yes                         | set the name for ROS node                                         | must not start with a leading '/' |
 * | topic_name     |      -         | string  | -              |   -           | Yes                         | set the name for ROS topic                                        | must start with a leading '/' |
 * | msgs_name      |      -         | string  | -              |   -           | No                          | set the base name for the topics and interfaces                   | If it is not specified, the control related topics and services will not be initialized |
 * | period         |      -         | double  | s              |   0.02        | No                          | refresh period of the broadcasted values in s                     | optional, default 20ms |
 *
 * ROS message type used is sensor_msgs/JointState.msg (http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
 */

class ControlBoard_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::os::PeriodicThread,
        public yarp::dev::WrapperSingle
{
private:
    sensor_msgs::msg::JointState m_ros_struct;

    yarp::sig::Vector m_times; // time for each joint

    std::vector<std::string>     m_jointNames; // name of the joints
    std::string                  m_nodeName;                // name of the rosNode
    std::string                  m_jointStateTopicName;               // name of the rosTopic
    std::string                  m_msgs_name;
    std::string                  m_posTopicName;
    std::string                  m_posDirTopicName;
    std::string                  m_velTopicName;
    std::string                  m_getModesSrvName;
    std::string                  m_setModesSrvName;
    std::string                  m_getJointsNamesSrvName;
    std::string                  m_getAvailableModesSrvName;
    std::map<std::string,size_t> m_quickJointRef;
    mutable std::mutex           m_cmdMutex;

//     yarp::os::Node* node; // ROS node
    std::uint32_t m_counter {0}; // incremental counter in the ROS message

//     yarp::os::PortWriterBuffer<yarp::rosmsg::sensor_msgs::JointState> rosOutputState_buffer; // Buffer associated to the ROS topic
//     yarp::os::Publisher<yarp::rosmsg::sensor_msgs::JointState> rosPublisherPort;             // Dedicated ROS topic publisher

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher;
    rclcpp::Node::SharedPtr m_node;

    static constexpr double m_default_period = 0.02; // s
    double m_period {m_default_period};

    yarp::os::Stamp m_time; // envelope to attach to the state port

    size_t m_subdevice_joints {0};

    // Devices
    yarp::dev::DeviceDriver*     m_subdevice_ptr{nullptr};
    yarp::dev::IAxisInfo*        m_iAxisInfo{nullptr};
    yarp::dev::IEncodersTimed*   m_iEncodersTimed{nullptr};
    yarp::dev::ITorqueControl*   m_iTorqueControl{nullptr};
    yarp::dev::IPositionDirect*  m_iPositionDirect{nullptr};
    yarp::dev::IVelocityControl* m_iVelocityControl{nullptr};
    yarp::dev::IControlMode*     m_iControlMode{nullptr};
    yarp::dev::IPositionControl* m_iPositionControl{nullptr};

    // Ros2 related attributes
    Ros2Spinner*            m_spinner{nullptr};
    rclcpp::Subscription<yarp_control_msgs::msg::Position>::SharedPtr            m_posSubscription;
    rclcpp::Subscription<yarp_control_msgs::msg::PositionDirect>::SharedPtr      m_posDirectSubscription;
    rclcpp::Subscription<yarp_control_msgs::msg::Velocity>::SharedPtr            m_velSubscription;
    rclcpp::Service<yarp_control_msgs::srv::GetJointsNames>::SharedPtr           m_getJointsNamesSrv;
    rclcpp::Service<yarp_control_msgs::srv::GetControlModes>::SharedPtr          m_getControlModesSrv;
    rclcpp::Service<yarp_control_msgs::srv::SetControlModes>::SharedPtr          m_setControlModesSrv;
    rclcpp::Service<yarp_control_msgs::srv::GetAvailableControlModes>::SharedPtr m_getAvailableModesSrv;

    bool setDevice(yarp::dev::DeviceDriver* device);
    bool initRos2Control(const std::string& name);

    void closeDevice();
    void closePorts();
    bool updateAxisName();

    // Utilities
    bool messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values, const std::vector<double> &derivative);
    bool messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values);
    bool messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<std::string> &ref_values);
    bool namesCheck(const std::vector<std::string> &names);

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
    void positionTopic_callback(const yarp_control_msgs::msg::Position::SharedPtr msg);
    void positionDirectTopic_callback(const yarp_control_msgs::msg::PositionDirect::SharedPtr msg);
    void velocityTopic_callback(const yarp_control_msgs::msg::Velocity::SharedPtr msg);

public:
    ControlBoard_nws_ros2();
    ControlBoard_nws_ros2(const ControlBoard_nws_ros2&) = delete;
    ControlBoard_nws_ros2(ControlBoard_nws_ros2&&) = delete;
    ControlBoard_nws_ros2& operator=(const ControlBoard_nws_ros2&) = delete;
    ControlBoard_nws_ros2& operator=(ControlBoard_nws_ros2&&) = delete;
    ~ControlBoard_nws_ros2() override = default;

    // yarp::dev::DeviceDriver
    bool close() override;
    bool open(yarp::os::Searchable& prop) override;

    // yarp::dev::IWrapper
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // yarp::os::PeriodicThread
    void run() override;
};

#endif // YARP_DEV_CONTROLBOARD_NWS_ROS2_H
