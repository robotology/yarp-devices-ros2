/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_HAPTICDEVICE_NWS_ROS2_H
#define YARP_DEV_HAPTICDEVICE_NWS_ROS2_H

#include <string>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IHapticDevice.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <Ros2Spinner.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "HapticDevice_nws_ros2_ParamsParser.h"

/**
 *  @ingroup dev_impl_nws_ros2
 *
 * \brief `HapticDevice_nws_ros2`: A HapticDevice network wrapper server for ROS2.
 *
 * \section HapticDevice_nws_ros2_device_parameters Description of input parameters
 *
 *  Parameters required by this device are shown in class: HapticDevice_nws_ros2_ParamsParser
 */
class HapticDevice_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        public yarp::os::PeriodicThread,
        HapticDevice_nws_ros2_ParamsParser
{
public:
    HapticDevice_nws_ros2();
    HapticDevice_nws_ros2(const HapticDevice_nws_ros2&) = delete;
    HapticDevice_nws_ros2(HapticDevice_nws_ros2&&) = delete;
    HapticDevice_nws_ros2& operator=(const HapticDevice_nws_ros2&) = delete;
    HapticDevice_nws_ros2& operator=(HapticDevice_nws_ros2&&) = delete;
    ~HapticDevice_nws_ros2() override = default;

    // yarp::dev::DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // yarp::dev::WrapperSingle
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // yarp::os::PeriodicThread
    bool threadInit() override;
    void threadRelease() override;
    void run() override;

private:
    // ROS2 handler setup / teardown
    bool publisherConfigureRosHandlers();
    bool subscriberConfigureRosHandlers();
    bool servicesConfigureRosHandlers();
    void destroyRosHandlers();

    // Callbacks
    void _feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void _setTransformationCallback(const geometry_msgs::msg::Transform::SharedPtr msg);
    void setForceModeCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void stopFeedbackCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // ROS2 node and spinner
    rclcpp::Node::SharedPtr m_node;
    Ros2Spinner*            m_spinner {nullptr};

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr            m_stat;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr      m_buttons;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr          m_force;
    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr       m_transform;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 m_forceMode;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr     m_feedback;
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr    m_setTransformation;

    // Services
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                m_setForceModeService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                m_stopFeedbackService;

    // Device interface
    yarp::dev::IHapticDevice* iHapticDevice {nullptr};
};

#endif // YARP_DEV_HAPTICDEVICE_NWS_ROS2_H