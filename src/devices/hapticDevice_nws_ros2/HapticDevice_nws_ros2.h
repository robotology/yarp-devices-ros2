/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_HAPTICDEVICE_NWS_ROS2_H
#define YARP_DEV_HAPTICDEVICE_NWS_ROS2_H

#include <string>
#include <vector>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IHapticDevice.h>
#include <yarp/os/PeriodicThread.h>

#include <Ros2Spinner.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yarp_control_msgs/srv/get_max_feedback.hpp>
#include <yarp_control_msgs/srv/get_transformation.hpp>
#include <yarp_control_msgs/srv/set_transformation.hpp>

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
    void run() override;

private:
    // ROS2 handler setup / teardown
    bool publisherConfigureRosHandlers();
    bool subscriberConfigureRosHandlers();
    bool servicesConfigureRosHandlers();
    void destroyRosHandlers();

    // Callbacks subscribed
    void feedbackCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    // Callbacks for services
    void stopFeedbackCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void maxFeedbackCallback(
        const std::shared_ptr<yarp_control_msgs::srv::GetMaxFeedback::Request> request,
        std::shared_ptr<yarp_control_msgs::srv::GetMaxFeedback::Response> response);
    void getTransformationCallback(
        const std::shared_ptr<yarp_control_msgs::srv::GetTransformation::Request> request,
        std::shared_ptr<yarp_control_msgs::srv::GetTransformation::Response> response);
    void setTransformationCallback(
        const std::shared_ptr<yarp_control_msgs::srv::SetTransformation::Request> request,
        std::shared_ptr<yarp_control_msgs::srv::SetTransformation::Response> response);

    // Callbacks for parameters
    bool configureRosParameters();
    bool forceModeCallback(const std::string& mode);
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> & parameters);

    // ROS2 node and spinner
    rclcpp::Node::SharedPtr m_node;
    Ros2Spinner*            m_spinner {nullptr};
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_params;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr                m_stat;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr          m_buttons;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr          m_feedback;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                    m_stopFeedbackService;
    rclcpp::Service<yarp_control_msgs::srv::GetMaxFeedback>::SharedPtr    m_maxForce;
    rclcpp::Service<yarp_control_msgs::srv::GetTransformation>::SharedPtr m_getTransformation;
    rclcpp::Service<yarp_control_msgs::srv::SetTransformation>::SharedPtr m_setTransformation;

    // Parameters
    std::string mode_feedback;

    // Device interface
    yarp::dev::IHapticDevice* iHapticDevice {nullptr};
};

#endif // YARP_DEV_HAPTICDEVICE_NWS_ROS2_H
