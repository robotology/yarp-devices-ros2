// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HapticDevice_nws_ros2.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <kdl/frames.hpp>
#include <yarp/os/LogStream.h>
#include <Ros2Utils.h>

YARP_LOG_COMPONENT(HAPTICDEVICE_NWS_ROS2, "yarp.devices.HapticDevice_nws_ros2")

HapticDevice_nws_ros2::HapticDevice_nws_ros2() : yarp::os::PeriodicThread(m_period)
{
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::publisherConfigureRosHandlers()
{
    const auto prefix = "/" + m_topic_name;

    m_stat = m_node->create_publisher<geometry_msgs::msg::Pose>(
        prefix + "/state/pose", 10);

    m_buttons = m_node->create_publisher<std_msgs::msg::Int32MultiArray>(
        prefix + "/state/buttons", 10);

    m_force = m_node->create_publisher<geometry_msgs::msg::Wrench>(
        prefix + "/state/force_feedback", 10);

    m_transform = m_node->create_publisher<geometry_msgs::msg::Transform>(
        prefix + "/state/transform", 10);

    m_forceMode = m_node->create_publisher<std_msgs::msg::Bool>(
        prefix + "/state/force_mode", 10);

    return true;
}

bool HapticDevice_nws_ros2::subscriberConfigureRosHandlers()
{
    const auto prefix = "/" + m_topic_name;

    m_feedback = m_node->create_subscription<sensor_msgs::msg::JointState>(
        prefix + "/feedback", 10,
        std::bind(&HapticDevice_nws_ros2::_feedbackCallback, this, std::placeholders::_1));

    if (!m_feedback)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the feedback subscription";
        return false;
    }

    m_setTransformation = m_node->create_subscription<geometry_msgs::msg::Transform>(
        prefix + "/set_transformation", 10,
        std::bind(&HapticDevice_nws_ros2::_setTransformationCallback, this, std::placeholders::_1));

    if (!m_setTransformation)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the set_transformation subscription";
        return false;
    }

    return true;
}

bool HapticDevice_nws_ros2::servicesConfigureRosHandlers()
{
    const auto prefix = "/" + m_topic_name;

    m_setForceModeService = m_node->create_service<std_srvs::srv::SetBool>(
        prefix + "/set_force_mode",
        std::bind(&HapticDevice_nws_ros2::setForceModeCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    if (!m_setForceModeService)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the SetForceMode service";
        return false;
    }

    m_stopFeedbackService = m_node->create_service<std_srvs::srv::Trigger>(
        prefix + "/stop_feedback",
        std::bind(&HapticDevice_nws_ros2::stopFeedbackCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    if (!m_stopFeedbackService)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the StopFeedback service";
        return false;
    }

    return true;
}

// -----------------------------------------------------------------------------

void HapticDevice_nws_ros2::destroyRosHandlers()
{
    // Publishers
    m_stat.reset();
    m_buttons.reset();
    m_force.reset();
    m_transform.reset();
    m_forceMode.reset();

    // Subscribers
    m_feedback.reset();
    m_setTransformation.reset();

    // Services
    m_setForceModeService.reset();
    m_stopFeedbackService.reset();
}

// -----------------------------------------------------------------------------
void HapticDevice_nws_ros2::_feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (iHapticDevice != nullptr)
    {
        yarp::sig::Vector force(3, 0.0);
        if (msg->effort.size() >= 3)
        {
            force[0] = msg->effort[0];
            force[1] = msg->effort[1];
            force[2] = msg->effort[2];
        }
        iHapticDevice->setFeedback(force);
    }
    else
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available in feedback callback";
    }
}

void HapticDevice_nws_ros2::_setTransformationCallback(const geometry_msgs::msg::Transform::SharedPtr msg)
{
    if (iHapticDevice == nullptr)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available in set_transformation callback";
        return;
    }

    yarp::sig::Matrix T(4, 4);
    T.eye();

    const auto& q = msg->rotation;
    
    // Quaternion to rotation matrix
    double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    T(0, 0) = 1.0 - 2.0*(qy*qy + qz*qz);
    T(0, 1) = 2.0*(qx*qy - qz*qw);
    T(0, 2) = 2.0*(qx*qz + qy*qw);
    T(1, 0) = 2.0*(qx*qy + qz*qw);
    T(1, 1) = 1.0 - 2.0*(qx*qx + qz*qz);
    T(1, 2) = 2.0*(qy*qz - qx*qw);
    T(2, 0) = 2.0*(qx*qz - qy*qw);
    T(2, 1) = 2.0*(qy*qz + qx*qw);
    T(2, 2) = 1.0 - 2.0*(qx*qx + qy*qy);

    T(0, 3) = msg->translation.x;
    T(1, 3) = msg->translation.y;
    T(2, 3) = msg->translation.z;

    if (!iHapticDevice->setTransformation(T))
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "setTransformation() failed";
    }
}

void HapticDevice_nws_ros2::setForceModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (!iHapticDevice)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available";
        response->success = false;
        response->message = "IHapticDevice interface not available";
        return;
    }

    bool result = request->data ?
                  iHapticDevice->setCartesianForceMode() :
                  iHapticDevice->setJointTorqueMode();

    response->message = request->data ?
                          "Cartesian Force mode enabled" :
                          "Joint Torque mode enabled";

    if (result)
    {
        yCInfo(HAPTICDEVICE_NWS_ROS2) << response->message;
        response->success = true;
    }
    else
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to set haptic mode";
        response->success = false;
        response->message = "Failed to set haptic mode";
    }
}

void HapticDevice_nws_ros2::stopFeedbackCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!iHapticDevice)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available";
        response->success = false;
        response->message = "IHapticDevice interface not available";
        return;
    }

    if (iHapticDevice->stopFeedback())
    {
        yCInfo(HAPTICDEVICE_NWS_ROS2) << "Feedback stopped";
        response->success = true;
        response->message = "Feedback stopped";
    }
    else
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to stop feedback";
        response->success = false;
        response->message = "Failed to stop feedback";
    }
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::attach(yarp::dev::PolyDriver * poly)
{
    if (poly == nullptr)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "attach() received nullptr";
        return false;
    }

    if (!poly->isValid())
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "attach() received invalid PolyDriver";
        return false;
    }

    if (!poly->view(iHapticDevice))
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "attach() failed to obtain iHapticDevice interface";
        return false;
    }

    if (!publisherConfigureRosHandlers() || !subscriberConfigureRosHandlers() || !servicesConfigureRosHandlers())
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to configure ROS handlers";
        destroyRosHandlers();
        return false;
    }

    yarp::os::PeriodicThread::setPeriod(m_period);
    if (!yarp::os::PeriodicThread::start())
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Error starting thread";
        return false;
    }

    if (m_spinner)
    {
        if (!m_spinner->start())
        {
            yCError(HAPTICDEVICE_NWS_ROS2) << "Error starting the spinner";
            return false;
        }
    }

    return true;
}

bool HapticDevice_nws_ros2::threadInit()
{
    return true;
}

// -----------------------------------------------------------------------------
bool HapticDevice_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    destroyRosHandlers();
    iHapticDevice = nullptr;
    return true;
}

// ------------------- DeviceDriver Related ------------------------------------

bool HapticDevice_nws_ros2::open(yarp::os::Searchable & config)
{
    parseParams(config);

    if (m_namespace.empty())
    {
        m_node = NodeCreator::createNode(m_node_name);
    }
    else
    {
        m_node = NodeCreator::createNode(m_node_name, m_namespace);
    }

    if (m_node == nullptr)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Opening" << m_node_name << "Node failed, check your yarp-ROS2 network configuration";
        return false;
    }

    m_spinner = new Ros2Spinner(m_node);
    if (!m_spinner)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the Ros2Spinner";
        return false;
    }

    return true;
}

void HapticDevice_nws_ros2::threadRelease()
{
}

// -----------------------------------------------------------------------------

bool HapticDevice_nws_ros2::close()
{
    if (m_spinner)
    {
        if (m_spinner->isRunning())
        {
            m_spinner->stop();
        }
    }

    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detach();

    return true;
}

// -----------------------------------------------------------------------------
void HapticDevice_nws_ros2::run()
{
    if (iHapticDevice != nullptr)
    {
        // Pose
        yarp::sig::Vector pos, rpy;
        iHapticDevice->getPosition(pos);
        iHapticDevice->getOrientation(rpy);

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pos[0];
        pose_msg.position.y = pos[1];
        pose_msg.position.z = pos[2];

        const auto ori = KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]);
        ori.GetQuaternion(pose_msg.orientation.x, pose_msg.orientation.y,
                          pose_msg.orientation.z, pose_msg.orientation.w);
        m_stat->publish(pose_msg);

        // Buttons
        std_msgs::msg::Int32MultiArray btn_msg;
        yarp::sig::Vector buttons;
        iHapticDevice->getButtons(buttons);
        for (size_t i = 0; i < buttons.size(); ++i)
        {
            btn_msg.data.push_back(static_cast<int>(buttons[i]));
        }
        m_buttons->publish(btn_msg);

        // Force Feedback
        yarp::sig::Vector force;
        iHapticDevice->getMaxFeedback(force);
        geometry_msgs::msg::Wrench force_msg;
        force_msg.force.x = force[0];
        force_msg.force.y = force[1];
        force_msg.force.z = force[2];
        m_force->publish(force_msg);

        // Transform
        yarp::sig::Matrix trans;
        iHapticDevice->getTransformation(trans);
        geometry_msgs::msg::Transform trans_msg;
        trans_msg.translation.x = trans(0, 3);
        trans_msg.translation.y = trans(1, 3);
        trans_msg.translation.z = trans(2, 3);

        KDL::Rotation rot(trans(0,0), trans(0,1), trans(0,2),
                          trans(1,0), trans(1,1), trans(1,2),
                          trans(2,0), trans(2,1), trans(2,2));
        rot.GetQuaternion(trans_msg.rotation.x, trans_msg.rotation.y,
                          trans_msg.rotation.z, trans_msg.rotation.w);

        m_transform->publish(trans_msg);

        // Force mode
        bool isForce = false;
        iHapticDevice->isCartesianForceModeEnabled(isForce);
        std_msgs::msg::Bool mode_msg;
        mode_msg.data = isForce;
        m_forceMode->publish(mode_msg);
    }
}
