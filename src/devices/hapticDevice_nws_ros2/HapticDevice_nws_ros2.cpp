// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HapticDevice_nws_ros2.h"

#include <cmath>
#include <yarp/math/Math.h>
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


    return true;
}

bool HapticDevice_nws_ros2::subscriberConfigureRosHandlers()
{
    const auto prefix = "/" + m_topic_name;

    m_feedback = m_node->create_subscription<sensor_msgs::msg::JointState>(
        prefix + "/feedback", 10,
        std::bind(&HapticDevice_nws_ros2::feedbackCallback, this, std::placeholders::_1));

    if (!m_feedback)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the feedback subscription";
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

    m_forceMode = m_node->create_service<std_srvs::srv::Trigger>(
        prefix + "/state/force_mode",
        std::bind(&HapticDevice_nws_ros2::forceModeCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    if (!m_forceMode)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the forceMode service";
        return false;
    }

    m_maxForce = m_node->create_service<yarp_control_msgs::srv::GetMaxFeedback>(
        prefix + "/state/max_force_feedback", 
        std::bind(&HapticDevice_nws_ros2::maxForceFeedbackCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        )
    );
    
    if (!m_maxForce)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the MaxForceFeedback service";
        return false;
    }

    m_getTransformation = m_node->create_service<yarp_control_msgs::srv::GetTransformation>(
        prefix + "/state/transform",
        std::bind(&HapticDevice_nws_ros2::getTransformationCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));
     
    m_setTransformation = m_node->create_service<yarp_control_msgs::srv::SetTransformation>(
        prefix + "/set_transformation",
        std::bind(&HapticDevice_nws_ros2::setTransformationCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));

    if (!m_setTransformation)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the set_transformation service";
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

    // Subscribers
    m_feedback.reset();

    // Services
    m_setForceModeService.reset();
    m_stopFeedbackService.reset();
    m_maxForce.reset();
    m_getTransformation.reset();
    m_setTransformation.reset();
    m_forceMode.reset();
}
// -----------------------------------------------------------------------------
void HapticDevice_nws_ros2::feedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

void HapticDevice_nws_ros2::setTransformationCallback(
    const yarp_control_msgs::srv::SetTransformation::Request::SharedPtr request,
    std::shared_ptr<yarp_control_msgs::srv::SetTransformation::Response> response)
{
    if (!request)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Received null request in set_transformation callback";
        response->success = false;
        response->response = "Received null request";
        return;
    }

    if (iHapticDevice == nullptr)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available in set_transformation callback";
        return;
    }

    yarp::sig::Matrix T(4, 4);
    T.eye();

    const auto& q = request->transform.rotation;
    const auto& t = request->transform.translation;

    // Quaternion to rotation matrix
    yarp::math::Quaternion yarp_q;
    yarp_q.x() = q.x;
    yarp_q.y() = q.y;
    yarp_q.z() = q.z;
    yarp_q.w() = q.w;
    T.setSubmatrix(yarp_q.toRotationMatrix3x3(), 0, 0);
    T(0, 3) = t.x;
    T(1, 3) = t.y;
    T(2, 3) = t.z;

    if (!iHapticDevice->setTransformation(T))
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "setTransformation() failed";
        response->response = "setTransformation failed";
        response->success = false;
        return;
    }
    
    response->success = true;
}

void HapticDevice_nws_ros2::getTransformationCallback(
    const std::shared_ptr<yarp_control_msgs::srv::GetTransformation::Request> request,
    std::shared_ptr<yarp_control_msgs::srv::GetTransformation::Response> response)
{
    if (!iHapticDevice)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available";
        response->success = false;
        response->response = "IHapticDevice interface not available";
        return;
    }
    // Transform
    yarp::sig::Matrix trans;
    if (!iHapticDevice->getTransformation(trans) || trans.rows() < 3 || trans.cols() < 4)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to get transformation";
        response->success = false;
        response->response = "Failed to get transformation";
        return;
    }

    geometry_msgs::msg::Transform trans_msg;
    trans_msg.translation.x = trans(0, 3);
    trans_msg.translation.y = trans(1, 3);
    trans_msg.translation.z = trans(2, 3);

    yarp::math::Quaternion trans_q;
    yarp::sig::Matrix R = trans.submatrix(0,2,0,2);
    trans_q.fromRotationMatrix(R);
    trans_msg.rotation.x = trans_q.x();
    trans_msg.rotation.y = trans_q.y();
    trans_msg.rotation.z = trans_q.z();
    trans_msg.rotation.w = trans_q.w();

    response->transform = trans_msg;
    response->success = true;
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

void HapticDevice_nws_ros2::forceModeCallback(
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


    bool is_force_mode;
    if (!iHapticDevice->isCartesianForceModeEnabled(is_force_mode))
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to get haptic mode";
        response->success = false;
        response->message = "Failed to get haptic mode";
        return;
    }

    response->success = true;
    response->message = is_force_mode ? "Currently in Cartesian Force mode" : "Currently in Joint Torque mode";
}

void HapticDevice_nws_ros2::maxForceFeedbackCallback(
    const std::shared_ptr<yarp_control_msgs::srv::GetMaxFeedback::Request> request,
    std::shared_ptr<yarp_control_msgs::srv::GetMaxFeedback::Response> response)
{
    if (!iHapticDevice)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available";
        response->success = false;
        return;
    }

    // Force Feedback
    yarp::sig::Vector force;
    if (!iHapticDevice->getMaxFeedback(force) || force.size() < 3)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to get max feedback";
        response->success = false;
        return;
    }

    geometry_msgs::msg::Wrench force_msg;
    force_msg.force.x = force[0];
    force_msg.force.y = force[1];
    force_msg.force.z = force[2];

    response->max_feedback = force_msg;
    response->success = true;
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
        delete m_spinner;
        m_spinner = nullptr;
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
        yarp::sig::Vector pos;
        yarp::sig::Vector rpy;
        if (iHapticDevice->getPosition(pos) && pos.size() >= 3 &&
            iHapticDevice->getOrientation(rpy) && rpy.size() >= 3)
        {
            geometry_msgs::msg::Pose pose_msg;
            pose_msg.position.x = pos[0];
            pose_msg.position.y = pos[1];
            pose_msg.position.z = pos[2];

            yarp::sig::Matrix pose_matrix = yarp::math::rpy2dcm(rpy);
            yarp::math::Quaternion pose_q;
            pose_q.fromRotationMatrix(pose_matrix);
            pose_msg.orientation.x = pose_q.x();
            pose_msg.orientation.y = pose_q.y();
            pose_msg.orientation.z = pose_q.z();
            pose_msg.orientation.w = pose_q.w();
            m_stat->publish(pose_msg);
        }

        // Buttons
        yarp::sig::Vector buttons;
        if (iHapticDevice->getButtons(buttons))
        {
            std_msgs::msg::Int32MultiArray btn_msg;
            for (size_t i = 0; i < buttons.size(); ++i)
            {
                btn_msg.data.push_back(static_cast<int>(buttons[i]));
            }
            m_buttons->publish(btn_msg);
        }
    }
}
