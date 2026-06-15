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
    const auto prefix = m_topic_name.empty() ? "" : ("/" + m_topic_name);

    m_stat = m_node->create_publisher<geometry_msgs::msg::Pose>(
        prefix + "/state/pose", 10);

    m_buttons = m_node->create_publisher<std_msgs::msg::Int32MultiArray>(
        prefix + "/state/buttons", 10);

    return true;
}

bool HapticDevice_nws_ros2::subscriberConfigureRosHandlers()
{
    const auto prefix = m_topic_name.empty() ? "" : ("/" + m_topic_name);

    m_feedback = m_node->create_subscription<geometry_msgs::msg::Vector3>(
        prefix + "/feedback", 10,
        std::bind(&HapticDevice_nws_ros2::feedbackCallback, this, std::placeholders::_1));

    if (!m_feedback)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the feedback subscription";
        return false;
    }

    return true;
}

bool HapticDevice_nws_ros2::configureRosParameters()
{
    std::map<int, double> params;
    constexpr const char* DEFAULT_MODE_FEEDBACK = "cartesian";

    // Declare node Parameters
    rcl_interfaces::msg::ParameterDescriptor descriptor_msg;
    descriptor_msg.name = "mode_feedback";
    descriptor_msg.description = "Defines the type of the feedback, cartesian or joint.";
    descriptor_msg.read_only = false;
    descriptor_msg.additional_constraints = "Only 'cartesian' or 'joint' are allowed.";
    descriptor_msg.set__type(rcl_interfaces::msg::ParameterType::PARAMETER_STRING);

    m_node->declare_parameter<std::string>("mode_feedback", DEFAULT_MODE_FEEDBACK, descriptor_msg);
    m_node->get_parameter("mode_feedback", mode_feedback);

    m_params = m_node->add_on_set_parameters_callback([this](const auto & parameters) {
        return parameter_callback(parameters);
    });

    return true;
}

bool HapticDevice_nws_ros2::servicesConfigureRosHandlers()
{
    const auto prefix = m_topic_name.empty() ? "" : ("/" + m_topic_name);

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

    m_maxForce = m_node->create_service<yarp_control_msgs::srv::GetMaxFeedback>(
        prefix + "/get_max_feedback",
        std::bind(&HapticDevice_nws_ros2::maxFeedbackCallback,
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
        prefix + "/get_transform",
        std::bind(&HapticDevice_nws_ros2::getTransformationCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));

    if (!m_getTransformation)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the get_transformation service";
        return false;
    }

    m_setTransformation = m_node->create_service<yarp_control_msgs::srv::SetTransformation>(
        prefix + "/set_transform",
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
    m_stopFeedbackService.reset();
    m_maxForce.reset();
    m_getTransformation.reset();
    m_setTransformation.reset();

    // Parameters
    m_params.reset();
}
// -----------------------------------------------------------------------------
void HapticDevice_nws_ros2::feedbackCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    if (iHapticDevice != nullptr)
    {
        yarp::sig::Vector force(3, 0.0);
        if (msg->x != 0.0 || msg->y != 0.0 || msg->z != 0.0)
        {
            force[0] = msg->x;
            force[1] = msg->y;
            force[2] = msg->z;
        }
        iHapticDevice->setFeedback(force);
    }
    else
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available in feedback callback";
    }
}

// -----------------------------------------------------------------------------
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

void HapticDevice_nws_ros2::maxFeedbackCallback(
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

    geometry_msgs::msg::Vector3 force_msg;
    force_msg.x = force[0];
    force_msg.y = force[1];
    force_msg.z = force[2];

    response->max_feedback = force_msg;
    response->success = true;
}

// -----------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult HapticDevice_nws_ros2::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
        if (param.get_name() == "mode_feedback")
        {
            std::string new_mode = param.value_to_string();
            if (mode_feedback != new_mode)
            {
                bool success = forceModeCallback(new_mode);
                if (success)
                {
                    mode_feedback = new_mode;
                }
                else
                {
                    result.successful = false;
                    result.reason = "Failed to set mode_feedback to " + new_mode;
                    yCError(HAPTICDEVICE_NWS_ROS2) << result.reason;
                }
            }
        }
        else
        {
            yCWarning(HAPTICDEVICE_NWS_ROS2, "Parameter '%s' is not recognized. Ignoring.", param.get_name().c_str());
        }
    }

    return result;
}

bool HapticDevice_nws_ros2::forceModeCallback(const std::string& mode)
{
    if (!iHapticDevice)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "IHapticDevice interface not available";
        return false;
    }

    if (mode != "cartesian" && mode != "joint")
    {
        yCError(HAPTICDEVICE_NWS_ROS2, "Invalid value for mode_feedback: %s. Allowed values are 'cartesian' or 'joint'", mode.c_str());
        return false;
    }

    bool success = (mode == "cartesian")
        ? iHapticDevice->setCartesianForceMode()
        : iHapticDevice->setJointTorqueMode();

    if (!success)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to set haptic mode";
        return false;
    }

    bool is_force_mode;
    if (!iHapticDevice->isCartesianForceModeEnabled(is_force_mode))
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to get haptic mode";
        return false;
    }

    bool expected_force_mode = (mode == "cartesian");
    if (is_force_mode != expected_force_mode)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Switched to" << mode << "mode failed";
        return false;
    }

    yCInfo(HAPTICDEVICE_NWS_ROS2) << "Switched to" << mode << "mode successfully";
    return true;
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

    if (!publisherConfigureRosHandlers() || !subscriberConfigureRosHandlers()
        || !servicesConfigureRosHandlers() || !configureRosParameters())
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

// -----------------------------------------------------------------------------

bool HapticDevice_nws_ros2::detach()
{
    if (yarp::os::PeriodicThread::isRunning())
    {
        yarp::os::PeriodicThread::stop();
    }
    destroyRosHandlers();
    iHapticDevice = nullptr;
    return true;
}

// ------------------- DeviceDriver Related ------------------------------------

bool HapticDevice_nws_ros2::open(yarp::os::Searchable & config)
{
    if (!parseParams(config))
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Failed to parse parameters";
        return false;
    }

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
        yCError(HAPTICDEVICE_NWS_ROS2) << "Opening" << m_node_name << "node failed, check your YARP-ROS2 network configuration";
        return false;
    }

    m_spinner = new Ros2Spinner(m_node);
    if (!m_spinner)
    {
        yCError(HAPTICDEVICE_NWS_ROS2) << "Could not initialize the Ros2Spinner";
        return false;
    }

    yCInfo(HAPTICDEVICE_NWS_ROS2) << "Node" << m_node_name << "launched successfully";
    return true;
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

    return detach();
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

            yarp::sig::Matrix dcm = yarp::math::rpy2dcm(rpy);
            yarp::math::Quaternion pose_q;
            pose_q.fromRotationMatrix(dcm);
            pose_msg.orientation.x = pose_q.x();
            pose_msg.orientation.y = pose_q.y();
            pose_msg.orientation.z = pose_q.z();
            pose_msg.orientation.w = pose_q.w();
            m_stat->publish(pose_msg);
        }

        // Buttons
        yarp::sig::Vector buttons;
        if (iHapticDevice->getButtons(buttons) && buttons.size() >= 1)
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
