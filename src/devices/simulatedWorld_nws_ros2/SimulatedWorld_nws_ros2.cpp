/*
 * SPDX-FileCopyrightText: 2026 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "SimulatedWorld_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <Ros2Utils.h>

#include <yarp/sig/Pose6D.h>
#include <yarp/sig/ColorRGB.h>

#include <string>
#include <vector>

using namespace yarp::dev;
using namespace yarp::os;
using namespace std::placeholders;

namespace {
YARP_LOG_COMPONENT(SIMULATEDWORLD_NWS_ROS2, "yarp.device.simulatedWorld_nws_ros2")

inline yarp::sig::Pose6D toYarpPose(const simulated_world_nws_ros2_msgs::msg::Pose6D& msg)
{
    yarp::sig::Pose6D pose;
    pose.x = msg.x;
    pose.y = msg.y;
    pose.z = msg.z;
    pose.roll = msg.roll;
    pose.pitch = msg.pitch;
    pose.yaw = msg.yaw;
    return pose;
}

inline simulated_world_nws_ros2_msgs::msg::Pose6D toRosPose(const yarp::sig::Pose6D& pose)
{
    simulated_world_nws_ros2_msgs::msg::Pose6D msg;
    msg.x = pose.x;
    msg.y = pose.y;
    msg.z = pose.z;
    msg.roll = pose.roll;
    msg.pitch = pose.pitch;
    msg.yaw = pose.yaw;
    return msg;
}

inline yarp::sig::ColorRGB toYarpColor(const simulated_world_nws_ros2_msgs::msg::ColorRGB& msg)
{
    yarp::sig::ColorRGB color;
    color.r = static_cast<unsigned char>(msg.r);
    color.g = static_cast<unsigned char>(msg.g);
    color.b = static_cast<unsigned char>(msg.b);
    return color;
}
} // namespace

SimulatedWorld_nws_ros2::SimulatedWorld_nws_ros2() = default;

bool SimulatedWorld_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{
    yCInfo(SIMULATEDWORLD_NWS_ROS2, "Attaching to driver");
    if (driver && driver->isValid())
    {
        driver->view(m_iSim);
    }

    if (m_iSim == nullptr)
    {
        yCError(SIMULATEDWORLD_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    return true;
}

bool SimulatedWorld_nws_ros2::detach()
{
    m_iSim = nullptr;
    return true;
}

bool SimulatedWorld_nws_ros2::open(yarp::os::Searchable& config)
{
    yCInfo(SIMULATEDWORLD_NWS_ROS2, "Opening SimulatedWorld_nws_ros2");
    parseParams(config);

    if (!m_node_name.empty() && m_node_name[0] == '/')
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "node_name cannot begin with an initial /";
        return false;
    }

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    if (m_namespace.empty()) {
        m_node = NodeCreator::createNode(m_node_name, node_options);
    } else {
        m_node = NodeCreator::createNode(m_node_name, m_namespace, node_options);
    }
    if (m_node == nullptr) {
        yCError(SIMULATEDWORLD_NWS_ROS2) << " opening " << m_node_name << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }

    m_srv_makeSphere = m_node->create_service<simulated_world_nws_ros2_msgs::srv::MakeSphere>(
        m_makesphere, std::bind(&SimulatedWorld_nws_ros2::makeSphereCallback, this, _1, _2, _3));
    m_srv_makeBox = m_node->create_service<simulated_world_nws_ros2_msgs::srv::MakeBox>(
        m_makebox, std::bind(&SimulatedWorld_nws_ros2::makeBoxCallback, this, _1, _2, _3));
    m_srv_makeCylinder = m_node->create_service<simulated_world_nws_ros2_msgs::srv::MakeCylinder>(
        m_makecylinder, std::bind(&SimulatedWorld_nws_ros2::makeCylinderCallback, this, _1, _2, _3));
    m_srv_makeFrame = m_node->create_service<simulated_world_nws_ros2_msgs::srv::MakeFrame>(
        m_makeframe, std::bind(&SimulatedWorld_nws_ros2::makeFrameCallback, this, _1, _2, _3));
    m_srv_makeModel = m_node->create_service<simulated_world_nws_ros2_msgs::srv::MakeModel>(
        m_makemodel, std::bind(&SimulatedWorld_nws_ros2::makeModelCallback, this, _1, _2, _3));
    m_srv_changeColor = m_node->create_service<simulated_world_nws_ros2_msgs::srv::ChangeColor>(
        m_changecolor, std::bind(&SimulatedWorld_nws_ros2::changeColorCallback, this, _1, _2, _3));
    m_srv_setPose = m_node->create_service<simulated_world_nws_ros2_msgs::srv::SetPose>(
        m_setpose, std::bind(&SimulatedWorld_nws_ros2::setPoseCallback, this, _1, _2, _3));
    m_srv_enableGravity = m_node->create_service<simulated_world_nws_ros2_msgs::srv::EnableGravity>(
        m_enablegravity, std::bind(&SimulatedWorld_nws_ros2::enableGravityCallback, this, _1, _2, _3));
    m_srv_enableCollision = m_node->create_service<simulated_world_nws_ros2_msgs::srv::EnableCollision>(
        m_enablecollision, std::bind(&SimulatedWorld_nws_ros2::enableCollisionCallback, this, _1, _2, _3));
    m_srv_getPose = m_node->create_service<simulated_world_nws_ros2_msgs::srv::GetPose>(
        m_getpose, std::bind(&SimulatedWorld_nws_ros2::getPoseCallback, this, _1, _2, _3));
    m_srv_deleteObject = m_node->create_service<simulated_world_nws_ros2_msgs::srv::DeleteObject>(
        m_deleteobject, std::bind(&SimulatedWorld_nws_ros2::deleteObjectCallback, this, _1, _2, _3));
    m_srv_deleteAll = m_node->create_service<simulated_world_nws_ros2_msgs::srv::DeleteAll>(
        m_deleteall, std::bind(&SimulatedWorld_nws_ros2::deleteAllCallback, this, _1, _2, _3));
    m_srv_getList = m_node->create_service<simulated_world_nws_ros2_msgs::srv::GetList>(
        m_getlist, std::bind(&SimulatedWorld_nws_ros2::getListCallback, this, _1, _2, _3));
    m_srv_attach = m_node->create_service<simulated_world_nws_ros2_msgs::srv::Attach>(
        m_attach, std::bind(&SimulatedWorld_nws_ros2::attachCallback, this, _1, _2, _3));
    m_srv_detach = m_node->create_service<simulated_world_nws_ros2_msgs::srv::Detach>(
        m_detach, std::bind(&SimulatedWorld_nws_ros2::detachCallback, this, _1, _2, _3));
    m_srv_rename = m_node->create_service<simulated_world_nws_ros2_msgs::srv::Rename>(
        m_rename, std::bind(&SimulatedWorld_nws_ros2::renameCallback, this, _1, _2, _3));

    // Start the spinner
    m_spinner = new Ros2Spinner(m_node);
    m_spinner->start();

    yCInfo(SIMULATEDWORLD_NWS_ROS2) << "Waiting for device to attach";
    return true;
}

bool SimulatedWorld_nws_ros2::close()
{
    yCTrace(SIMULATEDWORLD_NWS_ROS2, "Close");

    // Stop and delete the spinner
    if (m_spinner) {
        m_spinner->stop();
        delete m_spinner;
        m_spinner = nullptr;
    }

    detach();
    rclcpp::shutdown();
    return true;
}

void SimulatedWorld_nws_ros2::makeSphereCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeSphere::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeSphere::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }
    if (request->frame_name == "None")
    {
        request->frame_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Frame name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->makeSphere(request->id, request->radius, toYarpPose(request->pose),
                                  toYarpColor(request->color), request->frame_name,
                                  request->gravity_enable, request->collision_enable);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::makeBoxCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeBox::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeBox::Response> response)
{

    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->frame_name == "None")
    {
        request->frame_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Frame name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
    }
    else
    {
        auto ret = m_iSim->makeBox(request->id, request->width, request->height, request->thickness,
                               toYarpPose(request->pose), toYarpColor(request->color),
                               request->frame_name, request->gravity_enable, request->collision_enable);
        response->success = static_cast<bool>(ret);
    }

}

void SimulatedWorld_nws_ros2::makeCylinderCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeCylinder::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeCylinder::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->frame_name == "None")
    {
        request->frame_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Frame name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->makeCylinder(request->id, request->radius, request->length,
                                    toYarpPose(request->pose), toYarpColor(request->color),
                                    request->frame_name, request->gravity_enable, request->collision_enable);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::makeFrameCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeFrame::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeFrame::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->frame_name == "None")
    {
        request->frame_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Frame name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->makeFrame(request->id, request->size, toYarpPose(request->pose),
                                 toYarpColor(request->color), request->frame_name,
                                 request->gravity_enable, request->collision_enable);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::makeModelCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeModel::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeModel::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }
    if (request->frame_name == "None")
    {
        request->frame_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Frame name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->makeModel(request->id, request->filename, toYarpPose(request->pose),
                                 request->frame_name, request->gravity_enable, request->collision_enable);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::changeColorCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::ChangeColor::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::ChangeColor::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->changeColor(request->id, toYarpColor(request->color));
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::setPoseCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::SetPose::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::SetPose::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->frame_name == "None")
    {
        request->frame_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Frame name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->setPose(request->id, toYarpPose(request->pose), request->frame_name);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::enableGravityCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableGravity::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableGravity::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }
    auto ret = m_iSim->enableGravity(request->id, request->enable);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::enableCollisionCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableCollision::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableCollision::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }
    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->enableCollision(request->id, request->enable);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::getPoseCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetPose::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetPose::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->frame_name == "None")
    {
        request->frame_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Frame name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    yarp::sig::Pose6D pose;
    auto ret = m_iSim->getPose(request->id, pose, request->frame_name);
    response->success = static_cast<bool>(ret);
    if (ret)
    {
        response->pose = toRosPose(pose);
    }
}

void SimulatedWorld_nws_ros2::deleteObjectCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteObject::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteObject::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }
     if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->deleteObject(request->id);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::deleteAllCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteAll::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteAll::Response> response)
{
    (void)request_header;
    (void)request;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    auto ret = m_iSim->deleteAll();
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::getListCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetList::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetList::Response> response)
{
    (void)request_header;
    (void)request;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    std::vector<std::string> names;
    auto ret = m_iSim->getList(names);
    response->success = static_cast<bool>(ret);
    if (ret)
    {
        response->names = names;
    }
}

void SimulatedWorld_nws_ros2::attachCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Attach::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Attach::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->link_name == "None")
    {
        request->link_name = "";
        yCWarning(SIMULATEDWORLD_NWS_ROS2) << "Link name was 'None', setting to empty string";
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
    }
    else
    {
        auto ret = m_iSim->attach(request->id, request->link_name);
        response->success = static_cast<bool>(ret);
    }
    
}

void SimulatedWorld_nws_ros2::detachCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Detach::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Detach::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }

    if (request->id == "None" || request->id.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid request ID";
        response->success = false;
        return;
    }

    auto ret = m_iSim->detach(request->id);
    response->success = static_cast<bool>(ret);
}

void SimulatedWorld_nws_ros2::renameCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Rename::Request> request,
    std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Rename::Response> response)
{
    (void)request_header;
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_iSim)
    {
        response->success = false;
        return;
    }
    if (request->old_name=="None" || request->old_name.empty() || request->new_name == "None" || request->new_name.empty())
    {
        yCError(SIMULATEDWORLD_NWS_ROS2) << "Invalid input parameters";
        response->success = false;
        return;
    }

    auto ret = m_iSim->rename(request->old_name, request->new_name);
    response->success = static_cast<bool>(ret);
}
