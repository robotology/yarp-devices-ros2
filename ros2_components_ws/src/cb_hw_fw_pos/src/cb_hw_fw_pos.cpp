#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <chrono>
#include <cstdlib>
#include <memory>

#include "cb_hw_fw_pos/cb_hw_fw_pos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

namespace cb_hw_fw_pos
{

CbHwFwPos::CbHwFwPos()
{
}

CbHwFwPos::~CbHwFwPos()
{
}

bool CbHwFwPos::_checkJoints(const std::vector<hardware_interface::ComponentInfo>& joints)
{
    std::vector<std::string> all_joints;
    auto namesRequest = std::make_shared<yarp_control_msgs::srv::GetJointsNames::Request>();
    while (!m_getJointsNamesClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
        RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    }
    auto namesResponse = m_getJointsNamesClient->async_send_request(namesRequest);
    if(rclcpp::spin_until_future_complete(m_node, namesResponse) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(m_node->get_logger(), "Got joints names");
        all_joints = namesResponse.get()->names;
    }
    else {
        RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints names");
        return false;
    }

    for(const auto& joint : joints){
        if (std::find(all_joints.begin(), all_joints.end(), joint.name) == all_joints.end())
        {
            RCLCPP_FATAL(m_node->get_logger(),"The joint named %s was not found among the available ones",
                         joint.name.c_str());
            return false;
        }
        m_jointNames.push_back(joint.name);
    }
    return true;
}

CallbackReturn CbHwFwPos::_initExportableInterfaces(const std::vector<hardware_interface::ComponentInfo>& joints)
{
    if(!_checkJoints(joints))
    {
        RCLCPP_FATAL(m_node->get_logger(),"Unable to initialize the joints. Check the previous errors for more details");
        return CallbackReturn::ERROR;
    }
    m_hwCommandsPositions.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    m_oldPositions.resize(joints.size(), std::numeric_limits<double>::quiet_NaN());
    size_t i=0;

    for (const auto& joint : joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                m_node->get_logger(),
                "This device supports only POSITION command interfaces. Check again your hardware configuration");
            return CallbackReturn::ERROR;
        }

        m_hwCommandsPositions[i] = 0.0;
        m_oldPositions[i++] = 0.0;
    }

    return _getHWCurrentValues();
}

CallbackReturn CbHwFwPos::_getHWCurrentValues()
{
    auto posRequest = std::make_shared<yarp_control_msgs::srv::GetPosition::Request>();
    posRequest->names = m_jointNames;
    while (!m_getPositionClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(m_node->get_logger(), "service not available, waiting again...");
    }
    auto posFuture = m_getPositionClient->async_send_request(posRequest);
    auto posResponse = std::make_shared<yarp_control_msgs::srv::GetPosition::Response>();
    if(rclcpp::spin_until_future_complete(m_node, posFuture) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(m_node->get_logger(), "Got joints positions");
        posResponse = posFuture.get();
    }
    else {
        RCLCPP_ERROR(m_node->get_logger(),"Failed to get joints positions");
        return CallbackReturn::ERROR;
    }

    for (size_t i=0; i<m_jointNames.size(); i++)
    {
        m_hwCommandsPositions[i] = posResponse->positions[i];
        m_oldPositions[i] = posResponse->positions[i];
    }

    m_active = true;

    return CallbackReturn::SUCCESS;
}

CallbackReturn CbHwFwPos::on_init(const hardware_interface::HardwareInfo & info)
{
    if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    if(info.hardware_parameters.count("node_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwFwPos"),"No node name specified");
        return CallbackReturn::ERROR;
    }
    if(info.hardware_parameters.count("cb_nws_msgs_name")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwFwPos"),"No msgs name for the controlBoard_nws_ros2 specified");
        return CallbackReturn::ERROR;
    }
    if(info.hardware_parameters.count("continuous_pos_write")<=0)
    {
        RCLCPP_FATAL(rclcpp::get_logger("CbHwFwPos"),"No flag for the position continuous writing");
        return CallbackReturn::ERROR;
    }

    m_nodeName = info_.hardware_parameters["node_name"];
    m_msgs_name = info_.hardware_parameters["cb_nws_msgs_name"];
    m_continuousPosWrite = info_.hardware_parameters["continuous_pos_write"]==std::string("true") || info_.hardware_parameters["continuous_pos_write"]==std::string("True");

    m_node = rclcpp::Node::make_shared(m_nodeName);

    // Initialize topics and services names ------------------------------------------------------------------- //
    m_posTopicName = m_msgs_name+"/position";
    m_getModesClientName = m_msgs_name+"/get_modes";
    m_setModesClientName = m_msgs_name+"/set_modes";
    m_getPositionClientName = m_msgs_name+"/get_position";
    m_getAvailableModesClientName = m_msgs_name+"/get_available_modes";
    m_getJointsNamesClientName = m_msgs_name+"/get_joints_names";

    // Initialize publishers ---------------------------------------------------------------------------------- //
    m_posPublisher = m_node->create_publisher<yarp_control_msgs::msg::Position>(m_posTopicName, 10);
    if(!m_posPublisher){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Position publisher");
        return CallbackReturn::ERROR;
    }

    // Initialize services clients ---------------------------------------------------------------------------- //
    m_getJointsNamesClient = m_node->create_client<yarp_control_msgs::srv::GetJointsNames>(m_getJointsNamesClientName);
    if(!m_getJointsNamesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetJointsNames service client");
        return CallbackReturn::ERROR;
    }
    m_getControlModesClient = m_node->create_client<yarp_control_msgs::srv::GetControlModes>(m_getModesClientName);
    if(!m_getControlModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetControlModes service client");
        return CallbackReturn::ERROR;
    }
    m_getPositionClient = m_node->create_client<yarp_control_msgs::srv::GetPosition>(m_getPositionClientName);
    if(!m_getPositionClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetPosition service client");
        return CallbackReturn::ERROR;
    }
    m_setControlModesClient = m_node->create_client<yarp_control_msgs::srv::SetControlModes>(m_setModesClientName);
    if(!m_setControlModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the SetControlModes service client");
        return CallbackReturn::ERROR;
    }
    m_getAvailableModesClient = m_node->create_client<yarp_control_msgs::srv::GetAvailableControlModes>(m_getAvailableModesClientName);
    if(!m_getAvailableModesClient){
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetAvailableControlModes service client");
        return CallbackReturn::ERROR;
    }

    return _initExportableInterfaces(info_.joints);
}

std::vector<hardware_interface::StateInterface> CbHwFwPos::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> ifacesToReturn;
    return ifacesToReturn;
}

std::vector<hardware_interface::CommandInterface> CbHwFwPos::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ifacesToReturn;
    for (size_t i=0; i<m_hwCommandsPositions.size(); i++)
    {
        ifacesToReturn.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_hwCommandsPositions[i]));
    }

    return ifacesToReturn;
}

hardware_interface::return_type CbHwFwPos::prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CbHwFwPos::perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces)
{
    return hardware_interface::return_type::OK;
}

CallbackReturn CbHwFwPos::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    if(_getHWCurrentValues() == CallbackReturn::ERROR)
    {
        RCLCPP_ERROR(m_node->get_logger(),"Could not successfully read the current joints positions. Check previous errors for more info");
        return CallbackReturn::ERROR;
    }
    m_active = true;
    return CallbackReturn::SUCCESS;
}

CallbackReturn CbHwFwPos::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    m_active = false;
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type CbHwFwPos::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CbHwFwPos::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if(!m_active)
    {
        return hardware_interface::return_type::OK;
    }
    else if(m_hwCommandsPositions == m_oldPositions && !m_continuousPosWrite)
    {
        return hardware_interface::return_type::OK;
    }
    yarp_control_msgs::msg::Position posToSend;
    posToSend.names = m_jointNames;
    posToSend.positions = m_hwCommandsPositions;

    m_posPublisher->publish(posToSend);
    m_oldPositions = m_hwCommandsPositions;

    return hardware_interface::return_type::OK;
}

}  // namespace cb_hw_fw_pos

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cb_hw_fw_pos::CbHwFwPos, hardware_interface::SystemInterface)
