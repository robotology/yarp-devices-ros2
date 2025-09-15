#ifndef CB_HW_FW_POS_READ_POS_VEL__CB_HW_FW_POS_READ_POS_VEL_HPP_
#define CB_HW_FW_POS_READ_POS_VEL__CB_HW_FW_POS_READ_POS_VEL_HPP_

#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

//Custom ros2 interfaces
#include <yarp_control_msgs/srv/get_control_modes.hpp>
#include <yarp_control_msgs/srv/get_position.hpp>
#include <yarp_control_msgs/srv/get_velocity.hpp>
#include <yarp_control_msgs/srv/set_control_modes.hpp>
#include <yarp_control_msgs/srv/get_available_control_modes.hpp>
#include <yarp_control_msgs/srv/get_joints_names.hpp>
#include <yarp_control_msgs/msg/position.hpp>
#include <yarp_control_msgs/msg/velocity.hpp>
#include <yarp_control_msgs/msg/position_direct.hpp>

#include <joint_states_client/joint_states_client.hpp>

#include <mutex>

#include "cb_hw_fw_pos_read_pos_vel/visibility_control.h"

namespace cb_hw_fw_pos_read_pos_vel {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CbHwFwPosReadPosVel : public hardware_interface::SystemInterface
{
private:
    bool                                                        m_active{false};
    bool                                                        m_gotSomeData{false};
    bool                                                        m_continuousPosWrite{true};
    std::string                                                 m_nodeName;             // name of the rosNode
    std::string                                                 m_msgs_name;            // prefix for control_board_nws_ros2 services and topics
    mutable std::mutex                                          m_cmdMutex;
    std::vector<std::string>                                    m_jointNames; // name of the joints
    std::vector<size_t>                                         m_jointsIndexes; // the indexes of the involved joints involved
    rclcpp::executors::SingleThreadedExecutor                   m_executor;
    std::shared_ptr<yarp_devices_ros2_utils::JointStatesClient> m_jointStatesClient;

    // ControlBoard_nws_ros2 related topics and services names
    std::string  m_posTopicName;                 // Position commands topic
    std::string  m_getModesClientName;           // Service client for joints current control modes
    std::string  m_getPositionClientName;        // Service client to get current position values
    std::string  m_getJointsNamesClientName;     // Service client to get the available joints names

    // Command and state interfaces
    // Store the commands for the robot
    std::vector<double>          m_hwCommandsPositions;
    std::vector<double>          m_oldPositions; // This array has to be used in order to avoid sending
                                                 // the same position commans over and over
    std::vector<double>          m_hwStatesPositions;  // Vector for state interfaces position values
    std::vector<double>          m_hwStatesVelocities; // Vector for state interfaces velocity values
    std::vector<std::string>     m_modes; // Current joints control modes.
                                          // We have to assume that the joints do not change
                                          // control mode without this hw interface class knowing.
                                          // In other words, the only way to switch control mode
                                          // should be via "perform_command_mode_switch" method.

    // Ros2 related attributes
    rclcpp::Node::SharedPtr m_node;

    //  yarp_control_msgs clients and publisher
    rclcpp::Publisher<yarp_control_msgs::msg::Position>::SharedPtr      m_posPublisher;
    rclcpp::Client<yarp_control_msgs::srv::GetJointsNames>::SharedPtr   m_getJointsNamesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetControlModes>::SharedPtr  m_getControlModesClient;
    rclcpp::Client<yarp_control_msgs::srv::GetPosition>::SharedPtr      m_getPositionClient;

    // Internal functions
    bool _checkJoints(const std::vector<hardware_interface::ComponentInfo>& joints);
    CallbackReturn _initExportableInterfaces(const std::vector<hardware_interface::ComponentInfo>& joints);
    CallbackReturn _getHWCurrentValues();

public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CbHwFwPosReadPosVel)
    CbHwFwPosReadPosVel();
    ~CbHwFwPosReadPosVel();

    // SystemInterface virtual functions
    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /*
    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;

    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> & start_interfaces, const std::vector<std::string> & stop_interfaces) override;
    */

    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    CB_HW_FW_POS_READ_POS_VEL_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

};

}  // namespace cb_hw_fw_pos_read_pos_vel

#endif  // CB_HW_FW_POS_READ_POS_VEL__CB_HW_INTERFACE_HPP_
