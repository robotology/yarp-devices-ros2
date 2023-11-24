#ifndef JOINT_STATES_CLIENT_HPP_
#define JOINT_STATES_CLIENT_HPP_

#include <string>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mutex>

namespace yarp_devices_ros2_utils {

class JointStatesClient : public rclcpp::Node
{
public:
    /**
     * @brief Constructor a new JointStatesCLient object
     */
    JointStatesClient(const std::string& name, const std::string& topicName);

    /**
     * @brief Position and velocity getters
     */
    bool getPositions(const std::vector<std::string>& axesNames, std::vector<double>& positions);
    bool getVelocities(const std::vector<std::string>& axesNames, std::vector<double>& velocities);
    bool getEfforts(const std::vector<std::string>& axesNames, std::vector<double>& efforts);

private:
    bool                          m_firstDataReceived{false};
    std::string                   m_jointStatesTopicName;         // Topic for joint states
    mutable std::mutex            m_dataMutex;
    sensor_msgs::msg::JointState  m_currentJointsStates;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  m_jointStateSubscription;

    bool _getElements(const std::vector<std::string>& axesNames,const std::vector<double>& toSearch, std::vector<double>& toFill);
    void _jointsStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}

#endif // JOINT_STATES_CLIENT_HPP_