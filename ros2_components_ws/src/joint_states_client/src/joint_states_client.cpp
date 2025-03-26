#include <algorithm>
#include <iterator>
#include <set>
#include <cstdlib>
#include <memory>

#include "joint_states_client/joint_states_client.hpp"
#include "rcutils/logging_macros.h"

using namespace std::placeholders;

namespace yarp_devices_ros2_utils {

JointStatesClient::JointStatesClient(const std::string& name, const std::string& topicName) :
        rclcpp::Node(name)
{
    m_jointStatesTopicName = topicName;
    // Initialize subscriptions
    m_jointStateSubscription = this->create_subscription<sensor_msgs::msg::JointState>(m_jointStatesTopicName, 10,
                                                                                       std::bind(&JointStatesClient::_jointsStatesCallback,
                                                                                                 this, _1));
}

void JointStatesClient::_jointsStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);
    if(!m_firstDataReceived) m_firstDataReceived = true;
    //RCLCPP_INFO(this->get_logger(),"Got data");
    m_currentJointsStates = *msg;
}

bool JointStatesClient::_getElements(const std::vector<std::string>& axesNames,const std::vector<double>& toSearch, std::vector<double>& toFill)
{
    std::lock_guard<std::mutex> dataGuard(m_dataMutex);
    if(!m_firstDataReceived)
    {
        RCLCPP_ERROR(this->get_logger(),"No data received");
    }
    std::vector<std::string>::iterator itr;
    int index;
    toFill.clear();

    for(const auto& name : axesNames)
    {
        itr = std::find(m_currentJointsStates.name.begin(), m_currentJointsStates.name.end(), name);

        if (itr != m_currentJointsStates.name.end()) {
            index = std::distance(m_currentJointsStates.name.begin(), itr);
            toFill.push_back(toSearch[index]);
        }
        else {
            RCLCPP_ERROR(this->get_logger(),"Axis %s not found",name.c_str());
            return false;
        }
    }

    return true;
}

bool JointStatesClient::getPositions(const std::vector<std::string>& axesNames, std::vector<double>& positions)
{
    return _getElements(axesNames, m_currentJointsStates.position, positions);
}

bool JointStatesClient::getVelocities(const std::vector<std::string>& axesNames, std::vector<double>& velocities)
{
    return _getElements(axesNames, m_currentJointsStates.velocity, velocities);
}

bool JointStatesClient::getEfforts(const std::vector<std::string>& axesNames, std::vector<double>& efforts)
{
    return _getElements(axesNames,m_currentJointsStates.effort,efforts);
}

}