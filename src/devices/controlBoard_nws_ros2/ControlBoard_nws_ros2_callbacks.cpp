/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "ControlBoard_nws_ros2.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <Ros2Utils.h>
#include <rcutils/logging_macros.h>

using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

namespace {
YARP_LOG_COMPONENT(CONTROLBOARD_ROS2, "yarp.ros2.controlBoard_nws_ros2", yarp::os::Log::TraceType);

// UTILITIES ----------------------------------------------------------------------------------------------------------- START //

// Maps for quick control mode conversion

const std::map<yarp::conf::vocab32_t,std::string> fromCtrlModeToString{{VOCAB_CM_IDLE,"IDLE"},
                                                                       {VOCAB_CM_TORQUE,"TORQUE"},
                                                                       {VOCAB_CM_POSITION,"POSITION"},
                                                                       {VOCAB_CM_POSITION_DIRECT,"POSITION_DIRECT"},
                                                                       {VOCAB_CM_VELOCITY,"VELOCITY"},
                                                                       {VOCAB_CM_CURRENT,"CURRENT"},
                                                                       {VOCAB_CM_PWM,"PWM"},
                                                                       {VOCAB_CM_IMPEDANCE_POS,"IMPEDANCE_POS"},
                                                                       {VOCAB_CM_IMPEDANCE_VEL,"IMPEDANCE_VEL"},
                                                                       {VOCAB_CM_MIXED,"MIXED"},
                                                                       {VOCAB_CM_HW_FAULT,"HW_FAULT"},
                                                                       {VOCAB_CM_CALIBRATING,"CALIBRATING"},
                                                                       {VOCAB_CM_CALIB_DONE,"CALIB_DONE"},
                                                                       {VOCAB_CM_NOT_CONFIGURED,"NOT_CONFIGURED"},
                                                                       {VOCAB_CM_CONFIGURED,"CONFIGURED"}
                                                                      };
const std::map<std::string,yarp::conf::vocab32_t> fromStringToCtrlMode{{"IDLE",VOCAB_CM_IDLE},
                                                                       {"TORQUE",VOCAB_CM_TORQUE},
                                                                       {"POSITION",VOCAB_CM_POSITION},
                                                                       {"POSITION_DIRECT",VOCAB_CM_POSITION_DIRECT},
                                                                       {"VELOCITY",VOCAB_CM_VELOCITY},
                                                                       {"CURRENT",VOCAB_CM_CURRENT},
                                                                       {"PWM",VOCAB_CM_PWM},
                                                                       {"IMPEDANCE_POS",VOCAB_CM_IMPEDANCE_POS},
                                                                       {"IMPEDANCE_VEL",VOCAB_CM_IMPEDANCE_VEL},
                                                                       {"MIXED",VOCAB_CM_MIXED},
                                                                       {"IDLE",VOCAB_CM_IDLE}
                                                                      };
const std::vector<std::string> implementedCtrlModes{"POSITION",
                                              "POSITION_DIRECT",
                                              "VELOCITY"};

inline double convertRadiansToDegrees(double degrees)
{
    return degrees / M_PI * 180.0;
}

inline double convertDegreesToRadians(double radians)
{
    return radians / 180.0 * M_PI;
}

} // namespace

// UTILITIES ------------------------------------------------------------------------------------------------------------- END //


bool ControlBoard_nws_ros2::messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values, const std::vector<double> &derivative){
    if(!namesCheck(names)) {
        return false;
    }

    bool allJoints = names.size() == 0 || names.size() == m_subdevice_joints;

    if(ref_values.size() == 0){
        yCError(CONTROLBOARD_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"%s vector cannot be empty",valueName.c_str());

        return false;
    }
    if(!allJoints && (names.size() != ref_values.size())){
        yCError(CONTROLBOARD_ROS2) << "The" << valueName << "vector and the names one are not the same size";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector and the names one are not the same size",valueName.c_str());

        return false;
    }

    bool noRef = derivative.size() == 0;
    if(!noRef && (ref_values.size() != derivative.size())){
        yCError(CONTROLBOARD_ROS2) << "The" << valueName << "vector and the secondary one are not the same size";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector and the secondary one are not the same size",valueName.c_str());

        return false;
    }
    bool allJointsFail = allJoints && (ref_values.size() != m_subdevice_joints || (!noRef && derivative.size() != m_subdevice_joints));

    if(allJointsFail){
        yCError(CONTROLBOARD_ROS2) << "All joints where selected bt the vector sizes do not match";
        RCLCPP_ERROR(m_node->get_logger(),"All joints where selected bt the vector sizes do not match");

        return false;
    }

    return true;
}


bool ControlBoard_nws_ros2::messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values){
    if(!namesCheck(names)) {
        return false;
    }

    bool allJoints = names.size() == 0 || names.size() == m_subdevice_joints;
    bool diffValNames = !allJoints && (names.size() != ref_values.size());
    bool emptyValues = ref_values.size() == 0;

    if(emptyValues){
        yCError(CONTROLBOARD_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector cannot be empty",valueName.c_str());

        return false;
    }

    bool allJointsFail = allJoints && ref_values.size() != m_subdevice_joints;

    return !diffValNames && !allJointsFail;
}


bool ControlBoard_nws_ros2::namesCheck(const std::vector<std::string> &names){
    if(names.size() > m_subdevice_joints){
        yCError(CONTROLBOARD_ROS2) << "The specified joint names vector is longer than expected";
        RCLCPP_ERROR(m_node->get_logger(),"The specified joint names vector is longer than expected");

        return false;
    }
    for (const auto& name : names){
        if(m_quickJointRef.count(name) == 0){
            yCError(CONTROLBOARD_ROS2) << name << "is not a valid joint name";
            RCLCPP_ERROR(m_node->get_logger(),"%s is not a valid joint name",name.c_str());

            return false;
        }
    }

    return true;
}


bool ControlBoard_nws_ros2::messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<std::string> &ref_values){
    if(!namesCheck(names)) {
        return false;
    }

    bool allJoints = names.size() == 0 || names.size() == m_subdevice_joints;
    bool diffValNames = !allJoints && (names.size() != ref_values.size());
    bool emptyValues = ref_values.size() == 0;

    if(emptyValues){
        yCError(CONTROLBOARD_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector cannot be empty",valueName.c_str());

        return false;
    }

    bool allJointsFail = allJoints && ref_values.size() != m_subdevice_joints;

    return !diffValNames && !allJointsFail;
}


void ControlBoard_nws_ros2::positionTopic_callback(const yarp_control_msgs::msg::Position::SharedPtr msg) {

    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = msg->names.size() == 0;
    bool noSpeed = msg->ref_velocities.size() == 0;

    if(!msg){
        yCError(CONTROLBOARD_ROS2) << "Invalid message";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid message");

        return;
    }
    if(!messageVectorsCheck("Position",msg->names,msg->positions,msg->ref_velocities)){

        return;
    }

    double tempVel;
    double tempPos;
    JointTypeEnum jType;
    std::vector<double> convertedPos;
    std::vector<int> selectedJoints;
    std::vector<double> convertedVel;

    for(size_t i=0; i<(noJoints ? m_subdevice_joints : msg->positions.size()); i++){
        size_t index = noJoints ? i : m_quickJointRef[msg->names[i]];
        if(!noJoints) {selectedJoints.push_back(index);}
        m_iAxisInfo->getJointType(index, jType);
        if(!noSpeed){
            if(jType == VOCAB_JOINTTYPE_REVOLUTE){
                tempVel = convertRadiansToDegrees(msg->ref_velocities[i]);
            }
            else{
                tempVel = msg->ref_velocities[i];
            }
            convertedVel.push_back(tempVel);
        }

        if(jType == VOCAB_JOINTTYPE_REVOLUTE){
            tempPos = convertRadiansToDegrees(msg->positions[i]);
        }
        else{
            tempPos = msg->positions[i];
        }
        convertedPos.push_back(tempPos);
    }
    if(noJoints){
        if(!noSpeed){
            m_iPositionControl->setRefSpeeds(&convertedVel[0]);
        }
        m_iPositionControl->positionMove(&convertedPos[0]);
    }
    else{
        if(!noSpeed){
            m_iPositionControl->setRefSpeeds(convertedPos.size(),&selectedJoints[0],&convertedVel[0]);
        }
        m_iPositionControl->positionMove(convertedPos.size(),&selectedJoints[0],&convertedPos[0]);
    }
}


void ControlBoard_nws_ros2::positionDirectTopic_callback(const yarp_control_msgs::msg::PositionDirect::SharedPtr msg) {
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = msg->names.size() == 0;

    if(!msg){
        yCError(CONTROLBOARD_ROS2) << "Invalid message";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid message");

        return;
    }

    if(!messageVectorsCheck("Position",msg->names,msg->positions)){

        return;
    }

    double tempPos;
    JointTypeEnum jType;
    std::vector<double> convertedPos;
    std::vector<int> selectedJoints;

    for(size_t i=0; i<noJoints ? m_subdevice_joints : msg->positions.size(); i++){
        size_t index = noJoints ? i : m_quickJointRef[msg->names[i]];
        if(!noJoints) {selectedJoints.push_back(index);}
        m_iAxisInfo->getJointType(index, jType);
        if(jType == VOCAB_JOINTTYPE_REVOLUTE){
            tempPos = convertRadiansToDegrees(msg->positions[i]);
        }
        else{
            tempPos = msg->positions[i];
        }
        convertedPos.push_back(tempPos);
    }

    if(noJoints){
        m_iPositionDirect->setPositions(&convertedPos[0]);
    }
    else{
        m_iPositionDirect->setPositions(convertedPos.size(),&selectedJoints[0],&convertedPos[0]);
    }
}


void ControlBoard_nws_ros2::velocityTopic_callback(const yarp_control_msgs::msg::Velocity::SharedPtr msg) {

    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = msg->names.size() == 0;
    bool noAccel = msg->ref_accelerations.size() == 0;

    if(!msg){
        yCError(CONTROLBOARD_ROS2) << "Invalid message";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid message");

        return;
    }
    if(!messageVectorsCheck("Velocities",msg->names,msg->velocities,msg->ref_accelerations)){

        return;
    }

    double tempVel;
    double tempAccel;
    JointTypeEnum jType;
    std::vector<double> convertedVel;
    std::vector<int> selectedJoints;
    std::vector<double> convertedAccel;

    for(size_t i=0; i<noJoints ? m_subdevice_joints : msg->velocities.size(); i++){
        size_t index = noJoints ? i : m_quickJointRef[msg->names[i]];
        if(!noJoints) {selectedJoints.push_back(index);}
        m_iAxisInfo->getJointType(index, jType);
        if(!noAccel){
            if(jType == VOCAB_JOINTTYPE_REVOLUTE){
                tempAccel = convertRadiansToDegrees(msg->ref_accelerations[i]);
            }
            else{
                tempAccel = msg->ref_accelerations[i];
            }
            convertedAccel.push_back(tempAccel);
        }
        if(jType == VOCAB_JOINTTYPE_REVOLUTE){
            tempVel = convertRadiansToDegrees(msg->velocities[i]);
        }
        else{
            tempVel = msg->velocities[i];
        }
        convertedVel.push_back(tempVel);
    }
    if(noJoints){
        if(!noAccel){
            m_iVelocityControl->setRefAccelerations(&convertedAccel[0]);
        }
        m_iVelocityControl->velocityMove(&convertedVel[0]);
    }
    else{
        if(!noAccel){
            m_iVelocityControl->setRefAccelerations(convertedAccel.size(),&selectedJoints[0],&convertedAccel[0]);
        }
        m_iVelocityControl->velocityMove(convertedVel.size(),&selectedJoints[0],&convertedVel[0]);
    }
}


void ControlBoard_nws_ros2::getJointsNamesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                   const std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Request> request,
                                                   std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);
    if(!request){
        yCError(CONTROLBOARD_ROS2) << "Invalid request";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

        response->response = "INVALID";

        return;
    }

    bool noIndexes = request->joint_indexes.size() == 0;

    if(!noIndexes && request->joint_indexes.size() > m_subdevice_joints){
        yCError(CONTROLBOARD_ROS2) << "request->joint_indexes vector cannot be longer than the actual number of joints:"<< request->joint_indexes.size() << "instead of" << m_subdevice_joints;
        RCLCPP_ERROR(m_node->get_logger(),"request->joint_indexes vector cannot be longer than the actual number of joints: %ld instead of %ld", request->joint_indexes.size(), m_subdevice_joints);

        response->response = "SIZE_ERROR";

        return;
    }

    std::string tempName;
    if(noIndexes){
        for(size_t i=0; i<m_subdevice_joints; i++){
            if(!m_iAxisInfo->getAxisName(i,tempName)){
                yCError(CONTROLBOARD_ROS2) << "Name retrieval failed for joint number"<<i;
                RCLCPP_ERROR(m_node->get_logger(),"Name retrieval failed for joint number %ld",i);

                response->response = "GET_NAME_ERROR";

                return;
            }
            response->names.push_back(tempName);
        }
    }
    else{
        for(const auto &i : request->joint_indexes){
            if(!m_iAxisInfo->getAxisName(i,tempName)){
                yCError(CONTROLBOARD_ROS2) << "Name retrieval failed for joint number"<<i;
                RCLCPP_ERROR(m_node->get_logger(),"Name retrieval failed for joint number %d",i);

                response->response = "GET_NAME_ERROR";

                return;
            }
            response->names.push_back(tempName);
        }
    }

    response->response = "OK";
}


void ControlBoard_nws_ros2::getAvailableModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                      const std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Request> request,
                                                      std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);
    if(!request){
        yCError(CONTROLBOARD_ROS2) << "Invalid request";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

        response->response = "INVALID";

        return;
    }

    if(request->only_implemented){
        response->modes = implementedCtrlModes;
    }
    else{
        for(const auto &x : fromStringToCtrlMode){
            response->modes.push_back(x.first);
        }
    }

    response->response = "OK";
}


void ControlBoard_nws_ros2::getControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                    const std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Request> request,
                                                    std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = request->names.size() == 0;

    if(!request){
        yCError(CONTROLBOARD_ROS2) << "Invalid request";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

        response->response = "INVALID";

        return;
    }

    if(!noJoints){
        if(!namesCheck(request->names)){
            response->response = "NAMES_ERROR";

            return;
        }
    }

    size_t forLimit = noJoints ? m_subdevice_joints : request->names.size();
    int *tempMode = new int[1];
    std::vector<std::string> modesToSend;

    for (size_t i=0; i<forLimit; i++){

        if(!m_iControlMode->getControlMode(noJoints ? i : m_quickJointRef[request->names[i]],tempMode)){
            yCError(CONTROLBOARD_ROS2) << "Error while retrieving the control mode for joint"<<request->names[i];
            RCLCPP_ERROR(m_node->get_logger(),"Error while retrieving the control mode for joint %s",request->names[i].c_str());
            response->response = "RETRIEVE_ERROR";

            delete tempMode;
            return;
        }
        modesToSend.push_back(fromCtrlModeToString.at(*tempMode));
    }
    response->modes = modesToSend;
    response->response = "OK";

    delete tempMode;
}


void ControlBoard_nws_ros2::getPositionCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                const std::shared_ptr<yarp_control_msgs::srv::GetPosition::Request> request,
                                                std::shared_ptr<yarp_control_msgs::srv::GetPosition::Response> response) {
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = request->names.size() == 0;

    if(!request){
        yCError(CONTROLBOARD_ROS2) << "Invalid request";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

        response->response = "INVALID";

        return;
    }

    if(!noJoints){
        if(!namesCheck(request->names)){
            response->response = "NAMES_ERROR";

            return;
        }
    }

    size_t forLimit = noJoints ? m_subdevice_joints : request->names.size();
    double *tempPos = new double[m_jointNames.size()];
    std::vector<double> positionsToSend;

    if(!m_iEncodersTimed->getEncoders(tempPos)){
        yCError(CONTROLBOARD_ROS2) << "Error while retrieving joints positions";
        RCLCPP_ERROR(m_node->get_logger(),"Error while retrieving joints positions");
        response->response = "RETRIEVE_ERROR";

        delete tempPos;
        return;
    }

    double position;
    JointTypeEnum jType;
    for (size_t i=0; i<forLimit; i++){
        size_t index = noJoints ? i : m_quickJointRef[request->names[i]];
        m_iAxisInfo->getJointType(index, jType);
        if(jType == VOCAB_JOINTTYPE_REVOLUTE)
        {
            position = convertDegreesToRadians(tempPos[index]);
        }
        else
        {
            position = tempPos[index];
        }
        positionsToSend.push_back(position);
    }
    response->positions = positionsToSend;
    response->response = "OK";

    delete tempPos;
}


void ControlBoard_nws_ros2::getVelocityCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                const std::shared_ptr<yarp_control_msgs::srv::GetVelocity::Request> request,
                                                std::shared_ptr<yarp_control_msgs::srv::GetVelocity::Response> response) {
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = request->names.size() == 0;

    if(!request){
        yCError(CONTROLBOARD_ROS2) << "Invalid request";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

        response->response = "INVALID";

        return;
    }

    if(!noJoints){
        if(!namesCheck(request->names)){
            response->response = "NAMES_ERROR";

            return;
        }
    }

    size_t forLimit = noJoints ? m_subdevice_joints : request->names.size();
    double *tempVel = new double[m_jointNames.size()];
    std::vector<double> velocitiesToSend;

    if(!m_iEncodersTimed->getEncoderSpeeds(tempVel)){
        yCError(CONTROLBOARD_ROS2) << "Error while retrieving joints speeds";
        RCLCPP_ERROR(m_node->get_logger(),"Error while retrieving joints speeds");
        response->response = "RETRIEVE_ERROR";

        delete tempVel;
        return;
    }

    double velocity;
    JointTypeEnum jType;
    for (size_t i=0; i<forLimit; i++){
        size_t index = noJoints ? i : m_quickJointRef[request->names[i]];
        m_iAxisInfo->getJointType(index, jType);
        if(jType == VOCAB_JOINTTYPE_REVOLUTE)
        {
            velocity = convertDegreesToRadians(tempVel[index]);
        }
        else
        {
            velocity = tempVel[index];
        }
        velocitiesToSend.push_back(velocity);
    }
    response->velocities = velocitiesToSend;
    response->response = "OK";

    delete tempVel;
}


void ControlBoard_nws_ros2::setControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                    const std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Request> request,
                                                    std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = request->names.size() == 0;

    if(!request){
        yCError(CONTROLBOARD_ROS2) << "Invalid request";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

        response->response = "INVALID";

        return;
    }

    if(!messageVectorsCheck("Control mode",request->names,request->modes)){

        response->response = "INVALID";

        return;
    }

    size_t forLimit = noJoints ? m_subdevice_joints : request->names.size();

    for (size_t i=0; i<forLimit; i++){
        if(!fromStringToCtrlMode.count(request->modes[i])){
            yCError(CONTROLBOARD_ROS2) << "Cannot set mode to" << request->modes[i];
            RCLCPP_ERROR(m_node->get_logger(),"Cannot set mode to %s",request->modes[i].c_str());

            return;
        }
        if(!m_iControlMode->setControlMode(noJoints ? i : m_quickJointRef[request->names[i]],fromStringToCtrlMode.at(request->modes[i]))){
            yCError(CONTROLBOARD_ROS2) << "Error while setting the control mode for joint"<<request->names[i]<<"to"<<request->modes[i];
            RCLCPP_ERROR(m_node->get_logger(),"Error while setting the control mode for joint %s to %s",request->names[i].c_str(),request->modes[i].c_str());
            response->response = "SET_ERROR";

            return;
        }
    }

    response->response = "OK";
}
