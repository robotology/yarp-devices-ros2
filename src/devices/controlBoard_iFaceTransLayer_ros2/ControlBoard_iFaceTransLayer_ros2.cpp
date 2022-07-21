/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "ControlBoard_iFaceTransLayer_ros2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <cmath>
#include <Ros2Utils.h>
#include <rcutils/logging_macros.h>

YARP_LOG_COMPONENT(CONTROLBOARD_IFACETRANSLAYER_ROS2, "yarp.devices.ControlBoard_iFaceTransLayer_ros2")

// UTILITIES ----------------------------------------------------------------------------------------------------------- START //

// Maps for quick control mode conversion

std::map<yarp::conf::vocab32_t,std::string> fromCtrlModeToString{{VOCAB_CM_IDLE,"IDLE"},
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
std::map<std::string,yarp::conf::vocab32_t> fromStringToCtrlMode{{"IDLE",VOCAB_CM_IDLE},
                                                                 {"TORQUE",VOCAB_CM_TORQUE},
                                                                 {"POSITION",VOCAB_CM_POSITION},
                                                                 {"POSITION_DIRECT",VOCAB_CM_POSITION_DIRECT},
                                                                 {"VELOCITY",VOCAB_CM_VELOCITY},
                                                                 {"CURRENT",VOCAB_CM_CURRENT},
                                                                 {"PWM",VOCAB_CM_PWM},
                                                                 {"IMPEDANCE_POS",VOCAB_CM_IMPEDANCE_POS},
                                                                 {"IMPEDANCE_VEL",VOCAB_CM_IMPEDANCE_VEL},
                                                                 {"MIXED",VOCAB_CM_MIXED}
                                                                };
std::vector<std::string> implementedCtrlModes{"POSITION",
                                              "POSITION_DIRECT",
                                              "VELOCITY"};

// Simple utility function to replace all occurrences of a certain substring
std::string replace_all(const std::string& s, std::string const& toReplace, std::string const& replaceWith) {
    std::ostringstream oss;
    std::size_t pos = 0;
    std::size_t prevPos = pos;
    std::string toReturn;

    while (true) {
        prevPos = pos;
        pos = s.find(toReplace, pos);
        if (pos == std::string::npos)
            break;
        oss << s.substr(prevPos, pos - prevPos);
        oss << replaceWith;
        pos += toReplace.size();
    }

    oss << s.substr(prevPos);
    toReturn = oss.str();

    return toReturn;
}

// UTILITIES ------------------------------------------------------------------------------------------------------------- END //

ControlBoard_iFaceTransLayer_ros2::ControlBoard_iFaceTransLayer_ros2()
{
}


ControlBoard_iFaceTransLayer_ros2::~ControlBoard_iFaceTransLayer_ros2()
{
}


bool ControlBoard_iFaceTransLayer_ros2::open(yarp::os::Searchable &config)
{
    if (!config.check("name")) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "missing name parameter";
        return false;
    }
    m_name = config.find("name").asString();
    if (m_name[0] == '/') {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "name parameter cannot begin with '/'";
        return false;
    }

    m_nodeName = replace_all(m_name,"/","_")+"_node";
    m_posTopicName = "/"+m_name+"/position";
    m_posDirTopicName = "/"+m_name+"/position_direct";
    m_rosPosTopicName = "/"+m_name+"/ros_position";
    m_velTopicName = "/"+m_name+"/velocity";
    m_rosVelTopicName = "/"+m_name+"/ros_velocity";
    m_getModesSrvName = "/"+m_name+"/get_modes";
    m_setModesSrvName = "/"+m_name+"/set_modes";
    m_getAvailableModesSrvName = "/"+m_name+"/get_available_modes";
    m_getJointsNamesSrvName = "/"+m_name+"/get_joints_names";

    // Check if we need to create subdevice or if they are
    // passed later on thorugh attach()
    if (config.check("subdevice")) {
        config.setMonitor(config.getMonitor());
        yarp::os::Property prop;
        prop.fromString(config.toString());
        if (!openAndAttachSubDevice(prop)) {
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "Error while opening subdevice");
            return false;
        }
        m_subdevice_ready = true;
    }
    
    // Creating node --------------------------------------------------------------------------------------------------- //
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    m_node = NodeCreator::createNode(m_nodeName, node_options);
    if (m_node == nullptr) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << " opening " << m_nodeName << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }

    // Creating topics ------------------------------------------------------------------------------------------------- //

    m_posSubscription = m_node->create_subscription<yarp_control_msgs::msg::Position>(m_posTopicName, 10,
                                                                                      std::bind(&ControlBoard_iFaceTransLayer_ros2::positionTopic_callback,
                                                                                      this, std::placeholders::_1));
    m_posDirectSubscription = m_node->create_subscription<yarp_control_msgs::msg::PositionDirect>(m_posDirTopicName, 10,
                                                                                                  std::bind(&ControlBoard_iFaceTransLayer_ros2::positionDirectTopic_callback,
                                                                                                            this, std::placeholders::_1));
    m_velSubscription = m_node->create_subscription<yarp_control_msgs::msg::Velocity>(m_velTopicName, 10,
                                                                                      std::bind(&ControlBoard_iFaceTransLayer_ros2::velocityTopic_callback,
                                                                                      this, std::placeholders::_1));
    m_rosPosSubscription = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(m_rosPosTopicName, 10,
                                                                                         std::bind(&ControlBoard_iFaceTransLayer_ros2::positionGet_callback,
                                                                                         this, std::placeholders::_1));
    m_rosVelSubscription = m_node->create_subscription<std_msgs::msg::Float64MultiArray>(m_rosVelTopicName, 10,
                                                                                         std::bind(&ControlBoard_iFaceTransLayer_ros2::velocityGet_callback,
                                                                                         this, std::placeholders::_1));

    // Creating services ----------------------------------------------------------------------------------------------- //

    rmw_qos_profile_t qos;
    qos.history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    qos.depth=10;
    rmw_time_t time;
    time.sec=10000;
    time.nsec = 0;
    qos.deadline= time;
    qos.lifespan=time;
    qos.liveliness_lease_duration=time;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    qos.avoid_ros_namespace_conventions = true;
    m_getControlModesSrv = m_node->create_service<yarp_control_msgs::srv::GetControlModes>(m_getModesSrvName,
                                                                                           std::bind(&ControlBoard_iFaceTransLayer_ros2::getControlModesCallback,
                                                                                                     this,std::placeholders::_1,std::placeholders::_2,
                                                                                                     std::placeholders::_3));
    m_setControlModesSrv = m_node->create_service<yarp_control_msgs::srv::SetControlModes>(m_setModesSrvName,
                                                                                           std::bind(&ControlBoard_iFaceTransLayer_ros2::setControlModesCallback,
                                                                                                     this,std::placeholders::_1,std::placeholders::_2,
                                                                                                     std::placeholders::_3));
    m_getAvailableModesSrv = m_node->create_service<yarp_control_msgs::srv::GetAvailableControlModes>(m_getAvailableModesSrvName,
                                                                                                      std::bind(&ControlBoard_iFaceTransLayer_ros2::getAvailableModesCallback,
                                                                                                                this,std::placeholders::_1,std::placeholders::_2,
                                                                                                                std::placeholders::_3));
    m_getJointsNamesSrv = m_node->create_service<yarp_control_msgs::srv::GetJointsNames>(m_getJointsNamesSrvName,
                                                                                         std::bind(&ControlBoard_iFaceTransLayer_ros2::getJointsNamesCallback,
                                                                                                   this,std::placeholders::_1,std::placeholders::_2,
                                                                                                   std::placeholders::_3));

	// In case attach is not deferred and the controlboard already owns a valid device
    // we can start the thread. Otherwise this will happen when attach is called
    if (m_subdevice_ready) {
        if (!start()) {
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error starting thread";
            return false;
        }
    }

    return true;
}


void ControlBoard_iFaceTransLayer_ros2::run()
{
	rclcpp::spin(m_node);

    return;
}


bool ControlBoard_iFaceTransLayer_ros2::messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values, const std::vector<double> &derivative){
    if(!namesCheck(names)) {
        return false;
    }
    
    bool allJoints = names.size() == 0 || names.size() == m_subdevice_joints;

    if(ref_values.size() == 0){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"%s vector cannot be empty",valueName.c_str());

        return false;
    }
    if(!allJoints && (names.size() != ref_values.size())){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "The" << valueName << "vector and the names one are not the same size";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector and the names one are not the same size",valueName.c_str());

        return false;
    }

    bool noRef = derivative.size() == 0;
    if(!noRef && (ref_values.size() != derivative.size())){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "The" << valueName << "vector and the secondary one are not the same size";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector and the secondary one are not the same size",valueName.c_str());

        return false;
    }
    bool allJointsFail = allJoints && (ref_values.size() != m_subdevice_joints || (!noRef && derivative.size() != m_subdevice_joints));

    if(allJointsFail){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "All joints where selected bt the vector sizes do not match";
        RCLCPP_ERROR(m_node->get_logger(),"All joints where selected bt the vector sizes do not match");

        return false;
    }

    return true;
}


bool ControlBoard_iFaceTransLayer_ros2::messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values){
    if(!namesCheck(names)) {
        return false;
    }

    bool allJoints = names.size() == 0 || names.size() == m_subdevice_joints;
    bool diffValNames = !allJoints && (names.size() != ref_values.size());
    bool emptyValues = ref_values.size() == 0;

    if(emptyValues){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector cannot be empty",valueName.c_str());

        return false;
    }

    bool allJointsFail = allJoints && ref_values.size() != m_subdevice_joints;

    return !diffValNames && !allJointsFail;
}


bool ControlBoard_iFaceTransLayer_ros2::namesCheck(const std::vector<std::string> &names){
    if(names.size() > m_subdevice_joints){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "The specified joint names vector is longer than expected";
        RCLCPP_ERROR(m_node->get_logger(),"The specified joint names vector is longer than expected");

        return false;
    }
    for (const auto& name : names){
        if(m_quickJointRef.count(name) == 0){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << name << "is not a valid joint name";
            RCLCPP_ERROR(m_node->get_logger(),"%s is not a valid joint name",name.c_str());

            return false;
        }
    }

    return true;
}


bool ControlBoard_iFaceTransLayer_ros2::messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<std::string> &ref_values){
    if(!namesCheck(names)) {
        return false;
    }

    bool allJoints = names.size() == 0 || names.size() == m_subdevice_joints;
    bool diffValNames = !allJoints && (names.size() != ref_values.size());
    bool emptyValues = ref_values.size() == 0;

    if(emptyValues){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector cannot be empty",valueName.c_str());

        return false;
    }

    bool allJointsFail = allJoints && ref_values.size() != m_subdevice_joints;

    return !diffValNames && !allJointsFail;
}


void ControlBoard_iFaceTransLayer_ros2::positionTopic_callback(const yarp_control_msgs::msg::Position::SharedPtr msg) {

    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = msg->names.size() == 0;
    bool noSpeed = msg->ref_velocities.size() == 0;

    if(!msg){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Invalid message";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid message");

        return;
    }
    if(!messageVectorsCheck("Position",msg->names,msg->positions,msg->ref_velocities)){

        return;
    }

    bool *done = new bool[1];
    if(noJoints){
        if (!m_iPositionControl->checkMotionDone(done)){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Communication error on checking motion done";
            RCLCPP_ERROR(m_node->get_logger(),"Communication error on checking motion done");
            
            delete done;
            return;
        }
        if(!done[0]){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Cannot start a new movement while another one is still being preformed";
            RCLCPP_ERROR(m_node->get_logger(),"Cannot start a new movement while another one is still being preformed");

            delete done;
            return;
        }
        if (!noSpeed){
            if(!m_iPositionControl->setRefSpeeds(&msg->ref_velocities[0])){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the reference velocities";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference velocities");
                
                delete done;
                return;
            }
        }
        if(!m_iPositionControl->positionMove(&msg->positions[0])){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the positions";
            RCLCPP_ERROR(m_node->get_logger(),"Error in setting the positions");

            delete done;
            return;
        }
    }
    else{
        for(size_t i=0; i<msg->positions.size(); i++){
            size_t index = m_quickJointRef[msg->names[i]];
            if (!m_iPositionControl->checkMotionDone(index,done)){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Communication error on checking motion done";
                RCLCPP_ERROR(m_node->get_logger(),"Communication error on checking motion done");

                delete done;
                return;
            }
            if(!done[0]){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Cannot start a new movement while another one is still being preformed";
                RCLCPP_ERROR(m_node->get_logger(),"Cannot start a new movement while another one is still being preformed");

                delete done;
                return;
            }
            if(!noSpeed){
                if(!m_iPositionControl->setRefSpeed(index,msg->ref_velocities[i])){
                    yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the reference velocity";
                    RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference velocity");

                    delete done;
                    return;
                }
            }
            if(!m_iPositionControl->positionMove(index,msg->positions[i])){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the position";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the position");

                delete done;
                return;
            }
        }
    }

    delete done;
}


void ControlBoard_iFaceTransLayer_ros2::positionDirectTopic_callback(const yarp_control_msgs::msg::PositionDirect::SharedPtr msg) {
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = msg->names.size() == 0;

    if(!msg){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Invalid message";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid message");

        return;
    }

    if(!messageVectorsCheck("Position",msg->names,msg->positions)){

        return;
    }

    if(noJoints){
        if(!m_iPositionDirect->setPositions(&msg->positions[0])){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the positions";
            RCLCPP_ERROR(m_node->get_logger(),"Error in setting the positions");

            return;
        }
    }
    else{
        for(size_t i=0; i<msg->positions.size(); i++){
            size_t index = m_quickJointRef[msg->names[i]];
            if(!m_iPositionDirect->setPosition(index,msg->positions[i])){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the position";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the position");

                return;
            }
        }
    }
}


void ControlBoard_iFaceTransLayer_ros2::velocityTopic_callback(const yarp_control_msgs::msg::Velocity::SharedPtr msg) {
    
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = msg->names.size() == 0;
    bool noAccel = msg->ref_accelerations.size() == 0;

    if(!msg){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Invalid message";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid message");

        return;
    }
    if(!messageVectorsCheck("Velocities",msg->names,msg->velocities,msg->ref_accelerations)){

        return;
    }

    if(noJoints){
        if (!noAccel){
            if(!m_iVelocityControl->setRefAccelerations(&msg->ref_accelerations[0])){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the reference accelerations";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference accelerations");

                return;
            }
        }
        if(!m_iVelocityControl->velocityMove(&msg->velocities[0])){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the velocities";
            RCLCPP_ERROR(m_node->get_logger(),"Error in setting the velocities");

            return;
        }
    }
    else{
        for(size_t i=0; i<msg->velocities.size(); i++){
            size_t index = m_quickJointRef[msg->names[i]];
            if(!noAccel){
                if(!m_iVelocityControl->setRefAcceleration(index,msg->ref_accelerations[i])){
                    yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the reference acceleration";
                    RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference acceleration");

                    return;
                }
            }
            if(!m_iVelocityControl->velocityMove(index,msg->velocities[i])){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error in setting the velocitie";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the velocitie");

                return;
            }
        }
    }
}


void ControlBoard_iFaceTransLayer_ros2::getJointsNamesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                   const std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Request> request,
                                                   std::shared_ptr<yarp_control_msgs::srv::GetJointsNames::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);
    if(!request){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Invalid request";
        RCLCPP_ERROR(m_node->get_logger(),"Invalid request");

        response->response = "INVALID";

        return;
    }

    bool noIndexes = request->joint_indexes.size() == 0;

    if(!noIndexes && request->joint_indexes.size() > m_subdevice_joints){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "request->joint_indexes vector cannot be longer than the actual number of joints:"<< request->joint_indexes.size() << "instead of" << m_subdevice_joints;
        RCLCPP_ERROR(m_node->get_logger(),"request->joint_indexes vector cannot be longer than the actual number of joints: %ld instead of %ld", request->joint_indexes.size(), m_subdevice_joints);

        response->response = "SIZE_ERROR";

        return;
    }

    std::string tempName;
    if(noIndexes){
        for(size_t i=0; i<m_subdevice_joints; i++){
            if(!m_iAxisInfo->getAxisName(i,tempName)){
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Name retrieval failed for joint number"<<i;
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
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Name retrieval failed for joint number"<<i;
                RCLCPP_ERROR(m_node->get_logger(),"Name retrieval failed for joint number %d",i);

                response->response = "GET_NAME_ERROR";

                return;
            }
            response->names.push_back(tempName);
        }
    }

    response->response = "OK";
}


void ControlBoard_iFaceTransLayer_ros2::getAvailableModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                      const std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Request> request,
                                                      std::shared_ptr<yarp_control_msgs::srv::GetAvailableControlModes::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);
    if(!request){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Invalid request";
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


void ControlBoard_iFaceTransLayer_ros2::getControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                    const std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Request> request,
                                                    std::shared_ptr<yarp_control_msgs::srv::GetControlModes::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = request->names.size() == 0;

    if(!request){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Invalid request";
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
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error while retrieving the control mode for joint"<<request->names[i];
            RCLCPP_ERROR(m_node->get_logger(),"Error while retrieving the control mode for joint %s",request->names[i].c_str());
            response->response = "RETRIEVE_ERROR";

            delete tempMode;
            return;
        }
        modesToSend.push_back(fromCtrlModeToString[*tempMode]);
    }
    response->modes = modesToSend;
    response->response = "OK";

    delete tempMode;
}


void ControlBoard_iFaceTransLayer_ros2::setControlModesCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                                    const std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Request> request,
                                                    std::shared_ptr<yarp_control_msgs::srv::SetControlModes::Response> response){
    std::lock_guard <std::mutex> lg(m_cmdMutex);

    bool noJoints = request->names.size() == 0;

    if(!request){
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Invalid request";
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
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Cannot set mode to" << request->modes[i];
            RCLCPP_ERROR(m_node->get_logger(),"Cannot set mode to %s",request->modes[i].c_str());

            return;
        }
        if(!m_iControlMode->setControlMode(noJoints ? i : m_quickJointRef[request->names[i]],fromStringToCtrlMode[request->modes[i]])){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error while setting the control mode for joint"<<request->names[i]<<"to"<<request->modes[i];
            RCLCPP_ERROR(m_node->get_logger(),"Error while setting the control mode for joint %s to %s",request->names[i].c_str(),request->modes[i].c_str());
            response->response = "SET_ERROR";

            return;
        }
    }

    response->response = "OK";
}


void ControlBoard_iFaceTransLayer_ros2::positionGet_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

    std::lock_guard <std::mutex> lg(m_cmdMutex);
#if 0  //Unnecessary check, since the hw modules do the check themselves
    int *tempMode = new int[1];
    double *tempVel = new double[1];
    for(size_t i=0; i<m_subdevice_joints; i++)
    {
        if(!m_iControlMode->getControlMode(i,tempMode)){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "There is a bit of a problem here...";
        }
        switch(*tempMode)
        {
            case VOCAB_CM_VELOCITY:
                m_iVelocityControl->getRefVelocity(i,tempVel);
                if(*tempVel!=0){
                    m_iVelocityControl->velocityMove(i,0.0);
                }
                m_iControlMode->setControlMode(i,VOCAB_CM_POSITION);
                break;
            case VOCAB_CM_POSITION:
                continue;
                break;
            default:
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Unsupported mode";
                return;
        }
    }

    delete tempMode;
    delete tempVel;
#endif

    m_iPositionControl->positionMove(&msg->data[0]);
}

void ControlBoard_iFaceTransLayer_ros2::velocityGet_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

    std::lock_guard <std::mutex> lg(m_cmdMutex);
#if 0  //Unnecessary check, since the hw modules do the check themselves
    int *tempMode = new int[1];
    bool *tempDone = new bool[1];
    for(size_t i=0; i<m_subdevice_joints; i++)
    {
        m_iControlMode->getControlMode(i,tempMode);
        switch(*tempMode)
        {
            case VOCAB_CM_POSITION:
                m_iPositionControl->checkMotionDone(i,tempDone);
                if(!*tempDone){
                    yCDebug(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Cannot change mode while moving";
                    return;
                }
                m_iControlMode->setControlMode(i,VOCAB_CM_VELOCITY);
                break;
            case VOCAB_CM_VELOCITY:
                continue;
                break;
            default:
                yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Unsupported mode";
                return;
        }
    }

    delete tempMode;
    delete tempDone;

#endif

    m_iVelocityControl->velocityMove(&msg->data[0]);
}

// For the simulator, if a subdevice parameter is given to the wrapper, it will
// open it and attach to immediately.
bool ControlBoard_iFaceTransLayer_ros2::openAndAttachSubDevice(yarp::os::Property& prop)
{
    yarp::os::Property p;
    auto* subDeviceOwned = new yarp::dev::PolyDriver;
    p.fromString(prop.toString());

    std::string subdevice = prop.find("subdevice").asString();
    p.setMonitor(prop.getMonitor(), subdevice.c_str()); // pass on any monitoring
    p.unput("device");
    p.put("device", subdevice); // subdevice was already checked before

    // if errors occurred during open, quit here.
    yCDebug(CONTROLBOARD_IFACETRANSLAYER_ROS2, "opening subdevice");
    subDeviceOwned->open(p);

    if (!subDeviceOwned->isValid()) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "opening subdevice... FAILED");
        return false;
    }

    return setDevice(subDeviceOwned, true);
}


bool ControlBoard_iFaceTransLayer_ros2::setDevice(yarp::dev::DeviceDriver* driver, bool owned)
{
    yCAssert(CONTROLBOARD_IFACETRANSLAYER_ROS2, driver);

    // Save the pointer and subDeviceOwned
    m_subdevice_ptr = driver;
    m_subdevice_owned = owned;

    m_subdevice_ptr->view(m_iPositionControl);
    if (!m_iPositionControl) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "<%s - %s>: IPositionControl interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    m_subdevice_ptr->view(m_iPositionDirect);
    if (!m_iPositionDirect) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "<%s - %s>: IPositionDirect interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    m_subdevice_ptr->view(m_iVelocityControl);
    if (!m_iVelocityControl) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "<%s - %s>: IVelocityControl interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    m_subdevice_ptr->view(m_iControlMode);
    if (!m_iControlMode) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "<%s - %s>: IControlMode interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    m_subdevice_ptr->view(m_iAxisInfo);
    if (!m_iAxisInfo) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "<%s - %s>: IAxisInfo interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    // Get the number of controlled joints
    int tmp_axes;
    if (!m_iPositionControl->getAxes(&tmp_axes)) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "<%s - %s>: Failed to get axes number for subdevice ",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }
    if (tmp_axes <= 0) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2, "<%s - %s>: attached device has an invalid number of joints (%d)",  m_nodeName.c_str(), m_posTopicName.c_str(), tmp_axes);
        return false;
    }
    m_subdevice_joints = static_cast<size_t>(tmp_axes);

    std::string tmpName;
    for(size_t i=0; i<m_subdevice_joints; i++){
        if(!m_iAxisInfo->getAxisName(i,tmpName)){
            yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error retrieving axis" << i << "name. For this device to work, every joint needs a name";

            return false;
        }
        m_quickJointRef[tmpName] = i;
    }

    return true;
}


void ControlBoard_iFaceTransLayer_ros2::closeDevice()
{
    // If the subdevice is owned, close and delete the device
    if (m_subdevice_owned) {
        yCAssert(CONTROLBOARD_IFACETRANSLAYER_ROS2, m_subdevice_ptr);
        m_subdevice_ptr->close();
        delete m_subdevice_ptr;
    }
    m_subdevice_ptr = nullptr;
    m_subdevice_owned = false;
    m_subdevice_joints = 0;
    m_subdevice_ready = false;


    // Clear all interfaces
    m_iPositionControl = nullptr;
    m_iVelocityControl = nullptr;
    m_iAxisInfo = nullptr;
}

bool ControlBoard_iFaceTransLayer_ros2::attach(yarp::dev::PolyDriver* poly)
{
    // Check if we already instantiated a subdevice previously
    if (m_subdevice_ready) {
        return false;
    }

    if (!setDevice(poly, false)) {
        return false;
    }

    if (!start()) {
        yCError(CONTROLBOARD_IFACETRANSLAYER_ROS2) << "Error starting thread";
        return false;
    }

    return true;
}

bool ControlBoard_iFaceTransLayer_ros2::detach()
{
    //check if we already instantiated a subdevice previously
    if (m_subdevice_owned) {
        return false;
    }

    // Ensure that the device is not running
    if (isRunning()) {
        stop();
    }

    closeDevice();

    return true;
}


bool ControlBoard_iFaceTransLayer_ros2::close()
{
    yCTrace(CONTROLBOARD_IFACETRANSLAYER_ROS2);
    if (Thread::isRunning())
    {
		rclcpp::shutdown();
        Thread::stop();
    }
    
    return true;
}
