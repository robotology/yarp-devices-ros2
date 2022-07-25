/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
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
                                                                       {"MIXED",VOCAB_CM_MIXED}
                                                                      };
const std::vector<std::string> implementedCtrlModes{"POSITION",
                                              "POSITION_DIRECT",
                                              "VELOCITY"};

/** convert degrees to radiants for ROS messages */
inline double convertDegreesToRadians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

inline double convertRadiansToDegrees(double degrees)
{
    return degrees * M_PI / 180.0;
}
} // namespace

// UTILITIES ------------------------------------------------------------------------------------------------------------- END //

ControlBoard_nws_ros2::ControlBoard_nws_ros2() :
        yarp::os::PeriodicThread(default_period)
{
}

void ControlBoard_nws_ros2::closePorts()
{
    // FIXME
    yCWarning(CONTROLBOARD_ROS2, "FIXME: closePorts() is not implemented yet!");
    YARP_UNUSED(this);
}

bool ControlBoard_nws_ros2::close()
{
    if(m_spinner){
        if(m_spinner->isRunning()){
            m_spinner->stop();
        }
    }
    // Ensure that the device is not running
    if (isRunning()) {
        stop();
    }

    closeDevice();
    closePorts();

    return true;
}


bool ControlBoard_nws_ros2::open(Searchable& config)
{
    Property prop;
    prop.fromString(config.toString());

    // Check parameter, so if both are present we use the correct one
    if (prop.check("period")) {
        if (!prop.find("period").isFloat64()) {
            yCError(CONTROLBOARD_ROS2) << "'period' parameter is not a double value";
            return false;
        }
        period = prop.find("period").asFloat64();
        if (period <= 0) {
            yCError(CONTROLBOARD_ROS2) << "'period' parameter is not valid, read value is" << period;
            return false;
        }
    } else {
        yCDebug(CONTROLBOARD_ROS2) << "'period' parameter missing, using default thread period = 0.02s";
        period = default_period;
    }

    // Check if we need to create subdevice or if they are
    // passed later on thorugh attach()
    if (prop.check("subdevice")) {
        prop.setMonitor(config.getMonitor());
        if (!openAndAttachSubDevice(prop)) {
            yCError(CONTROLBOARD_ROS2, "Error while opening subdevice");
            return false;
        }
        subdevice_ready = true;
    }

    // check for node_name parameter
    if (!config.check("node_name")) {
        yCError(CONTROLBOARD_ROS2) << " cannot find node_name parameter";
        return false;
    }
    m_nodeName = config.find("node_name").asString();
    if(m_nodeName[0] == '/'){
        yCError(CONTROLBOARD_ROS2) << "node_name cannot have an initial /";
        return false;
    }
    // check for topic_name parameter
    if (!config.check("topic_name")) {
        yCError(CONTROLBOARD_ROS2) << " cannot find topic_name parameter";
        return false;
    }
    topicName = config.find("topic_name").asString();
    if(topicName[0] != '/'){
        yCError(CONTROLBOARD_ROS2) << "topic_name must begin with an initial /";
        return false;
    }
    yCInfo(CONTROLBOARD_ROS2) << "topic_name is " << topicName;

    m_node = NodeCreator::createNode(m_nodeName);
    m_publisher = m_node->create_publisher<sensor_msgs::msg::JointState>(topicName, 10);

    yCError(CONTROLBOARD_ROS2) << "-------------------------------------------------------- Artropodi 1";
    if (config.check("msgs_name")) {
        yCError(CONTROLBOARD_ROS2) << "-------------------------------------------------------- Artropodi 2";
        std::string msgs_name = config.find("msgs_name").asString();
        yCError(CONTROLBOARD_ROS2) << "-------------------------------------------------------- Artropodi 3";
        if (msgs_name[0] != '/') {
            yCError(CONTROLBOARD_ROS2) << "-------------------------------------------------------- Artropodi 4";
            msgs_name = "/"+msgs_name;
        }
        yCError(CONTROLBOARD_ROS2) << "-------------------------------------------------------- Artropodi 5";
        if(!initRos2Control(msgs_name)){
            yCError(CONTROLBOARD_ROS2) << "Error initializing the ROS2 control related topics and services";
            RCLCPP_ERROR(m_node->get_logger(),"Error initializing the ROS2 control related topics and services");
            return false;
        }
        yCError(CONTROLBOARD_ROS2) << "-------------------------------------------------------- Artropodi 6";
    }

    // In case attach is not deferred and the controlboard already owns a valid device
    // we can start the thread. Otherwise this will happen when attach is called
    if (subdevice_ready) {
        setPeriod(period);
        if (!start()) {
            yCError(CONTROLBOARD_ROS2) << "Error starting thread";
            return false;
        }
        if(m_spinner){
            if(!m_spinner->start()){
                yCError(CONTROLBOARD_ROS2) << "Error starting the spinner";
            }
        }
    }

    return true;
}


bool ControlBoard_nws_ros2::initRos2Control(const std::string& name){
    m_posTopicName = name+"/position";
    m_posDirTopicName = name+"/position_direct";
    m_velTopicName = name+"/velocity";
    m_getModesSrvName = name+"/get_modes";
    m_setModesSrvName = name+"/set_modes";
    m_getAvailableModesSrvName = name+"/get_available_modes";
    m_getJointsNamesSrvName = name+"/get_joints_names";

    // Creating topics ------------------------------------------------------------------------------------------------- //

    m_posSubscription = m_node->create_subscription<yarp_control_msgs::msg::Position>(m_posTopicName, 10,
                                                                                      std::bind(&ControlBoard_nws_ros2::positionTopic_callback,
                                                                                      this, std::placeholders::_1));
    if(!m_posSubscription){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the Position msg subscription";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Position msg subscription");

        return false;
    }
    m_posDirectSubscription = m_node->create_subscription<yarp_control_msgs::msg::PositionDirect>(m_posDirTopicName, 10,
                                                                                                  std::bind(&ControlBoard_nws_ros2::positionDirectTopic_callback,
                                                                                                            this, std::placeholders::_1));
    if(!m_posDirectSubscription){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the Position direct msg subscription";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Position direct msg subscription");

        return false;
    }
    m_velSubscription = m_node->create_subscription<yarp_control_msgs::msg::Velocity>(m_velTopicName, 10,
                                                                                      std::bind(&ControlBoard_nws_ros2::velocityTopic_callback,
                                                                                      this, std::placeholders::_1));
    if(!m_velSubscription){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the Velocity msg subscription";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the Velocity msg subscription");

        return false;
    }

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
                                                                                           std::bind(&ControlBoard_nws_ros2::getControlModesCallback,
                                                                                                     this,std::placeholders::_1,std::placeholders::_2,
                                                                                                     std::placeholders::_3));
    if(!m_getControlModesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetControlModes service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetControlModes service");

        return false;
    }
    m_setControlModesSrv = m_node->create_service<yarp_control_msgs::srv::SetControlModes>(m_setModesSrvName,
                                                                                           std::bind(&ControlBoard_nws_ros2::setControlModesCallback,
                                                                                                     this,std::placeholders::_1,std::placeholders::_2,
                                                                                                     std::placeholders::_3));
    if(!m_setControlModesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the SetControlModes service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the SetControlModes service");

        return false;
    }
    m_getAvailableModesSrv = m_node->create_service<yarp_control_msgs::srv::GetAvailableControlModes>(m_getAvailableModesSrvName,
                                                                                                      std::bind(&ControlBoard_nws_ros2::getAvailableModesCallback,
                                                                                                                this,std::placeholders::_1,std::placeholders::_2,
                                                                                                                std::placeholders::_3));
    if(!m_getAvailableModesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetAvailableModes service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetAvailableModes service");

        return false;
    }
    m_getJointsNamesSrv = m_node->create_service<yarp_control_msgs::srv::GetJointsNames>(m_getJointsNamesSrvName,
                                                                                         std::bind(&ControlBoard_nws_ros2::getJointsNamesCallback,
                                                                                                   this,std::placeholders::_1,std::placeholders::_2,
                                                                                                   std::placeholders::_3));
    if(!m_getJointsNamesSrv){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetJointsNames service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetJointsNames service");

        return false;
    }

    m_spinner = new Ros2Spinner(m_node);
    if (!m_spinner){
        yCError(CONTROLBOARD_ROS2) << "Could not initialize the GetJointsNames service";
        RCLCPP_ERROR(m_node->get_logger(),"Could not initialize the GetJointsNames service");

        return false;
    }

    std::string tmpName;
    for(size_t i=0; i<subdevice_joints; i++){
        if(!iAxisInfo->getAxisName(i,tmpName)){
            yCError(CONTROLBOARD_ROS2) << "Error retrieving axis" << i << "name. For this device to work, every joint needs a name";

            return false;
        }
        m_quickJointRef[tmpName] = i;
    }

    return true;
}


// For the simulator, if a subdevice parameter is given to the wrapper, it will
// open it and attach to immediately.
bool ControlBoard_nws_ros2::openAndAttachSubDevice(Property& prop)
{
    Property p;
    auto* subDeviceOwned = new PolyDriver;
    p.fromString(prop.toString());

    std::string subdevice = prop.find("subdevice").asString();
    p.setMonitor(prop.getMonitor(), subdevice.c_str()); // pass on any monitoring
    p.unput("device");
    p.put("device", subdevice); // subdevice was already checked before

    // if errors occurred during open, quit here.
    yCDebug(CONTROLBOARD_ROS2, "opening subdevice");
    subDeviceOwned->open(p);

    if (!subDeviceOwned->isValid()) {
        yCError(CONTROLBOARD_ROS2, "opening subdevice... FAILED");
        return false;
    }

    return setDevice(subDeviceOwned, true);
}


bool ControlBoard_nws_ros2::setDevice(yarp::dev::DeviceDriver* driver, bool owned)
{
    yCAssert(CONTROLBOARD_ROS2, driver);

    // Save the pointer and subDeviceOwned
    subdevice_ptr = driver;
    subdevice_owned = owned;

    subdevice_ptr->view(iPositionControl);
    if (!iPositionControl) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IPositionControl interface was not found in subdevice. Quitting",  m_nodeName.c_str(), topicName.c_str());
        return false;
    }

    subdevice_ptr->view(m_iPositionDirect);
    if (!m_iPositionDirect) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IPositionDirect interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    subdevice_ptr->view(m_iVelocityControl);
    if (!m_iVelocityControl) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IVelocityControl interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    subdevice_ptr->view(m_iControlMode);
    if (!m_iControlMode) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IControlMode interface was not found in subdevice. Quitting",  m_nodeName.c_str(), m_posTopicName.c_str());
        return false;
    }

    subdevice_ptr->view(iEncodersTimed);
    if (!iEncodersTimed) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IEncodersTimed interface was not found in subdevice. Quitting",  m_nodeName.c_str(), topicName.c_str());
        return false;
    }

    subdevice_ptr->view(iTorqueControl);
    if (!iTorqueControl) {
        yCWarning(CONTROLBOARD_ROS2, "<%s - %s>: ITorqueControl interface was not found in subdevice.",  m_nodeName.c_str(), topicName.c_str());
    }

    subdevice_ptr->view(iAxisInfo);
    if (!iAxisInfo) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: IAxisInfo interface was not found in subdevice. Quitting",  m_nodeName.c_str(), topicName.c_str());
        return false;
    }

    // Get the number of controlled joints
    int tmp_axes;
    if (!iPositionControl->getAxes(&tmp_axes)) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: Failed to get axes number for subdevice ",  m_nodeName.c_str(), topicName.c_str());
        return false;
    }
    if (tmp_axes <= 0) {
        yCError(CONTROLBOARD_ROS2, "<%s - %s>: attached device has an invalid number of joints (%d)",  m_nodeName.c_str(), topicName.c_str(), tmp_axes);
        return false;
    }
    subdevice_joints = static_cast<size_t>(tmp_axes);
    times.resize(subdevice_joints);
    ros_struct.name.resize(subdevice_joints);
    ros_struct.position.resize(subdevice_joints);
    ros_struct.velocity.resize(subdevice_joints);
    ros_struct.effort.resize(subdevice_joints);

    if (!updateAxisName()) {
        return false;
    }

    return true;
}


void ControlBoard_nws_ros2::closeDevice()
{
    // If the subdevice is owned, close and delete the device
    if (subdevice_owned) {
        yCAssert(CONTROLBOARD_ROS2, subdevice_ptr);
        subdevice_ptr->close();
        delete subdevice_ptr;
    }
    subdevice_ptr = nullptr;
    subdevice_owned = false;
    subdevice_joints = 0;
    subdevice_ready = false;

    times.clear();

    // Clear all interfaces
    iPositionControl = nullptr;
    iEncodersTimed = nullptr;
    iTorqueControl = nullptr;
    iAxisInfo = nullptr;
}

bool ControlBoard_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    // Check if we already instantiated a subdevice previously
    if (subdevice_ready) {
        return false;
    }

    if (!setDevice(poly, false)) {
        return false;
    }

    setPeriod(period);
    if (!start()) {
        yCError(CONTROLBOARD_ROS2) << "Error starting thread";
        return false;
    }

    if(m_spinner){
        if(!m_spinner->start()){
            yCError(CONTROLBOARD_ROS2) << "Error starting the spinner";
        }
    }

    return true;
}

bool ControlBoard_nws_ros2::detach()
{
    //check if we already instantiated a subdevice previously
    if (subdevice_owned) {
        return false;
    }

    // Ensure that the device is not running
    if (isRunning()) {
        stop();
    }

    closeDevice();

    return true;
}

bool ControlBoard_nws_ros2::updateAxisName()
{
    // IMPORTANT!! This function has to be called BEFORE the thread starts,
    // the name has to be correct right from the first message!!

    yCAssert(CONTROLBOARD_ROS2, iAxisInfo);

    std::vector<std::string> tmpVect;
    for (size_t i = 0; i < subdevice_joints; i++) {
        std::string tmp;
        bool ret = iAxisInfo->getAxisName(i, tmp);
        if (!ret) {
            yCError(CONTROLBOARD_ROS2, "Joint name for axis %zu not found!", i);
            return false;
        }
        tmpVect.emplace_back(tmp);
    }

    yCAssert(CONTROLBOARD_ROS2, tmpVect.size() == subdevice_joints);

    jointNames = tmpVect;

    return true;
}

void ControlBoard_nws_ros2::run()
{
    yCAssert(CONTROLBOARD_ROS2, iEncodersTimed);
    yCAssert(CONTROLBOARD_ROS2, iAxisInfo);

    bool positionsOk = iEncodersTimed->getEncodersTimed(ros_struct.position.data(), times.data());
    YARP_UNUSED(positionsOk);

    bool speedsOk = iEncodersTimed->getEncoderSpeeds(ros_struct.velocity.data());
    YARP_UNUSED(speedsOk);

    if (iTorqueControl) {
        bool torqueOk = iTorqueControl->getTorques(ros_struct.effort.data());
        YARP_UNUSED(torqueOk);
    }

    // Update the port envelope time by averaging all timestamps
    time.update(std::accumulate(times.begin(), times.end(), 0.0) / subdevice_joints);
    yarp::os::Stamp averageTime = time;

    // Data from HW have been gathered few lines before
    JointTypeEnum jType;
    for (size_t i = 0; i < subdevice_joints; i++) {
        iAxisInfo->getJointType(i, jType);
        if (jType == VOCAB_JOINTTYPE_REVOLUTE) {
            ros_struct.position[i] = convertDegreesToRadians(ros_struct.position[i]);
            ros_struct.velocity[i] = convertDegreesToRadians(ros_struct.velocity[i]);
        }
    }

    ros_struct.name = jointNames;
    ros_struct.header.stamp.sec = int(averageTime.getTime()); // FIXME
    ros_struct.header.stamp.nanosec = static_cast<int>(1000000000 * (averageTime.getTime() - int(averageTime.getTime()))); // FIXME

//    ros_struct.header.stamp = m_node->get_clock()->now();    //@@@@@@@@@@@ FIXME: averageTime.getTime();
//     ros_struct.header.frame_id = m_frame_id; // FIXME


    // FIXME
    ++counter;
//     ros_struct.header.seq = counter++;

    m_publisher->publish(ros_struct);
}

bool ControlBoard_nws_ros2::messageVectorsCheck(const std::string &valueName, const std::vector<std::string> &names, const std::vector<double> &ref_values, const std::vector<double> &derivative){
    if(!namesCheck(names)) {
        return false;
    }
    
    bool allJoints = names.size() == 0 || names.size() == subdevice_joints;

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
    bool allJointsFail = allJoints && (ref_values.size() != subdevice_joints || (!noRef && derivative.size() != subdevice_joints));

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

    bool allJoints = names.size() == 0 || names.size() == subdevice_joints;
    bool diffValNames = !allJoints && (names.size() != ref_values.size());
    bool emptyValues = ref_values.size() == 0;

    if(emptyValues){
        yCError(CONTROLBOARD_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector cannot be empty",valueName.c_str());

        return false;
    }

    bool allJointsFail = allJoints && ref_values.size() != subdevice_joints;

    return !diffValNames && !allJointsFail;
}


bool ControlBoard_nws_ros2::namesCheck(const std::vector<std::string> &names){
    if(names.size() > subdevice_joints){
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

    bool allJoints = names.size() == 0 || names.size() == subdevice_joints;
    bool diffValNames = !allJoints && (names.size() != ref_values.size());
    bool emptyValues = ref_values.size() == 0;

    if(emptyValues){
        yCError(CONTROLBOARD_ROS2) << "The" << valueName << "vector cannot be empty";
        RCLCPP_ERROR(m_node->get_logger(),"The %s vector cannot be empty",valueName.c_str());

        return false;
    }

    bool allJointsFail = allJoints && ref_values.size() != subdevice_joints;

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

    bool *done = new bool[1];
    if(noJoints){
        if (!iPositionControl->checkMotionDone(done)){
            yCError(CONTROLBOARD_ROS2) << "Communication error on checking motion done";
            RCLCPP_ERROR(m_node->get_logger(),"Communication error on checking motion done");
            
            delete done;
            return;
        }
        if(!done[0]){
            yCError(CONTROLBOARD_ROS2) << "Cannot start a new movement while another one is still being preformed";
            RCLCPP_ERROR(m_node->get_logger(),"Cannot start a new movement while another one is still being preformed");

            delete done;
            return;
        }
        if (!noSpeed){
            if(!iPositionControl->setRefSpeeds(&msg->ref_velocities[0])){
                yCError(CONTROLBOARD_ROS2) << "Error in setting the reference velocities";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference velocities");
                
                delete done;
                return;
            }
        }
        if(!iPositionControl->positionMove(&msg->positions[0])){
            yCError(CONTROLBOARD_ROS2) << "Error in setting the positions";
            RCLCPP_ERROR(m_node->get_logger(),"Error in setting the positions");

            delete done;
            return;
        }
    }
    else{
        for(size_t i=0; i<msg->positions.size(); i++){
            size_t index = m_quickJointRef[msg->names[i]];
            if (!iPositionControl->checkMotionDone(index,done)){
                yCError(CONTROLBOARD_ROS2) << "Communication error on checking motion done";
                RCLCPP_ERROR(m_node->get_logger(),"Communication error on checking motion done");

                delete done;
                return;
            }
            if(!done[0]){
                yCError(CONTROLBOARD_ROS2) << "Cannot start a new movement while another one is still being preformed";
                RCLCPP_ERROR(m_node->get_logger(),"Cannot start a new movement while another one is still being preformed");

                delete done;
                return;
            }
            if(!noSpeed){
                if(!iPositionControl->setRefSpeed(index,msg->ref_velocities[i])){
                    yCError(CONTROLBOARD_ROS2) << "Error in setting the reference velocity";
                    RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference velocity");

                    delete done;
                    return;
                }
            }
            if(!iPositionControl->positionMove(index,msg->positions[i])){
                yCError(CONTROLBOARD_ROS2) << "Error in setting the position";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the position");

                delete done;
                return;
            }
        }
    }

    delete done;
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

    if(noJoints){
        if(!m_iPositionDirect->setPositions(&msg->positions[0])){
            yCError(CONTROLBOARD_ROS2) << "Error in setting the positions";
            RCLCPP_ERROR(m_node->get_logger(),"Error in setting the positions");

            return;
        }
    }
    else{
        for(size_t i=0; i<msg->positions.size(); i++){
            size_t index = m_quickJointRef[msg->names[i]];
            if(!m_iPositionDirect->setPosition(index,msg->positions[i])){
                yCError(CONTROLBOARD_ROS2) << "Error in setting the position";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the position");

                return;
            }
        }
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

    if(noJoints){
        if (!noAccel){
            if(!m_iVelocityControl->setRefAccelerations(&msg->ref_accelerations[0])){
                yCError(CONTROLBOARD_ROS2) << "Error in setting the reference accelerations";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference accelerations");

                return;
            }
        }
        if(!m_iVelocityControl->velocityMove(&msg->velocities[0])){
            yCError(CONTROLBOARD_ROS2) << "Error in setting the velocities";
            RCLCPP_ERROR(m_node->get_logger(),"Error in setting the velocities");

            return;
        }
    }
    else{
        for(size_t i=0; i<msg->velocities.size(); i++){
            size_t index = m_quickJointRef[msg->names[i]];
            if(!noAccel){
                if(!m_iVelocityControl->setRefAcceleration(index,msg->ref_accelerations[i])){
                    yCError(CONTROLBOARD_ROS2) << "Error in setting the reference acceleration";
                    RCLCPP_ERROR(m_node->get_logger(),"Error in setting the reference acceleration");

                    return;
                }
            }
            if(!m_iVelocityControl->velocityMove(index,msg->velocities[i])){
                yCError(CONTROLBOARD_ROS2) << "Error in setting the velocitie";
                RCLCPP_ERROR(m_node->get_logger(),"Error in setting the velocitie");

                return;
            }
        }
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

    if(!noIndexes && request->joint_indexes.size() > subdevice_joints){
        yCError(CONTROLBOARD_ROS2) << "request->joint_indexes vector cannot be longer than the actual number of joints:"<< request->joint_indexes.size() << "instead of" << subdevice_joints;
        RCLCPP_ERROR(m_node->get_logger(),"request->joint_indexes vector cannot be longer than the actual number of joints: %ld instead of %ld", request->joint_indexes.size(), subdevice_joints);

        response->response = "SIZE_ERROR";

        return;
    }

    std::string tempName;
    if(noIndexes){
        for(size_t i=0; i<subdevice_joints; i++){
            if(!iAxisInfo->getAxisName(i,tempName)){
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
            if(!iAxisInfo->getAxisName(i,tempName)){
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

    size_t forLimit = noJoints ? subdevice_joints : request->names.size();
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

    size_t forLimit = noJoints ? subdevice_joints : request->names.size();

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
