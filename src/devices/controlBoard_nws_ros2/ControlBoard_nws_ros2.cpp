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

using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

namespace {
YARP_LOG_COMPONENT(CONTROLBOARD_ROS2, "yarp.ros2.controlBoard_nws_ros2", yarp::os::Log::TraceType);

/** convert degrees to radiants for ROS messages */
inline double convertDegreesToRadians(double degrees)
{
    return degrees / 180.0 * M_PI;
}
} // namespace


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

    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    m_node = std::make_shared<rclcpp::Node>(m_nodeName);
    m_publisher = m_node->create_publisher<sensor_msgs::msg::JointState>(topicName, 10);

    // In case attach is not deferred and the controlboard already owns a valid device
    // we can start the thread. Otherwise this will happen when attach is called
    if (subdevice_ready) {
        setPeriod(period);
        if (!start()) {
            yCError(CONTROLBOARD_ROS2) << "Error starting thread";
            return false;
        }
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

    ros_struct.header.stamp = m_node->get_clock()->now();    //@@@@@@@@@@@ FIXME: averageTime.getTime();
//     ros_struct.header.frame_id = m_frame_id; // FIXME


    // FIXME
    ++counter;
//     ros_struct.header.seq = counter++;

    m_publisher->publish(ros_struct);
}
