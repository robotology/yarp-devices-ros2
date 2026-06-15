/*
 * SPDX-FileCopyrightText: 2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "AudioRecorder_nws_ros2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <cmath>
#include <Ros2Utils.h>

YARP_LOG_COMPONENT(AUDIORECORDER_NWS_ROS2, "yarp.devices.AudioRecorder_nws_ros2")

AudioRecorder_nws_ros2::AudioRecorder_nws_ros2() : yarp::os::PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}

AudioRecorder_nws_ros2::~AudioRecorder_nws_ros2()
{
    m_audioRecorder_interface = nullptr;
}


bool AudioRecorder_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{

    if (driver->isValid())
    {
        driver->view(m_audioRecorder_interface);
    } else {
        yCError(AUDIORECORDER_NWS_ROS2) << "not valid driver";
    }

    if (m_audioRecorder_interface == nullptr)
    {
        yCError(AUDIORECORDER_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    yCInfo(AUDIORECORDER_NWS_ROS2, "Attach complete");
    PeriodicThread::setPeriod(m_period);
    return PeriodicThread::start();
}


bool AudioRecorder_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_audioRecorder_interface = nullptr;
    return true;
}

bool AudioRecorder_nws_ros2::threadInit()
{
    return true;
}

bool AudioRecorder_nws_ros2::open(yarp::os::Searchable &config)
{
    parseParams(config);
   if (m_node_name[0] == '/') {
        yCError(AUDIORECORDER_NWS_ROS2) << "node_name parameter cannot begin with '/'";
        return false;
    }
    if (m_topic_name[0] != '/') {
        yCError(AUDIORECORDER_NWS_ROS2) << "missing initial / in topic_name parameter";
        return false;
    }

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    if(m_namespace.empty()) {
        m_node = NodeCreator::createNode(m_node_name, node_options);
    } else {
        m_node = NodeCreator::createNode(m_node_name, m_namespace, node_options);
    }
    if (m_node == nullptr) {
        yCError(AUDIORECORDER_NWS_ROS2) << " opening " << m_node_name << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }
    m_ros2Publisher_status = m_node->create_publisher<std_msgs::msg::Int16MultiArray>(m_topic_name, 10);

    yCInfo(AUDIORECORDER_NWS_ROS2) << "Waiting for device to attach";
    return true;
}

void AudioRecorder_nws_ros2::threadRelease()
{
}

void AudioRecorder_nws_ros2::run()
{

    if (m_audioRecorder_interface!=nullptr && m_ros2Publisher_status)
    {
        if(m_ros2Publisher_status->get_subscription_count()>0)
        {
            double synchronized_timestamp = 0;

            yarp::sig::AudioBufferSize device_buffer_current_size;
            yarp::sig::AudioBufferSize device_buffer_max_size;
            bool isRecording=0;
            m_audioRecorder_interface->getRecordingAudioBufferCurrentSize(device_buffer_current_size);
            m_audioRecorder_interface->getRecordingAudioBufferMaxSize(device_buffer_max_size);
            m_audioRecorder_interface->isRecording(isRecording);

            if (std::isnan(synchronized_timestamp) == false)
            {
                m_timeStamp.update(synchronized_timestamp);
            }
            else
            {
                m_timeStamp.update(yarp::os::Time::now());
            }

            std_msgs::msg::Int16MultiArray statusMsg;
            statusMsg.data.resize(3);
            statusMsg.data[0] = static_cast<int>(isRecording);
            statusMsg.data[1] = static_cast<int>(device_buffer_current_size.getBufferElements());
            statusMsg.data[2] = static_cast<int>(device_buffer_max_size.getBufferElements());
            m_ros2Publisher_status->publish(statusMsg);
        }

    } else{
        yCError(AUDIORECORDER_NWS_ROS2) << "the interface is not valid";
    }
}

bool AudioRecorder_nws_ros2::close()
{
    yCTrace(AUDIORECORDER_NWS_ROS2);
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detach();
    return true;
}
