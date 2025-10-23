/*
 * SPDX-FileCopyrightText: 2025 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_AUDIORECORDER_NWS_ROS2_H
#define YARP_ROS2_AUDIORECORDER_NWS_ROS2_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <yarp/dev/IAudioGrabberSound.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/WrapperSingle.h>

#include "AudioRecorder_nws_ros2_ParamsParser.h"

#define DEFAULT_THREAD_PERIOD 0.02 //s

class AudioRecorder_nws_ros2 :
        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        AudioRecorder_nws_ros2_ParamsParser
{
public:
    AudioRecorder_nws_ros2();
    ~AudioRecorder_nws_ros2();

    // DeviceDriver
    bool open(yarp::os::Searchable &params) override;
    bool close() override;

    // WrapperSingle
    bool attach(yarp::dev::PolyDriver* driver) override;
    bool detach() override;

    // PeriodicThread
    bool threadInit() override;
    void threadRelease() override;
    void run() override;


private:
    // stamp count for timestamp
    yarp::os::Stamp m_timeStamp;

    //ros2 node
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr   m_ros2Publisher_status;

    //interfaces
    yarp::dev::PolyDriver m_driver;
    yarp::dev::IAudioGrabberSound* m_audioRecorder_interface{nullptr};
};

#endif // YARP_ROS2_AUDIORECORDER_NWS_ROS2_H
