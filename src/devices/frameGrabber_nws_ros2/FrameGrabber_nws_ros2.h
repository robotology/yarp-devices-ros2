/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_FRAMEGRABBER_NWS_ROS2_H
#define YARP_FRAMEGRABBER_NWS_ROS2_H

#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Stamp.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameGrabberControls.h>
#include <yarp/dev/IFrameGrabberControlsDC1394.h>
#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/dev/IRgbVisualParams.h>
#include <yarp/dev/WrapperSingle.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>


class FrameGrabber_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        public yarp::os::PeriodicThread
{
private:
    // Publishers
    typedef rclcpp::Publisher<sensor_msgs::msg::Image> ImageTopicType;
    typedef rclcpp::Publisher<sensor_msgs::msg::CameraInfo> CameraInfoTopicType;

    ImageTopicType::SharedPtr publisher_image;
    CameraInfoTopicType::SharedPtr publisher_cameraInfo;
    rclcpp::Node::SharedPtr m_node;
    // Subdevice
    yarp::dev::PolyDriver* subdevice {nullptr};
    bool isSubdeviceOwned {false};

    // Interfaces handled
    yarp::dev::IRgbVisualParams* iRgbVisualParams {nullptr};
    yarp::dev::IFrameGrabberImage* iFrameGrabberImage {nullptr};
    yarp::dev::IPreciselyTimed* iPreciselyTimed {nullptr};

    // Images
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* yarpimg {nullptr};

    // Internal state
    bool m_active {false};
    yarp::os::Stamp m_stamp;
    std::string m_frameId;
    std::string m_nodeName;

    // Options
    static constexpr double s_default_period = 0.03; // seconds
    double m_period {s_default_period};

    bool setCamInfo(sensor_msgs::msg::CameraInfo& cameraInfo);

public:
    FrameGrabber_nws_ros2();
    FrameGrabber_nws_ros2(const FrameGrabber_nws_ros2&) = delete;
    FrameGrabber_nws_ros2(FrameGrabber_nws_ros2&&) = delete;
    FrameGrabber_nws_ros2& operator=(const FrameGrabber_nws_ros2&) = delete;
    FrameGrabber_nws_ros2& operator=(FrameGrabber_nws_ros2&&) = delete;
    ~FrameGrabber_nws_ros2() override;

    // DeviceDriver
    bool close() override;
    bool open(yarp::os::Searchable& config) override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    //RateThread
    bool threadInit() override;
    void threadRelease() override;
    void run() override;
};

#endif // YARP_FRAMEGRABBER_NWS_ROS2_H
