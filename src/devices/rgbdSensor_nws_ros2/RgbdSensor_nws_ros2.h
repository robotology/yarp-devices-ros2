/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_RANGEFINDER2D_NWS_ROS2_H
#define YARP_DEV_RANGEFINDER2D_NWS_ROS2_H

#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IFrameGrabberControls.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "RgbdSensor_nws_ros2_ParamsParser.h"

#include <mutex>

/**
 *  @ingroup dev_impl_nws_ros2 dev_impl_media
 *
 * \brief `RgbdSensor_nws_ros2`: A Network grabber for kinect-like devices.
 *
 *  Documentation to be added
 *
*/
struct CameraInfoData {
    sensor_msgs::msg::CameraInfo colorCamInfo;
    sensor_msgs::msg::CameraInfo depthCamInfo;
};

class RgbdSensor_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        public yarp::os::PeriodicThread,
        RgbdSensor_nws_ros2_ParamsParser
{
public:
    RgbdSensor_nws_ros2();
    RgbdSensor_nws_ros2(const RgbdSensor_nws_ros2&) = delete;
    RgbdSensor_nws_ros2(RgbdSensor_nws_ros2&&) noexcept = delete;
    RgbdSensor_nws_ros2& operator=(const RgbdSensor_nws_ros2&) = delete;
    RgbdSensor_nws_ros2& operator=(RgbdSensor_nws_ros2&&) noexcept = delete;
    ~RgbdSensor_nws_ros2() override = default;

    // WrapperSingle
    bool        attach(yarp::dev::PolyDriver *poly) override;
    bool        detach() override;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

private:
    enum SensorType
    {
        COLOR_SENSOR,
        DEPTH_SENSOR
    };

    template <class T>
    struct param
    {
        param(T& inVar, std::string inName)
        {
            var = &inVar;
            parname = inName;
        }
        T* var;
        std::string parname;
    };

    rclcpp::Node::SharedPtr m_node;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rosPublisher_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rosPublisher_depth;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rosPublisher_colorCaminfo;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rosPublisher_depthCaminfo;

    std::string m_depth_info_topic_name;
    std::string m_color_info_topic_name;

    yarp::dev::IRGBDSensor* sensor_p {nullptr};
    yarp::dev::IFrameGrabberControls* fgCtrl {nullptr};
    bool m_forceInfoSync {true};
    bool m_firstTime {true};

    CameraInfoData m_camInfoData;

    bool writeData();
    bool setCamInfo(sensor_msgs::msg::CameraInfo& cameraInfo,
                    const std::string& frame_id,
                    const SensorType& sensorType);
    bool fromConfig(yarp::os::Searchable &config);
    bool initialize_ROS2(yarp::os::Searchable& config);

    bool read(yarp::os::ConnectionReader& connection);

    bool attach(yarp::dev::IRGBDSensor *s);

};



#endif // YARP_DEV_RANGEFINDER2D_NWS_ROS2_H
