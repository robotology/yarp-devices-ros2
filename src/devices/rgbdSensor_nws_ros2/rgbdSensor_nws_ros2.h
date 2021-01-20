/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef YARP_DEV_RANGEFINDER2D_NWS_ROS2_H
#define YARP_DEV_RANGEFINDER2D_NWS_ROS2_H

#include <yarp/dev/IWrapper.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/FrameGrabberControlImpl.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <mutex>

class Ros2Init
{
public:
    Ros2Init();

    std::shared_ptr<rclcpp::Node> node;

    static Ros2Init& get();
};




class RgbdSensor_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::dev::IWrapper,
        public yarp::dev::IMultipleWrapper,
        public yarp::os::PeriodicThread
{
public:
    RgbdSensor_nws_ros2();
    RgbdSensor_nws_ros2(const RgbdSensor_nws_ros2&) = delete;
    RgbdSensor_nws_ros2(RgbdSensor_nws_ros2&&) noexcept = delete;
    RgbdSensor_nws_ros2& operator=(const RgbdSensor_nws_ros2&) = delete;
    RgbdSensor_nws_ros2& operator=(RgbdSensor_nws_ros2&&) noexcept = delete;
    ~RgbdSensor_nws_ros2() override = default;

    // IMultipleWrapper
    bool        attachAll(const yarp::dev::PolyDriverList &p) override;
    bool        detachAll() override;

    // IWrapper
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

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rosPublisher_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rosPublisher_depth;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rosPublisher_colorCaminfo;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rosPublisher_depthCaminfo;

    std::string nodeName;
    std::string depthTopicName;
    std::string colorTopicName;
    std::string dInfoTopicName;
    std::string cInfoTopicName;
    std::string rosFrameId;

//     yarp::sig::FlexImage  colorImage;
//     DepthImage            depthImage;
//     UInt                  nodeSeq {0};

    yarp::dev::IRGBDSensor* sensor_p {nullptr};
    yarp::dev::IFrameGrabberControls* fgCtrl {nullptr};
    bool forceInfoSync {true};


    // If a subdevice parameter is given, the wrapper will open it and attach to immediately.
    // Typical usage: simulator or command line
    bool isSubdeviceOwned {false};
    yarp::dev::PolyDriver* subDeviceOwned {nullptr};
    bool openAndAttachSubDevice(yarp::os::Searchable& prop);

    bool writeData();
    bool setCamInfo(sensor_msgs::msg::CameraInfo& cameraInfo,
                    const std::string& frame_id,
                    const std::uint32_t& seq,
                    const SensorType& sensorType);

//     static std::string yarp2RosPixelCode(int code);


    bool fromConfig(yarp::os::Searchable &config);
    bool initialize_ROS2(yarp::os::Searchable& config);

    bool read(yarp::os::ConnectionReader& connection);

    bool attach(yarp::dev::IRGBDSensor *s);

};



#endif // YARP_DEV_RANGEFINDER2D_NWS_ROS2_H
