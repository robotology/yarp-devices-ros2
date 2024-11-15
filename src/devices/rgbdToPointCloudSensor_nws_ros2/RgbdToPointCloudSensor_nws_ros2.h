/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_RGBDTOPOINTCLOUDSENSOR_NWS_ROS2_H
#define YARP_DEV_RGBDTOPOINTCLOUDSENSOR_NWS_ROS2_H

#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IFrameGrabberControls.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include "RgbdToPointCloudSensor_nws_ros2_ParamsParser.h"

namespace RGBDToPointCloudRos2Impl {

const std::string frameId_param = "frame_id";
const std::string nodeName_param = "node_name";
const std::string pointCloudTopicName_param = "topic_name";

    constexpr double DEFAULT_THREAD_PERIOD = 0.03; // s
} // namespace
/**
 *  @ingroup dev_impl_nws_ros2 dev_impl_media
 *
 * \brief `rgbdToPointCloudSensor_nws_ros2`: A Network grabber for kinect-like devices. It attaches to an RGBD camera and publishes directly a pointcloud.
 *
 * \section rgbdToPointCloudSensor_nws_ros2_device_parameters Description of input parameters
 * This device will produce one stream of data for the point cloud
 * derived fron the combination of the data derived from Framegrabber and IDepthSensor interfaces.
 * See they documentation for more details about each interface.
 *
 *   Parameters required by this device are described in class: RgbdToPointClpidSensor_nws_ros2_ParamsParser
 *
 * ROS2 message type used is sensor_msgs/PointCloud2.msg ( https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)
 * Some example of configuration files:
 *
 * Example of configuration file using .ini format.
 *
 * \code{.unparsed}
 * device rgbdToPointCloudSensor_nws_ros2
 * subdevice <RGBDsensor>
 * period 33
 * topic_name /<robotName>/RGBDToPointCloud
 * frame_id /<robotName>/<framed_Id>
 * node_name /<robotName>/RGBDToPointCloudSensorNode
 * \endcode
 */

class RgbdToPointCloudSensor_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        public yarp::os::PeriodicThread,
        RgbdToPointCloudSensor_nws_ros2_ParamsParser
{

private:
    // defining types for shorter names
    typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> DepthImage;
    typedef unsigned int UInt;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_rosPublisher_pointCloud2;
    rclcpp::Node::SharedPtr m_node;

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

    UInt                  nodeSeq {0};

    yarp::dev::IRGBDSensor*             m_sensor_p {nullptr};
    yarp::dev::IFrameGrabberControls*   m_fgCtrl {nullptr};
    bool                                m_forceInfoSync {true};

    // Synch
    yarp::os::Property m_conf;

    bool writeData();

    static std::string yarp2RosPixelCode(int code);

    bool fromConfig(yarp::os::Searchable& config);
    bool initialize_ROS2(yarp::os::Searchable& config);

    bool read(yarp::os::ConnectionReader& connection);

    bool attach(yarp::dev::IRGBDSensor* s);


public:
    RgbdToPointCloudSensor_nws_ros2();
    RgbdToPointCloudSensor_nws_ros2(const RgbdToPointCloudSensor_nws_ros2&) = delete;
    RgbdToPointCloudSensor_nws_ros2(RgbdToPointCloudSensor_nws_ros2&&) noexcept = delete;
    RgbdToPointCloudSensor_nws_ros2& operator=(const RgbdToPointCloudSensor_nws_ros2&) = delete;
    RgbdToPointCloudSensor_nws_ros2& operator=(RgbdToPointCloudSensor_nws_ros2&&) noexcept = delete;
    ~RgbdToPointCloudSensor_nws_ros2() override = default;

    // WrapperSingle
    bool        attach(yarp::dev::PolyDriver *poly) override;
    bool        detach() override;

    // DeviceDriver
    bool        open(yarp::os::Searchable& config) override;
    bool        close() override;
    // PeriodicThread
    void        run() override;
};
#endif // YARP_DEV_RGBDTOPOINTCLOUDSENSOR_NWS_ROS2_H
