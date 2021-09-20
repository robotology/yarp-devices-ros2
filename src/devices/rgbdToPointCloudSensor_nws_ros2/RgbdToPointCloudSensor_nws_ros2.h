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

namespace RGBDToPointCloudRos2Impl {

const std::string frameId_param = "frame_id";
const std::string nodeName_param = "node_name";
const std::string pointCloudTopicName_param = "topic_name";

    constexpr double DEFAULT_THREAD_PERIOD = 0.03; // s
} // namespace
/**
 *  @ingroup dev_impl_wrapper
 *
 * \section rgbdToPointCloudSensor_nws_ros2_device_parameters Description of input parameters
 * A Network grabber for kinect-like devices.
 * This device will produce one stream of data for the point cloud
 * derived fron the combination of the data derived from Framegrabber and IDepthSensor interfaces.
 * See they documentation for more details about each interface.
 *
 *   Parameters required by this device are:
 * | Parameter name         | SubParameter            | Type    | Units          | Default Value | Required                        | Description                                                                                         | Notes |
 * |:----------------------:|:-----------------------:|:-------:|:--------------:|:-------------:|:------------------------------: |:---------------------------------------------------------------------------------------------------:|:-----:|
 * | period                 |      -                  | double  |  s             |   0.03        |  No                             | refresh period of the broadcasted values in ms                                                      | default 20ms |
 * | subdevice              |      -                  | string  |  -             |   -           |  alternative to 'attach' action | name of the subdevice to use as a data source                                                       | when used, parameters for the subdevice must be provided as well |
 * | topic_name             |      -                  | string  |  -             |   -           |  Yes                            | set the name for ROS point cloud topic                                                              | must start with a leading '/' |
 * | frame_id               |      -                  | string  |  -             |               |  Yes                            | set the name of the reference frame                                                                 |                               |
 * | node_name              |      -                  | string  |  -             |   -           |  Yes                            | set the name for ROS node                                                                           | must start with a leading '/' |
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
        public yarp::os::PeriodicThread
{

private:
    // defining types for shorter names
    typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> DepthImage;
    typedef unsigned int UInt;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rosPublisher_pointCloud2;
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



    std::string nodeName;
    std::string pointCloudTopicName;
    std::string rosFrameId;

    UInt                  nodeSeq {0};

    yarp::dev::IRGBDSensor*             sensor_p {nullptr};
    yarp::dev::IFrameGrabberControls*   fgCtrl {nullptr};
    bool                                forceInfoSync {true};

    // If a subdevice parameter is given, the wrapper will open it and attach to immediately.
    // Typical usage: simulator or command line
    bool                    isSubdeviceOwned {false};
    yarp::dev::PolyDriver*  subDeviceOwned {nullptr};

    // Synch
    yarp::os::Property m_conf;

    bool openAndAttachSubDevice(yarp::os::Searchable& prop);
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
