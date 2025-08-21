/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_RGBDSENSOR_NWC_ROS2
#define YARP_RGBDSENSOR_NWC_ROS2

// c++ libraries
#include <iostream>
#include <cstring>
#include <mutex>

// yarp libraries
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/os/Property.h>
#include <Ros2Spinner.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/impl/FixedSizeBuffersManager.h>
#include <yarp/sig/IntrinsicParams.h>


// ros libraries
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <Ros2Subscriber.h>

#include "RgbdSensor_nwc_ros2_ParamsParser.h"

/**
 *  @ingroup dev_impl_nwc_ros2
 *
 * \section rgbdSensor_nwc_ros2_device_parameters Description of input parameters
 * This device is an nwc take a stream of data from ros2 and exposes to the user IRGBDSensor interface.
 * See its documentation for more details about the interface.
 *
 *   Parameters required by this device are:shown in class: RgbdSensor_nwc_ros2_ParamsParser
 *
 * example of configuration file:
 *
 * Example of configuration parameters:
 *
 * \code{.unparsed}
 * device rgbdSensor_nwc_ros2
 * rgb_data_topic /<robotName>/camera/color/image_raw
 * rgb_info_topic /<robotName>/camera/color/camera_info
 * depth_data_topic /<robotName>/depth/color/image_rect_raw
 * depth_info_topic /<robotName>/depth/color/camera_info
 * \endcode
 */
/**
 * This class is an utility
 */



typedef yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
typedef yarp::sig::FlexImage flexImage;



/**
 * This class implements an nwc for ros2 for an rgbd sensor.
 */
class RgbdSensor_nwc_ros2:
        public yarp::dev::DeviceDriver,
        public yarp::dev::IRGBDSensor,
        RgbdSensor_nwc_ros2_ParamsParser
        {
        private:
            // mutex for writing or retrieving images
            std::mutex m_rgb_camera_info_mutex;
            std::mutex m_rgb_image_mutex;
            std::mutex m_depth_camera_info_mutex;
            std::mutex m_depth_image_mutex;

            // current depth and rgb images
            yarp::os::Stamp            m_current_depth_stamp;
            depthImage                 m_current_depth_image;
            std::string                m_depth_image_frame;
            yarp::sig::IntrinsicParams m_depth_params;
            double                     m_max_depth_width;
            double                     m_max_depth_height;

            yarp::os::Stamp            m_current_rgb_stamp;
            flexImage                  m_current_rgb_image;
            std::string                m_rgb_image_frame;
            yarp::sig::IntrinsicParams m_rgb_params;
            double                     m_max_rgb_width;
            double                     m_max_rgb_height;

            bool m_depth_image_valid = false;
            bool m_depth_stamp_valid = false;
            bool m_rgb_image_valid = false;
            bool m_rgb_stamp_valid = false;


            // ros2 variables for topics and subscriptions
            std::string m_topic_rgb_camera_info;
            std::string m_topic_depth_camera_info;

            // yarp variables
            int      m_verbose{2};

            // Spinner
            Ros2Spinner*            m_spinner{nullptr};

            //ros2 node and subscribers
            Ros2Subscriber<RgbdSensor_nwc_ros2, sensor_msgs::msg::CameraInfo>* m_sub1;
            Ros2Subscriber<RgbdSensor_nwc_ros2, sensor_msgs::msg::Image>* m_sub2;
            rclcpp::Node::SharedPtr m_node;
            //private functions
            void saveIntrinsics(sensor_msgs::msg::CameraInfo::SharedPtr msg, yarp::sig::IntrinsicParams& params);

        public:
            RgbdSensor_nwc_ros2();
            RgbdSensor_nwc_ros2(const RgbdSensor_nwc_ros2&) = delete;
            RgbdSensor_nwc_ros2(RgbdSensor_nwc_ros2&&) noexcept = delete;
            RgbdSensor_nwc_ros2& operator=(const RgbdSensor_nwc_ros2&) = delete;
            RgbdSensor_nwc_ros2& operator=(RgbdSensor_nwc_ros2&&) noexcept = delete;
            ~RgbdSensor_nwc_ros2() override = default;

            // DeviceDriver
            bool open(yarp::os::Searchable& config) override;
            bool close() override;

            // IRGBDSensor
            int    getRgbHeight() override;
            int    getRgbWidth() override;
            yarp::dev::ReturnValue   getRgbSupportedConfigurations(std::vector<yarp::dev::CameraConfig> &configurations) override;
            yarp::dev::ReturnValue   getRgbResolution(int &width, int &height) override;
            yarp::dev::ReturnValue   setRgbResolution(int width, int height) override;
            yarp::dev::ReturnValue   getRgbFOV(double& horizontalFov, double& verticalFov) override;
            yarp::dev::ReturnValue   setRgbFOV(double horizontalFov, double verticalFov) override;
            yarp::dev::ReturnValue   getRgbMirroring(bool& mirror) override;
            yarp::dev::ReturnValue   setRgbMirroring(bool mirror) override;

            yarp::dev::ReturnValue   getRgbIntrinsicParam(yarp::os::Property& intrinsic) override;
            int    getDepthHeight() override;
            int    getDepthWidth() override;
            yarp::dev::ReturnValue   getDepthResolution(int& width, int& height) override;
            yarp::dev::ReturnValue   setDepthResolution(int width, int height) override;
            yarp::dev::ReturnValue   getDepthFOV(double& horizontalFov, double& verticalFov) override;
            yarp::dev::ReturnValue   setDepthFOV(double horizontalFov, double verticalFov) override;
            yarp::dev::ReturnValue   getDepthIntrinsicParam(yarp::os::Property& intrinsic) override;
            yarp::dev::ReturnValue   getDepthAccuracy(double& accuracy) override;
            yarp::dev::ReturnValue   setDepthAccuracy(double accuracy) override;
            yarp::dev::ReturnValue   getDepthClipPlanes(double& nearPlane, double& farPlane) override;
            yarp::dev::ReturnValue   setDepthClipPlanes(double nearPlane, double farPlane) override;
            yarp::dev::ReturnValue   getDepthMirroring(bool& mirror) override;
            yarp::dev::ReturnValue   setDepthMirroring(bool mirror) override;

            yarp::dev::ReturnValue   getExtrinsicParam(yarp::sig::Matrix &extrinsic) override;

            yarp::dev::ReturnValue   getRgbImage(yarp::sig::FlexImage& rgb_image, yarp::os::Stamp* rgb_image_stamp = NULL) override;
            yarp::dev::ReturnValue   getDepthImage(depthImage& depth_image, yarp::os::Stamp* depth_image_stamp = nullptr) override;
            yarp::dev::ReturnValue   getImages(yarp::sig::FlexImage& rgb_image, depthImage& depth_image, yarp::os::Stamp* rgb_image_stamp=nullptr, yarp::os::Stamp* depth_image_stamp=nullptr) override;

            void callback(sensor_msgs::msg::CameraInfo::SharedPtr msg, std::string topic);
            void callback(sensor_msgs::msg::Image::SharedPtr msg, std::string topic);

            void depth_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);
            void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
            void color_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);
            void color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

            yarp::dev::ReturnValue   getSensorStatus(yarp::dev::IRGBDSensor::RGBDSensor_status& status) override;
            yarp::dev::ReturnValue   getLastErrorMsg(std::string& message, yarp::os::Stamp* timeStamp = NULL) override;
        };





#endif // YARP_RGBDSENSOR_NWC_ROS2
