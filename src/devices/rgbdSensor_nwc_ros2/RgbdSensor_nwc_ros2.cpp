/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "RgbdSensor_nwc_ros2.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <Ros2Utils.h>
#include <Ros2RGBDConversionUtils.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

YARP_LOG_COMPONENT(RGBDSENSOR_NWC_ROS2, "yarp.ros2.RgbdSensor_nwc_ros2", yarp::os::Log::TraceType);



RgbdSensor_nwc_ros2::RgbdSensor_nwc_ros2()
{
}

bool RgbdSensor_nwc_ros2::open(yarp::os::Searchable& config)
{

    // node_name check
    if (!config.check("node_name")) {
        yCError(RGBDSENSOR_NWC_ROS2) << "missing node_name parameter";
        return false;
    }
    m_ros2_node_name = config.find("node_name").asString();


    // depth topic base name check
    if (!config.check("depth_topic_name")) {
        yCError(RGBDSENSOR_NWC_ROS2) << "missing depth_topic_name parameter";
        return false;
    }
    m_topic_depth_image_raw = config.find("depth_topic_name").asString();

    m_topic_depth_camera_info = m_topic_depth_image_raw.substr(0, m_topic_depth_image_raw.rfind('/')) + "/camera_info";

    // color topic base name check
    if (!config.check("color_topic_name")) {
        yCError(RGBDSENSOR_NWC_ROS2) << "missing color_topic_name parameter";
        return false;
    }
    m_topic_rgb_image_raw = config.find("color_topic_name").asString();

    m_topic_rgb_camera_info = m_topic_rgb_image_raw.substr(0, m_topic_rgb_image_raw.rfind('/')) + "/camera_info";

    m_verbose = config.check("verbose");

    m_node = NodeCreator::createNode(m_ros2_node_name);
    m_sub1= new Ros2Subscriber<RgbdSensor_nwc_ros2, sensor_msgs::msg::CameraInfo>(m_node, this);
    m_sub1->subscribe_to_topic(m_topic_rgb_camera_info);
    m_sub1->subscribe_to_topic(m_topic_depth_camera_info);
    m_sub2= new Ros2Subscriber<RgbdSensor_nwc_ros2, sensor_msgs::msg::Image>(m_node, this);
    m_sub2->subscribe_to_topic(m_topic_rgb_image_raw);
    m_sub2->subscribe_to_topic(m_topic_depth_image_raw);

    m_spinner = new Ros2Spinner(m_node);
    m_spinner->start();

    yCInfo(RGBDSENSOR_NWC_ROS2) << "opened";

    return true;
}

bool RgbdSensor_nwc_ros2::close()
{
    yCInfo(RGBDSENSOR_NWC_ROS2, "closing...");
    delete m_spinner;
    yCInfo(RGBDSENSOR_NWC_ROS2, "closed");
    return true;
}

void RgbdSensor_nwc_ros2::callback(sensor_msgs::msg::Image::SharedPtr msg, std::string topic)
{
    if (topic == m_topic_rgb_image_raw) {
        color_raw_callback(msg);
    } else if (topic == m_topic_depth_image_raw) {
        depth_raw_callback(msg);
    }
}


void RgbdSensor_nwc_ros2::callback(sensor_msgs::msg::CameraInfo::SharedPtr msg, std::string topic)
{
    if (topic == m_topic_rgb_camera_info) {
        color_info_callback(msg);
    } else if (topic == m_topic_depth_camera_info)
    {
        depth_info_callback(msg);
    }
}

void RgbdSensor_nwc_ros2::depth_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    yCTrace(RGBDSENSOR_NWC_ROS2, "callback depth image");
    std::lock_guard<std::mutex> depth_image_guard(m_depth_image_mutex);
    yarp::dev::Ros2RGBDConversionUtils::convertDepthImageRos2ToYarpImageOf(msg,m_current_depth_image);
    m_depth_image_valid = true;
}

void RgbdSensor_nwc_ros2::depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> depth_camera_info_guard(m_depth_camera_info_mutex);
    yarp::dev::Ros2RGBDConversionUtils::updateStamp(msg, m_depth_image_frame, m_current_depth_stamp);
    saveIntrinsics(msg, m_depth_params);
    m_max_depth_height = msg->height;
    m_max_depth_width = msg->width;
    m_depth_stamp_valid = true;
}

void RgbdSensor_nwc_ros2::color_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    yCTrace(RGBDSENSOR_NWC_ROS2, "callback color image");
    std::lock_guard<std::mutex> rgb_image_guard(m_rgb_image_mutex);
    yarp::dev::Ros2RGBDConversionUtils::convertRGBImageRos2ToYarpFlexImage(msg, m_current_rgb_image);
    m_rgb_image_valid = true;
}

void RgbdSensor_nwc_ros2::color_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> rgb_camera_info_guard(m_rgb_camera_info_mutex);
    yarp::dev::Ros2RGBDConversionUtils::updateStamp(msg, m_rgb_image_frame, m_current_rgb_stamp);
    saveIntrinsics(msg, m_rgb_params);
    m_max_rgb_height = msg->height;
    m_max_rgb_width = msg->width;
    m_rgb_stamp_valid = true;
}

// TODO FIXME eable distortion inverse brown conrady or check distortion in realsense
void RgbdSensor_nwc_ros2::saveIntrinsics(sensor_msgs::msg::CameraInfo::SharedPtr msg, yarp::sig::IntrinsicParams& params)
{
    params.focalLengthX = msg->k[0];
    params.focalLengthY = msg->k[4];
    params.principalPointX = msg->k[2];
    params.principalPointY = msg->k[5];
    // distortion model
    if (msg->distortion_model=="plumb_bob")
    {
        if (params.distortionModel.k1 == 0 &&
        params.distortionModel.k2 == 0 &&
        params.distortionModel.k3 == 0 &&
        params.distortionModel.t1 == 0 &&
        params.distortionModel.t2 == 0)
        {
            params.distortionModel.type = yarp::sig::YarpDistortion::YARP_DISTORTION_NONE;
        }
        params.distortionModel.type = yarp::sig::YarpDistortion::YARP_PLUMB_BOB;
        params.distortionModel.k1 = msg->d[0];
        params.distortionModel.k2 = msg->d[1];
        params.distortionModel.t1 = msg->d[2];
        params.distortionModel.t2 = msg->d[3];
        params.distortionModel.k3 = msg->d[4];
    }
    else
    {
        yCError(RGBDSENSOR_NWC_ROS2) << "Unsupported distortion model";
        params.distortionModel.type = yarp::sig::YarpDistortion::YARP_UNSUPPORTED;
    }
}



int RgbdSensor_nwc_ros2::getRgbHeight()
{
    if (!m_rgb_image_valid) return 0;
    return m_current_rgb_image.height();
}

int RgbdSensor_nwc_ros2::getRgbWidth()
{
    if (!m_rgb_image_valid) return 0;
    return m_current_rgb_image.width();
}

bool RgbdSensor_nwc_ros2::getRgbSupportedConfigurations(yarp::sig::VectorOf<yarp::dev::CameraConfig> &configurations)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getRgbSupportedConfigurations not implemented yet";
    return false;
}

bool RgbdSensor_nwc_ros2::getRgbResolution(int &width, int &height)
{
    if (!m_rgb_image_valid)
    {
        width=0;
        height=0;
        return  false;
    }
    width  = m_current_rgb_image.width();
    height = m_current_rgb_image.height();
    return true;
}

bool RgbdSensor_nwc_ros2::setDepthResolution(int width, int height)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthResolution not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::setRgbResolution(int width, int height)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setRgbResolution not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::setRgbFOV(double horizontalFov, double verticalFov)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setRgbFOV not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::setDepthFOV(double horizontalFov, double verticalFov)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthFOV not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::setDepthAccuracy(double accuracy)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthAccuracy not supported";
    return false;
}

// not sure that it is correct, maybe I should save the full image dimension
bool RgbdSensor_nwc_ros2::getRgbFOV(double &horizontalFov, double &verticalFov)
{
    if (!m_rgb_image_valid && ! m_rgb_stamp_valid)
    {
        horizontalFov=0;
        verticalFov=0;
        return  false;
        yCError(RGBDSENSOR_NWC_ROS2) << "get rgb IntrinsicParam missing information";
    }
    horizontalFov = 2 * atan(m_max_rgb_width / (2 * m_rgb_params.focalLengthX)) * 180.0 / M_PI;
    verticalFov = 2 * atan(m_max_rgb_height / (2 * m_rgb_params.focalLengthY)) * 180.0 / M_PI;
    return true;
}


bool RgbdSensor_nwc_ros2::getRgbMirroring(bool& mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "Mirroring not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::setRgbMirroring(bool mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "Mirroring not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::getRgbIntrinsicParam(yarp::os::Property& intrinsic)
{
    if (m_rgb_stamp_valid)
    {
        intrinsic.clear();
        m_rgb_params.toProperty(intrinsic);
        return true;
    }
    return false;
}

int  RgbdSensor_nwc_ros2::getDepthHeight()
{
    if (!m_depth_image_valid) return 0;
    return m_current_depth_image.height();
}

int  RgbdSensor_nwc_ros2::getDepthWidth()
{
    if (!m_depth_image_valid) return 0;
    return m_current_depth_image.width();
}

bool RgbdSensor_nwc_ros2::getDepthFOV(double& horizontalFov, double& verticalFov)
{
    if (!m_depth_image_valid && !m_depth_stamp_valid)
    {
        horizontalFov=0;
        verticalFov=0;
        return  false;
        yCError(RGBDSENSOR_NWC_ROS2) << "get depth IntrinsicParam missing information";
    }
    horizontalFov = 2 * atan(m_max_rgb_width / (2 * m_depth_params.focalLengthX)) * 180.0 / M_PI;
    verticalFov = 2 * atan(m_max_rgb_height / (2 * m_depth_params.focalLengthY)) * 180.0 / M_PI;
    return true;
}

bool RgbdSensor_nwc_ros2::getDepthIntrinsicParam(yarp::os::Property& intrinsic)
{
    if(m_depth_stamp_valid)
    {
        intrinsic.clear();
        m_depth_params.toProperty(intrinsic);
        return true;
    }
    return false;
}

double RgbdSensor_nwc_ros2::getDepthAccuracy()
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getDepthAccuracy not supported";
    return 0;
}

bool RgbdSensor_nwc_ros2::getDepthClipPlanes(double& nearPlane, double& farPlane)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getDepthClipPlanes not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::setDepthClipPlanes(double nearPlane, double farPlane)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthClipPlanes not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::getDepthMirroring(bool& mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getDepthMirroring not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::setDepthMirroring(bool mirror)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "setDepthMirroring not supported";
    return false;
}

bool RgbdSensor_nwc_ros2::getExtrinsicParam(yarp::sig::Matrix& extrinsic)
{
    yCWarning(RGBDSENSOR_NWC_ROS2) << "getExtrinsicParam not supported";
    return  false;
}


/*
 * TODO FIXME to update with header instead of stamp
 */
bool RgbdSensor_nwc_ros2::getRgbImage(yarp::sig::FlexImage& rgb_image, yarp::os::Stamp* rgb_image_stamp)
{
    std::lock_guard<std::mutex> guard_rgb_image(m_rgb_image_mutex);
    std::lock_guard<std::mutex> guard_rgb_camera_info(m_rgb_camera_info_mutex);
    bool rgb_ok = false;
    if (m_rgb_image_valid)
    {
        if (m_rgb_stamp_valid)
        {
            if (rgb_image_stamp == nullptr)
            {
                yarp::os::Stamp stamp(m_current_rgb_stamp.getCount(),m_current_rgb_stamp.getTime());
                rgb_image_stamp = &stamp;
            } else {
                rgb_image_stamp->update(m_current_rgb_stamp.getTime());
            }
            yarp::dev::Ros2RGBDConversionUtils::deepCopyFlexImage(m_current_rgb_image, rgb_image);
            rgb_ok = true;
        }
        else
        {
            yCError(RGBDSENSOR_NWC_ROS2, "missing rgb camera info");
        }
    }
    else
    {
        yCError(RGBDSENSOR_NWC_ROS2, "missing rgb image");
    }
    return rgb_ok;
}

/*
 * TODO FIXME to update with header instead of stamp
 */
bool RgbdSensor_nwc_ros2::getDepthImage(depthImage& depth_image, yarp::os::Stamp* depth_image_stamp)
{
    std::lock_guard<std::mutex> guard_depth_image(m_depth_image_mutex);
    std::lock_guard<std::mutex> guard_depth_camera_info(m_depth_camera_info_mutex);
    bool depth_ok =false;

    if (m_depth_image_valid)
    {
        if (m_depth_stamp_valid)
        {
            if (depth_image_stamp == nullptr)
            {
                yarp::os::Stamp stamp(m_current_depth_stamp.getCount(),m_current_depth_stamp.getTime());
                depth_image_stamp = &stamp;
            } else {
                depth_image_stamp->update(m_current_depth_stamp.getTime());
            }
            yarp::dev::Ros2RGBDConversionUtils::deepCopyImageOf(m_current_depth_image, depth_image);
            depth_ok = true;
        }
        else
        {
            yCError(RGBDSENSOR_NWC_ROS2, "missing depth camera info");
        }
    }
    else
    {
        yCError(RGBDSENSOR_NWC_ROS2, "missing depth image");
    }
    return depth_ok;
}
/*
 * TODO FIXME to update with header instead of stamp
 */
bool RgbdSensor_nwc_ros2::getImages(yarp::sig::FlexImage& rgb_image, depthImage& depth_image, yarp::os::Stamp* rgb_image_stamp, yarp::os::Stamp* depth_image_stamp)
{
    bool rgb_ok, depth_ok;
    rgb_ok = getRgbImage(rgb_image, rgb_image_stamp);
    depth_ok = getDepthImage(depth_image, depth_image_stamp);
    return (rgb_ok && depth_ok);
}


RgbdSensor_nwc_ros2::RGBDSensor_status RgbdSensor_nwc_ros2::getSensorStatus()
{
    return RGBD_SENSOR_OK_IN_USE;
}


std::string RgbdSensor_nwc_ros2::getLastErrorMsg(yarp::os::Stamp* timeStamp)
{
    yCError(RGBDSENSOR_NWC_ROS2) << "get last error not yet implemented";
    return "get last error not yet implemented";
}
