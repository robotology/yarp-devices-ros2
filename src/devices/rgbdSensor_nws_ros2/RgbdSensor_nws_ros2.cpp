/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "RgbdSensor_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <string>
#include <vector>
#include <iostream>
#include <Ros2Utils.h>

#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace
{
YARP_LOG_COMPONENT(RGBDSENSOR_NWS_ROS2, "yarp.ros2.rgbdSensor_nws_ros2", yarp::os::Log::TraceType);

constexpr double DEFAULT_THREAD_PERIOD = 0.03; // s

// FIXME should be possible to set different frame_id for rgb and depth
const std::string frameId_param            = "frame_Id";
const std::string nodeName_param           = "nodeName";
const std::string colorTopicName_param     = "colorTopicName";
const std::string depthTopicName_param     = "depthTopicName";
const std::string depthInfoTopicName_param = "depthInfoTopicName";
const std::string colorInfoTopicName_param = "colorInfoTopicName";



std::string yarp2RosPixelCode(int code)
{
    switch (code)
    {
    case VOCAB_PIXEL_BGR:
        return sensor_msgs::image_encodings::BGR8;
    case VOCAB_PIXEL_BGRA:
        return sensor_msgs::image_encodings::BGRA8;
    case VOCAB_PIXEL_ENCODING_BAYER_BGGR16:
        return sensor_msgs::image_encodings::BAYER_BGGR16;
    case VOCAB_PIXEL_ENCODING_BAYER_BGGR8:
        return sensor_msgs::image_encodings::BAYER_BGGR8;
    case VOCAB_PIXEL_ENCODING_BAYER_GBRG16:
        return sensor_msgs::image_encodings::BAYER_GBRG16;
    case VOCAB_PIXEL_ENCODING_BAYER_GBRG8:
        return sensor_msgs::image_encodings::BAYER_GBRG8;
    case VOCAB_PIXEL_ENCODING_BAYER_GRBG16:
        return sensor_msgs::image_encodings::BAYER_GRBG16;
    case VOCAB_PIXEL_ENCODING_BAYER_GRBG8:
        return sensor_msgs::image_encodings::BAYER_GRBG8;
    case VOCAB_PIXEL_ENCODING_BAYER_RGGB16:
        return sensor_msgs::image_encodings::BAYER_RGGB16;
    case VOCAB_PIXEL_ENCODING_BAYER_RGGB8:
        return sensor_msgs::image_encodings::BAYER_RGGB8;
    case VOCAB_PIXEL_MONO:
        return sensor_msgs::image_encodings::MONO8;
    case VOCAB_PIXEL_MONO16:
        return sensor_msgs::image_encodings::MONO16;
    case VOCAB_PIXEL_RGB:
        return sensor_msgs::image_encodings::RGB8;
    case VOCAB_PIXEL_RGBA:
        return sensor_msgs::image_encodings::RGBA8;
    case VOCAB_PIXEL_MONO_FLOAT:
        return sensor_msgs::image_encodings::TYPE_32FC1;
    default:
        return sensor_msgs::image_encodings::RGB8;
    }
}

} // namespace


RgbdSensor_nws_ros2::RgbdSensor_nws_ros2() :
        yarp::os::PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}


// DeviceDriver
bool RgbdSensor_nws_ros2::open(yarp::os::Searchable &config)
{
    yCDebug(RGBDSENSOR_NWS_ROS2) << "Parameters are: " << config.toString();

    if(!fromConfig(config)) {
        yCError(RGBDSENSOR_NWS_ROS2) << "Failed to open, check previous log for error messages.";
        return false;
    }

    if(!initialize_ROS2(config)) {
        yCError(RGBDSENSOR_NWS_ROS2) << "Error initializing ROS topic";
        return false;
    }

    return true;
}



bool RgbdSensor_nws_ros2::fromConfig(yarp::os::Searchable &config)
{
    parseParams(config);
    if(m_node_name[0] == '/'){
        yCError(RGBDSENSOR_NWS_ROS2) << "node_name cannot begin with an initial /";
        return false;
    }

    if(m_depth_topic_name[0] != '/'){
        yCError(RGBDSENSOR_NWS_ROS2) << "depth_topic_name must begin with an initial /";
        return false;
    }
    m_depth_info_topic_name = m_depth_topic_name.substr(0, m_depth_topic_name.rfind('/')) + "/camera_info";

    if(m_color_topic_name[0] != '/'){
        yCError(RGBDSENSOR_NWS_ROS2) << "color_topic_name must begin with an initial /";
        return false;
    }
    m_color_info_topic_name = m_color_topic_name.substr(0, m_color_topic_name.rfind('/')) + "/camera_info";

    m_forceInfoSync = m_force_info_synch == 1;

    return true;
}


bool RgbdSensor_nws_ros2::initialize_ROS2(yarp::os::Searchable &params)
{
    m_node = NodeCreator::createNode(m_node_name);
    rosPublisher_color = m_node->create_publisher<sensor_msgs::msg::Image>(m_color_topic_name, 10);
    rosPublisher_depth = m_node->create_publisher<sensor_msgs::msg::Image>(m_depth_topic_name, 10);
    rosPublisher_colorCaminfo = m_node->create_publisher<sensor_msgs::msg::CameraInfo>(m_color_info_topic_name, 10);
    rosPublisher_depthCaminfo = m_node->create_publisher<sensor_msgs::msg::CameraInfo>(m_depth_info_topic_name, 10);
    return true;
}


bool RgbdSensor_nws_ros2::close()
{
    yCTrace(RGBDSENSOR_NWS_ROS2, "Close");
    detach();

    sensor_p = nullptr;
    fgCtrl = nullptr;

    return true;
}

// PeriodicThread

void RgbdSensor_nws_ros2::run()
{
    if (sensor_p!=nullptr) {
        static int i = 0;
        switch (sensor_p->getSensorStatus()) {
            case(yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE) :
            if (!writeData()) {
                yCError(RGBDSENSOR_NWS_ROS2, "Image not captured.. check hardware configuration");
            }
            i = 0;
            break;
        case(yarp::dev::IRGBDSensor::RGBD_SENSOR_NOT_READY):
            if(i < 1000) {
                if((i % 30) == 0) {
                    yCInfo(RGBDSENSOR_NWS_ROS2) << "Device not ready, waiting...";
                    }
                } else {
                    yCWarning(RGBDSENSOR_NWS_ROS2) << "Device is taking too long to start..";
                }
                i++;
            break;
        default:
            yCError(RGBDSENSOR_NWS_ROS2, "Sensor returned error");
        }
    } else {
        yCError(RGBDSENSOR_NWS_ROS2, "Sensor interface is not valid");
    }
}


/*
 * WrapperSingle interface
 */

bool RgbdSensor_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    if(poly)
    {
        poly->view(sensor_p);
        poly->view(fgCtrl);
    }

    if(sensor_p == nullptr)
    {
        yCError(RGBDSENSOR_NWS_ROS2) << "Attached device has no valid IRGBDSensor interface.";
        return false;
    }

    if(fgCtrl == nullptr)
    {
        yCWarning(RGBDSENSOR_NWS_ROS2) << "Attached device has no valid IFrameGrabberControls interface.";
    }

    return PeriodicThread::start();
}


bool RgbdSensor_nws_ros2::detach()
{
    if (yarp::os::PeriodicThread::isRunning())
        yarp::os::PeriodicThread::stop();

    sensor_p = nullptr;
    if (fgCtrl)
    {
        fgCtrl = nullptr;
    }
    return true;
}

bool RgbdSensor_nws_ros2::setCamInfo(sensor_msgs::msg::CameraInfo& cameraInfo,
                                     const std::string& frame_id,
                                     const SensorType& sensorType)
{
    double phyF = 0.0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double t1 = 0.0;
    double t2 = 0.0;
    double k3 = 0.0;

    std::string                  distModel;
    std::string currentSensor;
    yarp::os::Property                camData;
    std::vector<param<double> >  parVector;
    bool                    ok;

    currentSensor = sensorType == COLOR_SENSOR ? "Rgb" : "Depth";
    ok            = sensorType == COLOR_SENSOR ? sensor_p->getRgbIntrinsicParam(camData) : sensor_p->getDepthIntrinsicParam(camData);

    if (!ok)
    {
        yCError(RGBDSENSOR_NWS_ROS2) << "Unable to get intrinsic param from" << currentSensor << "sensor!";
        return false;
    }

    if(!camData.check("distortionModel"))
    {
        yCWarning(RGBDSENSOR_NWS_ROS2) << "Missing distortion model";
        return false;
    }

    distModel = camData.find("distortionModel").asString();

    parVector.emplace_back(phyF,"physFocalLength");
    parVector.emplace_back(fx,"focalLengthX");
    parVector.emplace_back(fy,"focalLengthY");
    parVector.emplace_back(cx,"principalPointX");
    parVector.emplace_back(cy,"principalPointY");

    if (distModel != "none")
    {
        parVector.emplace_back(k1,"k1");
        parVector.emplace_back(k2,"k2");
        parVector.emplace_back(t1,"t1");
        parVector.emplace_back(t2,"t2");
        parVector.emplace_back(k3,"k3");
    }

    for(auto& par : parVector) {
        if(!camData.check(par.parname)) {
            yCWarning(RGBDSENSOR_NWS_ROS2) << "Driver has not the param:" << par.parname;
            return false;
        }
        *(par.var) = camData.find(par.parname).asFloat64();
    }

    cameraInfo.header.frame_id      = frame_id;
    cameraInfo.width                = sensorType == COLOR_SENSOR ? sensor_p->getRgbWidth() : sensor_p->getDepthWidth();
    cameraInfo.height               = sensorType == COLOR_SENSOR ? sensor_p->getRgbHeight() : sensor_p->getDepthHeight();
    cameraInfo.distortion_model     = distModel;

    if (distModel != "none")
    {
        cameraInfo.d.resize(5);
        cameraInfo.d[0] = k1;
        cameraInfo.d[1] = k2;
        cameraInfo.d[2] = t1;
        cameraInfo.d[3] = t2;
        cameraInfo.d[4] = k3;
    }

    cameraInfo.k[0]  = fx;       cameraInfo.k[1] = 0;        cameraInfo.k[2] = cx;
    cameraInfo.k[3]  = 0;        cameraInfo.k[4] = fy;       cameraInfo.k[5] = cy;
    cameraInfo.k[6]  = 0;        cameraInfo.k[7] = 0;        cameraInfo.k[8] = 1;

    /*
     * ROS documentation on cameraInfo message:
     * "Rectification matrix (stereo cameras only)
     * A rotation matrix aligning the camera coordinate system to the ideal
     * stereo image plane so that epipolar lines in both stereo images are
     * parallel."
     * useless in our case, it will be an identity matrix
     */

    cameraInfo.r[0]  = 1;        cameraInfo.r[1] = 0;        cameraInfo.r[2] = 0;
    cameraInfo.r[3]  = 0;        cameraInfo.r[4] = 1;        cameraInfo.r[5] = 0;
    cameraInfo.r[6]  = 0;        cameraInfo.r[7] = 0;        cameraInfo.r[8] = 1;

    cameraInfo.p[0]  = fx;      cameraInfo.p[1] = 0;    cameraInfo.p[2]  = cx;  cameraInfo.p[3]  = 0;
    cameraInfo.p[4]  = 0;       cameraInfo.p[5] = fy;   cameraInfo.p[6]  = cy;  cameraInfo.p[7]  = 0;
    cameraInfo.p[8]  = 0;       cameraInfo.p[9] = 0;    cameraInfo.p[10] = 1;   cameraInfo.p[11] = 0;

    cameraInfo.binning_x  = cameraInfo.binning_y = 0;
    cameraInfo.roi.height = cameraInfo.roi.width = cameraInfo.roi.x_offset = cameraInfo.roi.y_offset = 0;
    cameraInfo.roi.do_rectify = false;
    return true;
}


bool RgbdSensor_nws_ros2::writeData()
{
    yarp::sig::FlexImage colorImage;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
    yarp::os::Stamp colorStamp;
    yarp::os::Stamp depthStamp;

    if (!sensor_p->getImages(colorImage, depthImage, &colorStamp, &depthStamp)) {
        return false;
    }

    static yarp::os::Stamp oldColorStamp = yarp::os::Stamp(0, 0);
    static yarp::os::Stamp oldDepthStamp = yarp::os::Stamp(0, 0);
    bool rgb_data_ok = true;
    bool depth_data_ok = true;

    if (((colorStamp.getTime() - oldColorStamp.getTime()) > 0) == false) {
        rgb_data_ok=false;
        //return true;
    } else {
        oldColorStamp = colorStamp;
    }

    if (((depthStamp.getTime() - oldDepthStamp.getTime()) > 0) == false) {
        depth_data_ok=false;
        //return true;
    } else {
        oldDepthStamp = depthStamp;
    }

    // Call setCamInfo only the first time
    if (m_firstTime) {
        setCamInfo(m_camInfoData.colorCamInfo, "color_frame", COLOR_SENSOR);
        setCamInfo(m_camInfoData.depthCamInfo, "depth_frame", DEPTH_SENSOR);
        m_firstTime = false;
    }

    // Use m_camInfoData for subsequent calls
    if (rgb_data_ok) {
        sensor_msgs::msg::Image rColorImage;
        rColorImage.data.resize(colorImage.getRawImageSize());
        rColorImage.width = colorImage.width();
        rColorImage.height = colorImage.height();
        memcpy(rColorImage.data.data(), colorImage.getRawImage(), colorImage.getRawImageSize());
        rColorImage.encoding = yarp2RosPixelCode(colorImage.getPixelCode());
        rColorImage.step = colorImage.getRowSize();
        rColorImage.header.frame_id = m_color_frame_id;
        rColorImage.header.stamp.sec = static_cast<int>(colorStamp.getTime()); // FIXME
        rColorImage.header.stamp.nanosec = static_cast<int>(1000000000UL * (colorStamp.getTime() - int(colorStamp.getTime()))); // FIXME
        rColorImage.is_bigendian = 0;

        rosPublisher_color->publish(rColorImage);

        if (m_forceInfoSync) {
            m_camInfoData.colorCamInfo.header.stamp = rColorImage.header.stamp;
        }
        rosPublisher_colorCaminfo->publish(m_camInfoData.colorCamInfo);
    }

    if (depth_data_ok)
    {
        sensor_msgs::msg::Image rDepthImage;
        rDepthImage.data.resize(depthImage.getRawImageSize());
        rDepthImage.width = depthImage.width();
        rDepthImage.height = depthImage.height();
        memcpy(rDepthImage.data.data(), depthImage.getRawImage(), depthImage.getRawImageSize());
        rDepthImage.encoding = yarp2RosPixelCode(depthImage.getPixelCode());
        rDepthImage.step = depthImage.getRowSize();
        rDepthImage.header.frame_id = m_depth_frame_id;
        rDepthImage.header.stamp.sec = static_cast<int>(depthStamp.getTime()); // FIXME
        rDepthImage.header.stamp.nanosec = static_cast<int>(1000000000UL * (depthStamp.getTime() - int(depthStamp.getTime()))); // FIXME
        rDepthImage.is_bigendian = 0;

        rosPublisher_depth->publish(rDepthImage);

        if (m_forceInfoSync) {
            m_camInfoData.depthCamInfo.header.stamp = rDepthImage.header.stamp;
        }
        rosPublisher_depthCaminfo->publish(m_camInfoData.depthCamInfo);
    }

    return true;
}
