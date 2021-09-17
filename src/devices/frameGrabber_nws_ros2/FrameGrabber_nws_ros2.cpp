/*
 * Copyright (C) 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "FrameGrabber_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>

#include <sensor_msgs/image_encodings.hpp>

namespace {
YARP_LOG_COMPONENT(FRAMEGRABBER_NWS_ROS2, "yarp.device.frameGrabber_nws_ros2")

// FIXME Copied from rgbdSensor_nws_ros2
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

FrameGrabber_nws_ros2::FrameGrabber_nws_ros2() :
        PeriodicThread(s_default_period)
{
}


FrameGrabber_nws_ros2::~FrameGrabber_nws_ros2()
{
    close();
}


bool FrameGrabber_nws_ros2::close()
{
    if (!m_active) {
        return false;
    }
    m_active = false;

    detach();


    if (subdevice) {
        subdevice->close();
        delete subdevice;
        subdevice = nullptr;
    }

    isSubdeviceOwned = false;

    return true;
}


bool FrameGrabber_nws_ros2::open(yarp::os::Searchable& config)
{
    if (m_active) {
        yCError(FRAMEGRABBER_NWS_ROS2, "Device is already opened");
        return false;
    }


    // Check "period" option
    if (config.check("period", "refresh period(in s) of the broadcasted values through yarp ports") && config.find("period").isFloat64()) {
        m_period = config.find("period").asFloat64();
    } else {
        yCInfo(FRAMEGRABBER_NWS_ROS2)
            << "Period parameter not found, using default of"
            << s_default_period
            << "seconds";
    }
    PeriodicThread::setPeriod(m_period);


    // Check "node_name" option and open node
    if (!config.check("node_name"))
    {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Missing node_name parameter";
        return false;
    }
    m_nodeName = config.find("node_name").asString();
    if (m_nodeName.c_str()[0] == '/') {
        yCError(FRAMEGRABBER_NWS_ROS2) << "node name cannot begin with /";
        return false;
    }


    // Check "topic_name" option and open publisher
    if (!config.check("topic_name"))
    {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Missing topic_name parameter";
        return false;
    }
    std::string topicName = config.find("topic_name").asString();
    if (topicName.c_str()[0] != '/') {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Missing '/' in topic_name parameter";
        return false;
    }
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    m_node = std::make_shared<rclcpp::Node>(m_nodeName);
    publisher_image = m_node->create_publisher<sensor_msgs::msg::Image>(topicName, 10);


    // set "cameraInfoTopicName" and open publisher
    std::string cameraInfoTopicName = topicName.substr(0,topicName.rfind('/')) + "/camera_info";
    publisher_cameraInfo = m_node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoTopicName, 10);


    // Check "frame_id" option
    if (!config.check("frame_id"))
    {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Missing frame_id parameter";
        return false;
    }
    m_frameId = config.find("frame_id").asString();

    // Check "frame_id" option
    if (!config.check("node_name"))
    {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Missing node_name parameter";
        return false;
    }
    m_nodeName = config.find("node_name").asString();
    if(m_nodeName[0] == '/'){
        yCError(FRAMEGRABBER_NWS_ROS2) << "node_name cannot have an initial /";
        return false;
    }

    // Check "subdevice" option and eventually open the device
    isSubdeviceOwned = config.check("subdevice");
    if (isSubdeviceOwned) {
        yarp::os::Property p;
        subdevice = new yarp::dev::PolyDriver;
        p.fromString(config.toString());
        p.put("pixelType", VOCAB_PIXEL_RGB);
        p.setMonitor(config.getMonitor(), "subdevice"); // pass on any monitoring
        p.unput("device");
        p.put("device", config.find("subdevice").asString()); // subdevice was already checked before

        // if errors occurred during open, quit here.
        subdevice->open(p);

        if (!(subdevice->isValid())) {
            yCError(FRAMEGRABBER_NWS_ROS2, "Unable to open subdevice");
            return false;
        }
        if (!attach(subdevice)) {
            yCError(FRAMEGRABBER_NWS_ROS2, "Unable to attach subdevice");
            return false;
        }
    } else {
        yCInfo(FRAMEGRABBER_NWS_ROS2) << "Running, waiting for attach...";
    }

    m_active = true;

    return true;
}

bool FrameGrabber_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly->isValid()) {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Device " << poly << " to attach to is not valid ... cannot proceed";
        return false;
    }

    poly->view(iRgbVisualParams);
    poly->view(iFrameGrabberImage);
    poly->view(iPreciselyTimed);

    if (iFrameGrabberImage == nullptr) {
        yCError(FRAMEGRABBER_NWS_ROS2) << "IFrameGrabberImage interface is not available on the device";
        return false;
    }

    if (iRgbVisualParams == nullptr) {
        yCWarning(FRAMEGRABBER_NWS_ROS2) << "IRgbVisualParams interface is not available on the device";
    }

    return PeriodicThread::start();
}


bool FrameGrabber_nws_ros2::detach()
{
    if (yarp::os::PeriodicThread::isRunning()) {
        yarp::os::PeriodicThread::stop();
    }

    iRgbVisualParams = nullptr;
    iFrameGrabberImage = nullptr;
    iPreciselyTimed = nullptr;

    return true;
}

bool FrameGrabber_nws_ros2::threadInit()
{
    yarpimg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    return true;
}

void FrameGrabber_nws_ros2::threadRelease()
{
    delete yarpimg;
    yarpimg = nullptr;
}


// Publish the images on the buffered port
void FrameGrabber_nws_ros2::run()
{
//     if (false /* FIXME Can we check if there are subscribers connected in ROS2? */) {
//         // If no subscribers are connected, do not call getImage on the interface.
//         return;
//     }

    if (iPreciselyTimed) {
        m_stamp = iPreciselyTimed->getLastInputStamp();
    } else {
        m_stamp.update(yarp::os::Time::now());
    }

    if (iFrameGrabberImage /* FIXME Can we check if there are subscribers connected in ROS2 */) {
        iFrameGrabberImage->getImage(*yarpimg);
        sensor_msgs::msg::Image rosimg;
        rosimg.data.resize(yarpimg->getRawImageSize());
        rosimg.width = yarpimg->width();
        rosimg.height = yarpimg->height();
        rosimg.encoding = yarp2RosPixelCode(yarpimg->getPixelCode());
        rosimg.step = yarpimg->getRowSize();
        rosimg.header.frame_id = m_frameId;
//         rosimg.header.stamp.sec = static_cast<int>(m_stamp.getTime()); // FIXME
//         rosimg.header.stamp.nanosec = static_cast<int>(1000000 * (m_stamp.getTime() - int(m_stamp.getTime()))); // FIXME
        rosimg.is_bigendian = 0;
        memcpy(rosimg.data.data(), yarpimg->getRawImage(), yarpimg->getRawImageSize());
        publisher_image->publish(rosimg);
    }

    if (iRgbVisualParams /* FIXME Can we check if there are subscribers connected in ROS2 */) {
        sensor_msgs::msg::CameraInfo cameraInfo;
        if (setCamInfo(cameraInfo)) {
            publisher_cameraInfo->publish(cameraInfo);
        }
    }
}

namespace {
template <class T>
struct param
{
    param(T& inVar, std::string inName) :
        var(&inVar),
        parname(std::move(inName))
    {
    }
    T*              var;
    std::string     parname;
};
} // namespace

bool FrameGrabber_nws_ros2::setCamInfo(sensor_msgs::msg::CameraInfo& cameraInfo)
{
    yarp::os::Property camData;
    if (!iRgbVisualParams->getRgbIntrinsicParam(camData)) {
        yCErrorThreadOnce(FRAMEGRABBER_NWS_ROS2) << "Unable to get intrinsic param from rgb sensor!";
        return false;
    }

    if (!camData.check("distortionModel")) {
        yCWarning(FRAMEGRABBER_NWS_ROS2) << "Missing distortion model";
        return false;
    }

    std::string distModel = camData.find("distortionModel").asString();
    if (distModel != "plumb_bob") {
        yCError(FRAMEGRABBER_NWS_ROS2) << "Distortion model not supported";
        return false;
    }

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

    std::vector<param<double>> parVector;
    parVector.emplace_back(phyF,"physFocalLength");
    parVector.emplace_back(fx,"focalLengthX");
    parVector.emplace_back(fy,"focalLengthY");
    parVector.emplace_back(cx,"principalPointX");
    parVector.emplace_back(cy,"principalPointY");
    parVector.emplace_back(k1,"k1");
    parVector.emplace_back(k2,"k2");
    parVector.emplace_back(t1,"t1");
    parVector.emplace_back(t2,"t2");
    parVector.emplace_back(k3,"k3");

    for(auto& par : parVector) {
        if(!camData.check(par.parname)) {
            yCWarning(FRAMEGRABBER_NWS_ROS2) << "Driver has not the param:" << par.parname;
            return false;
        }
        *(par.var) = camData.find(par.parname).asFloat64();
    }

    cameraInfo.header.frame_id      = m_frameId;
//     cameraInfo.header.stamp.sec     = static_cast<int>(m_stamp.getTime()); // FIXME
//     cameraInfo.header.stamp.nanosec = static_cast<int>(1000000 * (m_stamp.getTime() - int(m_stamp.getTime()))); // FIXME
    cameraInfo.width                = iRgbVisualParams->getRgbWidth();
    cameraInfo.height               = iRgbVisualParams->getRgbHeight();
    cameraInfo.distortion_model     = distModel;

    cameraInfo.d.resize(5);
    cameraInfo.d[0] = k1;
    cameraInfo.d[1] = k2;
    cameraInfo.d[2] = t1;
    cameraInfo.d[3] = t2;
    cameraInfo.d[4] = k3;

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
