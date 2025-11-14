/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "RgbdToPointCloudSensor_nws_ros2.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/PointCloudUtils.h>

#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <string>
#include <vector>
#include <Ros2Utils.h>

using namespace std::chrono_literals;
using namespace std;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace RGBDToPointCloudRos2Impl;
YARP_LOG_COMPONENT(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "yarp.ros2.RgbdToPointCloudSensor_nws_ros2", yarp::os::Log::TraceType);

RgbdToPointCloudSensor_nws_ros2::RgbdToPointCloudSensor_nws_ros2() :
        yarp::os::PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}


// DeviceDriver
bool RgbdToPointCloudSensor_nws_ros2::open(yarp::os::Searchable &config)
{
    yCDebug(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Parameters are: " << config.toString();

    if(!fromConfig(config)) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Failed to open, check previous log for error messages.";
        return false;
    }

    if(!initialize_ROS2(config)) {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Error initializing ROS topic";
        return false;
    }

    return true;
}


bool RgbdToPointCloudSensor_nws_ros2::fromConfig(yarp::os::Searchable &config)
{
    parseParams(config);
    setPeriod(m_period);

    return true;
}


bool RgbdToPointCloudSensor_nws_ros2::initialize_ROS2(yarp::os::Searchable &params)
{

    m_node = NodeCreator::createNode(m_node_name);
    m_rosPublisher_pointCloud2 = m_node->create_publisher<sensor_msgs::msg::PointCloud2>(m_topic_name, 10);
    return true;
}


bool RgbdToPointCloudSensor_nws_ros2::close()
{
    yCTrace(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Close");
    detachAll();

    return true;
}


// PeriodicThread
void RgbdToPointCloudSensor_nws_ros2::run()
{
    if (m_sensor_p!=nullptr) {
        static int i = 0;
        switch (m_sensor_p->getSensorStatus()) {
            case(yarp::dev::IRGBDSensor::RGBD_SENSOR_OK_IN_USE) :
            if (!writeData()) {
                yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Image not captured.. check hardware configuration");
            }
            i = 0;
            break;
        case(yarp::dev::IRGBDSensor::RGBD_SENSOR_NOT_READY):
            if(i < 1000) {
                if((i % 30) == 0) {
                    yCInfo(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Device not ready, waiting...";
                    }
                } else {
                    yCWarning(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Device is taking too long to start..";
                }
                i++;
            break;
        default:
            yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Sensor returned error");
        }
    } else {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2, "Sensor interface is not valid");
    }
}


bool RgbdToPointCloudSensor_nws_ros2::attach(yarp::dev::PolyDriver* poly)
{
    if(poly)
    {
        poly->view(m_sensor_p);
        poly->view(m_fgCtrl);
    }

    if(m_sensor_p == nullptr)
    {
        yCError(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Attached device has no valid IRGBDSensor interface.";
        return false;
    }

    if(m_fgCtrl == nullptr)
    {
        yCWarning(RGBDTOPOINTCLOUDSENSOR_NWS_ROS2) << "Attached device has no valid IFrameGrabberControls interface.";
    }

    return PeriodicThread::start();
}


bool RgbdToPointCloudSensor_nws_ros2::detach()
{
    if (yarp::os::PeriodicThread::isRunning())
        yarp::os::PeriodicThread::stop();

    m_sensor_p = nullptr;
    if (m_fgCtrl)
    {
        m_fgCtrl = nullptr;
    }
    return true;
}

bool RgbdToPointCloudSensor_nws_ros2::writeData()
{
    yarp::sig::FlexImage colorImage;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthImage;
    yarp::os::Stamp colorStamp;
    yarp::os::Stamp depthStamp;

    if (!m_sensor_p->getImages(colorImage, depthImage, &colorStamp, &depthStamp)) {
        return false;
    }

    static Stamp oldColorStamp = Stamp(0, 0);
    static Stamp oldDepthStamp = Stamp(0, 0);
    yarp::os::Property propIntrinsic;
    bool rgb_data_ok = true;
    bool depth_data_ok = true;
    bool intrinsic_ok = false;

    if ((colorStamp.getTime() - oldColorStamp.getTime()) <= 0) {
        rgb_data_ok = false;
    } else {
        oldColorStamp = colorStamp;
    }

    if ((depthStamp.getTime() - oldDepthStamp.getTime()) <= 0) {
        depth_data_ok = false;
    } else {
        oldDepthStamp = depthStamp;
    }
    intrinsic_ok = m_sensor_p->getRgbIntrinsicParam(propIntrinsic);


    // TBD: We should check here somehow if the timestamp was correctly updated and, if not, update it ourselves.
    if (rgb_data_ok && m_rosPublisher_pointCloud2->get_subscription_count() > 0) {
        if (depth_data_ok) {
            if (intrinsic_ok) {
                yarp::sig::IntrinsicParams intrinsics(propIntrinsic);
                yarp::sig::ImageOf<yarp::sig::PixelRgb> colorImagePixelRGB;
                colorImagePixelRGB.setExternal(colorImage.getRawImage(), colorImage.width(), colorImage.height());
                // create point cloud in yarp format
                yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloud = yarp::sig::utils::depthRgbToPC<yarp::sig::DataXYZRGBA,
                                                                                                         yarp::sig::PixelRgb>(depthImage,
                                                                                                                              colorImagePixelRGB,
                                                                                                                              intrinsics,
                                                                                                                              yarp::sig::utils::OrganizationType::Unorganized);
                sensor_msgs::msg::PointCloud2 pc2Ros;

                // filling ros header
                pc2Ros.header.frame_id = m_frame_id;

                //pc2Ros.header.stamp.sec = depthStamp.;
                pc2Ros.header.stamp.sec = int(depthStamp.getTime());
                pc2Ros.header.stamp.nanosec = int(1000000000UL * (depthStamp.getTime() - int(depthStamp.getTime())));

                // filling ros point field
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields.push_back(sensor_msgs::msg::PointField());
                pc2Ros.fields[0].name = "x";
                pc2Ros.fields[0].offset = 0;
                pc2Ros.fields[0].datatype = 7;
                pc2Ros.fields[0].count = 1;
                pc2Ros.fields[1].name = "y";
                pc2Ros.fields[1].offset = 4;
                pc2Ros.fields[1].datatype = 7;
                pc2Ros.fields[1].count = 1;
                pc2Ros.fields[2].name = "z";
                pc2Ros.fields[2].offset = 8;
                pc2Ros.fields[2].datatype = 7;
                pc2Ros.fields[2].count = 1;
                pc2Ros.fields[3].name = "rgb";
                pc2Ros.fields[3].offset = 16;
                pc2Ros.fields[3].datatype = 7;
                pc2Ros.fields[3].count = 1;

                std::vector<unsigned char> vec(yarpCloud.getRawData(), yarpCloud.getRawData() + yarpCloud.dataSizeBytes());
                pc2Ros.data = vec;
                pc2Ros.width = yarpCloud.width() * yarpCloud.height();
                pc2Ros.height = 1;
                pc2Ros.is_dense = yarpCloud.isDense();

                pc2Ros.point_step = sizeof(yarp::sig::DataXYZRGBA);
                pc2Ros.row_step = static_cast<std::uint32_t>(sizeof(yarp::sig::DataXYZRGBA) * pc2Ros.width);
                m_rosPublisher_pointCloud2->publish(pc2Ros);
            }
        }
    }

    nodeSeq++;

    return true;
}
