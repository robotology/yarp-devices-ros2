/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Ros2Wrapper.h"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std::chrono_literals;
using namespace yarp::os;
using namespace yarp::dev;


YARP_LOG_COMPONENT(ROS2WRAPPER, "yarp.ros2.ros2wrapper", yarp::os::Log::TraceType);


Ros2Init::Ros2Init()
{
    rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    node = std::make_shared<rclcpp::Node>("yarprobotinterface_node");
}

Ros2Init& Ros2Init::get()
{
    static Ros2Init instance;
    return instance;
}


Ros2Wrapper::Ros2Wrapper() :
        yarp::os::PeriodicThread(0.5)
{
}

bool Ros2Wrapper::attachAll(const PolyDriverList &device2attach)
{
    if (device2attach.size() != 1)
    {
        yCError(ROS2WRAPPER, "Rangefinder2DWrapper: cannot attach more than one device");
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach = device2attach[0]->poly;

    if (Idevice2attach->isValid())
    {
        Idevice2attach->view(iDevice);
    }

    if (nullptr == iDevice)
    {
        yCError(ROS2WRAPPER, "Rangefinder2DWrapper: subdevice passed to attach method is invalid");
        return false;
    }
    attach(iDevice);

    if(!iDevice->getDistanceRange(minDistance, maxDistance))
    {
        yCError(ROS2WRAPPER) << "Laser device does not provide min & max distance range.";
        return false;
    }

    if(!iDevice->getScanLimits(minAngle, maxAngle))
    {
        yCError(ROS2WRAPPER) << "Laser device does not provide min & max angle scan range.";
        return false;
    }

    if (!iDevice->getHorizontalResolution(resolution))
    {
        yCError(ROS2WRAPPER) << "Laser device does not provide horizontal resolution ";
        return false;
    }
    
    PeriodicThread::setPeriod(0.1);
    return PeriodicThread::start();
}

bool Ros2Wrapper::detachAll()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    iDevice = nullptr;
    return true;
}

void Ros2Wrapper::attach(yarp::dev::IRangefinder2D *s)
{
    iDevice = s;
}

void Ros2Wrapper::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    iDevice = nullptr;
}

void Ros2Wrapper::run()
{
    yCTrace(ROS2WRAPPER);
    auto message = std_msgs::msg::String();
    
    if (iDevice!=nullptr)
    {
        bool ret = true;
        IRangefinder2D::Device_status status;
        yarp::sig::Vector ranges;
        ret &= iDevice->getRawData(ranges);
        ret &= iDevice->getDeviceStatus(status);
        
		if (ret)
		{
//			if(iTimed)
	//			lastStateStamp = iTimed->getLastInputStamp();
		//	else
	//			lastStateStamp.update(yarp::os::Time::now());

			int ranges_size = ranges.size();

			sensor_msgs::msg::LaserScan rosData; //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ CHECK HERE
			
			//rosData.header.seq = rosMsgCounter++; //"""""""""""""""NON C'e PIU
			rosData.header.stamp = 0;//00lastStateStamp.getTime();
			rosData.header.frame_id = frame_id;

			rosData.angle_min = minAngle * M_PI / 180.0;
			rosData.angle_max = maxAngle * M_PI / 180.0;
			rosData.angle_increment = resolution * M_PI / 180.0;
			rosData.time_increment = 0;             // all points in a single scan are considered took at the very same time
			rosData.scan_time = getPeriod();        // time elapsed between two successive readings
			rosData.range_min = minDistance;
			rosData.range_max = maxDistance;
			rosData.ranges.resize(ranges_size);
			rosData.intensities.resize(ranges_size);

			for (int i = 0; i < ranges_size; i++)
			{
				// in yarp, NaN is used when a scan value is missing. For example when the angular range of the rangefinder is smaller than 360.
				// is ros, NaN is not used. Hence this check replaces NaN with inf.
				if (std::isnan(ranges[i]))
				{
				   rosData.ranges[i] = std::numeric_limits<double>::infinity();
				   rosData.intensities[i] = 0.0;
				}
				else
				{
				   rosData.ranges[i] = ranges[i];
				   rosData.intensities[i] = 0.0;
				}
			}
			m_publisher->publish(rosData);
		}
		else
		{
			yCError(ROS2WRAPPER, "Rangefinder2DWrapper: %s: Sensor returned error", sensorId.c_str());
		}
	}
}

bool Ros2Wrapper::open(yarp::os::Searchable &config)
{
    if(config.check("subdevice"))
    {
        Property       p;
        PolyDriverList driverlist;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if(!m_driver.open(p) || !m_driver.isValid())
        {
            yCError(ROS2WRAPPER) << "RangeFinder2DWrapper: failed to open subdevice.. check params";
            return false;
        }

        driverlist.push(&m_driver, "1");
        if(!attachAll(driverlist))
        {
            yCError(ROS2WRAPPER) << "RangeFinder2DWrapper: failed to open subdevice.. check params";
            return false;
        }
        isDeviceOwned = true;
    }
    
    yCTrace(ROS2WRAPPER);
    m_topic = config.check("topic", yarp::os::Value("ros2test_topic"), "Name of the ROS topic").asString();
    yCInfo(ROS2WRAPPER, "Ros2Test::open - %s", m_topic.c_str());

    m_publisher = Ros2Init::get().node->create_publisher<std_msgs::msg::String>(m_topic, 10);

    start();
    return true;
}
