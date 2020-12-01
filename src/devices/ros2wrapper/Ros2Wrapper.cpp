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

YARP_LOG_COMPONENT(ROS2WRAPPER, "yarp.ros2.ros2wrapper", yarp::os::Log::TraceType);

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

    //if(!sens_p->getDistanceRange(minDistance, maxDistance))
    //{
    //    yCError(ROS2WRAPPER) << "Unable to get data";
    //    return false;
    //}

    //PeriodicThread::setPeriod(_period);
    //return PeriodicThread::start();
    return true;
}

bool Ros2Wrapper::detachAll()
{
    //if (PeriodicThread::isRunning())
    //
    //    PeriodicThread::stop();
    //}
    iDevice = nullptr;
    return true;
}

void Ros2Wrapper::attach(yarp::dev::IRangefinder2D *s)
{
    iDevice = s;
}

void Ros2Wrapper::detach()
{
    //if (PeriodicThread::isRunning())
    //{
    //    PeriodicThread::stop();
    //}
    iDevice = nullptr;
}

bool Rangefinder2DWrapper::open(yarp::os::Searchable &config)
{
    if(config.check("subdevice"))
    {
        Property       p;
        PolyDriverList driverlist;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if(!driver.open(p) || !driver.isValid())
        {
            yCError(ROS2WRAPPER) << "RangeFinder2DWrapper: failed to open subdevice.. check params";
            return false;
        }

        driverlist.push(&driver, "1");
        if(!attachAll(driverlist))
        {
            yCError(ROS2WRAPPER) << "RangeFinder2DWrapper: failed to open subdevice.. check params";
            return false;
        }
        isDeviceOwned = true;
    }
    
    Ros2Test::open(config);
    
    return true;
}
