/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_GENERICSENSORS_NWS_ROS2_H
#define YARP_DEV_GENERICSENSORS_NWS_ROS2_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include "GenericSensor_nws_ros2_ParamsParser.h"

#include <rclcpp/rclcpp.hpp>
#include <Ros2Utils.h>


// The log component is defined in each device, with a specialized name
YARP_DECLARE_LOG_COMPONENT(GENERICSENSOR_NWS_ROS2)


/**
 * @ingroup dev_impl_nws_ros2
 *
 * \brief This abstract template needs to be specialized in a ROS Publisher, for a specific ROS mesagge/sensor type.
 *
 * | YARP device name |
 * |:-----------------:|
 * | `GenericSensor_nws_ros2` |
 *
 * The parameters accepted by this device are shown in class: GenericSensor_nws_ros2_ParamsParser
 *
 */

template <class ROS_MSG>
class GenericSensor_nws_ros2 :
        public yarp::os::PeriodicThread,
        public yarp::dev::DeviceDriver,
        public yarp::dev::IMultipleWrapper,
        protected GenericSensor_nws_ros2_ParamsParser
{
protected:
    rclcpp::Node::SharedPtr m_node;
    typename rclcpp::Publisher<ROS_MSG>::SharedPtr m_publisher;
    yarp::dev::PolyDriver* m_poly;
    double                 m_timestamp;
    std::string            m_framename;
    const size_t           m_sens_index = 0;
    yarp::dev::PolyDriver  m_subdevicedriver;

public:
    GenericSensor_nws_ros2();
    virtual ~GenericSensor_nws_ros2();

    /* DevideDriver methods */
    bool open(yarp::os::Searchable &params) override;
    bool close() override;

    /* IMultipleWrapper methods */
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;

    /* PeriodicRateThread methods */
    void threadRelease() override;
    void run() override;

protected:
    virtual bool viewInterfaces() = 0;
};

template <class ROS_MSG>
GenericSensor_nws_ros2<ROS_MSG>::GenericSensor_nws_ros2() :
    PeriodicThread(0.02)
{
    m_node = nullptr;
    m_publisher=nullptr;
    m_poly = nullptr;
    m_timestamp=0;
}

template <class ROS_MSG>
GenericSensor_nws_ros2<ROS_MSG>::~GenericSensor_nws_ros2() = default;

template <class ROS_MSG>
bool GenericSensor_nws_ros2<ROS_MSG>::open(yarp::os::Searchable & config)
{
    parseParams(config);
    if (m_node_name.c_str()[0] == '/') {
        yCError(GENERICSENSOR_NWS_ROS2) << "node name cannot begin with /";
        return false;
    }

    if (m_topic_name.c_str()[0] != '/') {
        yCError(GENERICSENSOR_NWS_ROS2) << "Missing '/' in topic_name parameter";
        return false;
    }

    m_node = NodeCreator::createNode(m_node_name); // add a ROS node
    m_publisher = m_node->create_publisher<ROS_MSG>(m_topic_name,rclcpp::QoS(10));

    if (m_node == nullptr) {
        yCError(GENERICSENSOR_NWS_ROS2) << "Opening " << m_node_name << " Node, check your yarp-ROS network configuration\n";
        return false;
    }

    if (m_publisher == nullptr) {
        yCError(GENERICSENSOR_NWS_ROS2) << "Opening " << m_topic_name << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }

    yCInfo(GENERICSENSOR_NWS_ROS2) << "Running, waiting for attach...";

    return true;
}

template <class ROS_MSG>
bool GenericSensor_nws_ros2<ROS_MSG>::close()
{
    return this->detachAll();
}

template <class ROS_MSG>
bool GenericSensor_nws_ros2<ROS_MSG>::attachAll(const yarp::dev::PolyDriverList & p)
{
    // Attach the device
    if (p.size() > 1)
    {
        yCError(GENERICSENSOR_NWS_ROS2, "This device only supports exposing a "
            "single MultipleAnalogSensors device on YARP ports, but %d devices have been passed in attachAll.",
            p.size());
        yCError(GENERICSENSOR_NWS_ROS2, "Please use the multipleanalogsensorsremapper device to combine several device in a new device.");
        detachAll();
        return false;
    }

    if (p.size() == 0)
    {
        yCError(GENERICSENSOR_NWS_ROS2, "No device passed to attachAll, please pass a device to expose on YARP ports.");
        return false;
    }

    m_poly = p[0]->poly;

    if (!m_poly)
    {
        yCError(GENERICSENSOR_NWS_ROS2, "Null pointer passed to attachAll.");
        return false;
    }

    // View all the interfaces
    bool ok = viewInterfaces();

    // Set rate period
    ok &= this->setPeriod(m_period);
    ok &= this->start();

    return ok;
}

template <class ROS_MSG>
bool GenericSensor_nws_ros2<ROS_MSG>::detachAll()
{
    // Stop the thread on detach
    if (this->isRunning()) {
        this->stop();
    }
    return true;
}

template <class ROS_MSG>
void GenericSensor_nws_ros2<ROS_MSG>::run()
{
}

template <class ROS_MSG>
void GenericSensor_nws_ros2<ROS_MSG>::threadRelease()
{
    return;
}

#endif
