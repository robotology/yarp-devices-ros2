/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_GENERICSENSORS_NWC_ROS2_H
#define YARP_DEV_GENERICSENSORS_NWC_ROS2_H

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <mutex>
#include "GenericSensor_nwc_ros2_ParamsParser.h"

#include <rclcpp/rclcpp.hpp>
#include <Ros2Utils.h>
#include <Ros2Spinner.h>


// The log component is defined in each device, with a specialized name
YARP_DECLARE_LOG_COMPONENT(GENERICSENSOR_NWC_ROS2)


/**
 * @ingroup dev_impl_wrapper
 *
 * \brief This abstract template needs to be specialized in a ROS2 subscription, for a specific ROS2 message/sensor type.
 *
 * | YARP device name |
 * |:-----------------:|
 * | `genericSensor_nwc_ros2` |
 *
 * The parameters accepted by this device are shown in class: GenericSensor_nwc_ros2_ParamsParser
 *
 */

template <class ROS_MSG>
class GenericSensor_nwc_ros2 :
        public yarp::dev::DeviceDriver,
        protected GenericSensor_nwc_ros2_ParamsParser
{
protected:
    double        m_periodInS{0.01};
    double        m_timestamp;
    std::string   m_framename;
    Ros2Spinner*  m_spinner{nullptr};
    const size_t  m_sens_index = 0;
    mutable std::mutex      m_dataMutex;
    yarp::dev::MAS_status   m_internalStatus;
    rclcpp::Node::SharedPtr m_node;
    typename rclcpp::Subscription<ROS_MSG>::SharedPtr m_subscription;

    // Subscription callback. To be implemented for each derived device
    virtual void subscription_callback(const std::shared_ptr<ROS_MSG> msg)=0;

public:
    GenericSensor_nwc_ros2();
    virtual ~GenericSensor_nwc_ros2();

    /* DevideDriver methods */
    bool open(yarp::os::Searchable &params) override;
    bool close() override;
};

template <class ROS_MSG>
GenericSensor_nwc_ros2<ROS_MSG>::GenericSensor_nwc_ros2()
{
    m_node = nullptr;
    m_subscription=nullptr;
    m_timestamp=0;
}

template <class ROS_MSG>
GenericSensor_nwc_ros2<ROS_MSG>::~GenericSensor_nwc_ros2() = default;

template <class ROS_MSG>
bool GenericSensor_nwc_ros2<ROS_MSG>::open(yarp::os::Searchable & config)
{
    parseParams(config);
    if (m_node_name.c_str()[0] == '/') {
        yCError(GENERICSENSOR_NWC_ROS2) << "node name cannot begin with /";
        return false;
    }

    if (m_topic_name.c_str()[0] != '/') {
        yCError(GENERICSENSOR_NWC_ROS2) << "Missing '/' in topic_name parameter";
        return false;
    }

    m_sensor_name = config.find("sensor_name").asString();

    m_node = NodeCreator::createNode(m_node_name);
    m_subscription = m_node->create_subscription<ROS_MSG>(m_topic_name, rclcpp::QoS(10),
                                                          std::bind(&GenericSensor_nwc_ros2<ROS_MSG>::subscription_callback,
                                                          this, std::placeholders::_1));

    if (m_node == nullptr) {
        yCError(GENERICSENSOR_NWC_ROS2) << "Opening " << m_node_name << " Node creation failed, check your yarp-ROS network configuration\n";
        return false;
    }

    if (m_subscription == nullptr) {
        yCError(GENERICSENSOR_NWC_ROS2) << "Opening " << m_topic_name << " Topic failed, check your yarp-ROS network configuration\n";
        return false;
    }

    m_internalStatus = yarp::dev::MAS_status::MAS_WAITING_FOR_FIRST_READ;
    m_spinner = new Ros2Spinner(m_node);
    m_spinner->start();

    yCInfo(GENERICSENSOR_NWC_ROS2) << "Device opened";

    return true;
}

template <class ROS_MSG>
bool GenericSensor_nwc_ros2<ROS_MSG>::close()
{
    yCInfo(GENERICSENSOR_NWC_ROS2) << "Closing...";
    delete m_spinner;
    yCInfo(GENERICSENSOR_NWC_ROS2) << "Closed";
    return true;
}

#endif
