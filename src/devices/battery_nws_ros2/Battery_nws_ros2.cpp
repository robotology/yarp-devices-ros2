/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Battery_nws_ros2.h"
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <Ros2Utils.h>

YARP_LOG_COMPONENT(BATTERY_NWS_ROS2, "yarp.devices.Battery_nws_ros2")

Battery_nws_ros2::Battery_nws_ros2() : yarp::os::PeriodicThread(DEFAULT_THREAD_PERIOD)
{
}

Battery_nws_ros2::~Battery_nws_ros2()
{
    m_battery_interface = nullptr;
}


bool Battery_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{
    if (driver->isValid())
    {
        driver->view(m_battery_interface);
    } else {
        yCError(BATTERY_NWS_ROS2) << "not valid driver";
    }

    if (m_battery_interface == nullptr)
    {
        yCError(BATTERY_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }
    PeriodicThread::setPeriod(m_period);
    return PeriodicThread::start();
}


bool Battery_nws_ros2::detach()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    m_battery_interface = nullptr;
    return true;
}

bool Battery_nws_ros2::threadInit()
{
    return true;
}

bool Battery_nws_ros2::open(yarp::os::Searchable &config)
{
    parseParams(config);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    m_node = NodeCreator::createNode(m_node_name, node_options);
    if (m_node == nullptr) {
        yCError(BATTERY_NWS_ROS2) << " opening " << m_node_name << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }

    m_ros2Publisher = m_node->create_publisher<sensor_msgs::msg::BatteryState>(m_topic_name, 10);
    return true;
}

void Battery_nws_ros2::threadRelease()
{
}

void Battery_nws_ros2::run()
{
    if (m_battery_interface==nullptr)
    {
        yCError(BATTERY_NWS_ROS2) << "the interface is not valid";
    }
    else if (!m_ros2Publisher)
    {
        yCError(BATTERY_NWS_ROS2) << "the publisher is not ready";
    }
    else
    {
        double voltage=0;
        double current=0;
        double charge=0;
        double temperature=0;
        yarp::dev::IBattery::Battery_status status;
        std::string battery_info;
        m_battery_interface->getBatteryVoltage(voltage);
        m_battery_interface->getBatteryCurrent(current);
        m_battery_interface->getBatteryCharge(charge);
        m_battery_interface->getBatteryStatus(status);
        m_battery_interface->getBatteryTemperature(temperature);
        m_battery_interface->getBatteryInfo(battery_info);

        m_timeStamp.update(yarp::os::Time::now());

        battMsg.voltage = voltage;
        battMsg.current = current;
        battMsg.temperature = temperature;
        battMsg.charge = std::numeric_limits<double>::quiet_NaN();//std::nan("");
     //   battMsg.capacity = std::nan("");
      //  battMsg.design_capacity = std::nan("");
        battMsg.percentage = charge;
        battMsg.power_supply_status = 0;
        battMsg.power_supply_health = 0;
        battMsg.power_supply_technology = 0;
        battMsg.present=true;
        battMsg.location="";
        battMsg.serial_number="";

        battMsg.header.frame_id = "" ;
        battMsg.header.stamp.sec = int(m_timeStamp.getTime());
        battMsg.header.stamp.nanosec = int(1000000000UL * (m_timeStamp.getTime() - int(m_timeStamp.getTime())));

        m_ros2Publisher->publish(battMsg);
    }
}

bool Battery_nws_ros2::close()
{
    yCTrace(BATTERY_NWS_ROS2);
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detach();
    return true;
}
