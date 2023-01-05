/*
 * Copyright (C) 2006-2023 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef YARP_DEV_MAP2D_NWS_ROS2_H
#define YARP_DEV_MAP2D_NWS_ROS2_H

#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <mutex>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/Map2DLocation.h>
#include <yarp/dev/Map2DArea.h>
#include <yarp/dev/Map2DPath.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/api.h>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <test_msgs/srv/basic_types.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

//Custom ros2 interfaces
#include <map2d_nws_ros2_msgs/srv/get_map_by_name.hpp>


/**
 *  @ingroup dev_impl_nws_ros2 dev_impl_navigation
 *
 * \section Map2D_nws_ros2
 *
 * \brief `Map2D_nws_ros2`: A device capable of read/save collections of maps from disk, and make them accessible to any Map2DClient device.
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter  | Type    | Units   |     Default Value     | Required    | Description                                                   |                                           Notes                                                 |
 * |:--------------:|:-------------:|:-------:|:-------:|:---------------------:|:----------: |:-------------------------------------------------------------:|:-----------------------------------------------------------------------------------------------:|
 * | name           |      -        | string  | -       | map2D_nws_ros         | No          | Device name prefix                                            |                                                                                                 |
 * | getmap         |      -        | string  | -       | getMap                | No          | The "GetMap" ROS service name                                 |               For the moment being the service always responds with an empty map                |
 * | getmapbyname   |      -        | string  | -       | getMapByName          | No          | The "GetMapByName" ROS2  custom service name                  | The map returned by this service is also available via publisher named "getmapbyname value"/pub |
 * | roscmdparser   |      -        | string  | -       | rosCmdParser          | No          | The "BasicTypes" ROS service name                             |             This is used to send commands to the nws via ros2 BasicTypes service                |
 * | markers_pub    |      -        | string  | -       | locationServerMarkers | No          | The visual markers array publisher name                       |                                                                                                 |
 * | node_name      |      -        | string  | -       |         -             | No          | The ROS2 node name. If absent, the device name will be used   |                                                                                                 |

 * \section Notes:
 * Integration with ROS2 map server is currently under development.
 */

class Map2D_nws_ros2 :
        public yarp::os::Thread,
        public yarp::dev::DeviceDriver,
        public yarp::os::PortReader,
        public yarp::dev::WrapperSingle
{
public:
    Map2D_nws_ros2();
    ~Map2D_nws_ros2() override = default;


    //IMultipleWrapper
    bool attach(yarp::dev::PolyDriver* driver) override;
    bool detach() override;

    // DeviceDriver
    bool open(yarp::os::Searchable &config) override;
    bool close() override;
    bool read(yarp::os::ConnectionReader& connection) override;

    //Other stuff
    void getMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                        std::shared_ptr<nav_msgs::srv::GetMap::Response> response);
    void getMapByNameCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<map2d_nws_ros2_msgs::srv::GetMapByName::Request> request,
                              std::shared_ptr<map2d_nws_ros2_msgs::srv::GetMapByName::Response> response);
    void rosCmdParserCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<test_msgs::srv::BasicTypes::Request> request,
                         std::shared_ptr<test_msgs::srv::BasicTypes::Response> response);
    bool updateVizMarkers();

    void run() override;

private:
    //drivers and interfaces
    yarp::dev::Nav2D::IMap2D*    m_iMap2D = nullptr;
    yarp::dev::PolyDriver        m_drv;

    std::mutex                   m_mutex;
    std::string                  m_name{"map2D_nws_ros"};
    std::string                  m_rpcPortName;
    std::string                  m_rosCmdParserName{"rosCmdParser"};
    std::string                  m_getMapName{"getMap"};
    std::string                  m_getMapByNameName{"getMapByName"};
    std::string                  m_markersName{"locationServerMarkers"};
    std::string                  m_currentMapName{"none"};
    std::string                  m_nodeName;
    bool                         m_spinned{false};

    yarp::os::RpcServer                                                    m_rpcPort;
    rclcpp::Node::SharedPtr                                                m_node;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr     m_ros2Publisher_markers{nullptr};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr             m_ros2Publisher_map{nullptr};
    rclcpp::Service<nav_msgs::srv::GetMap>::SharedPtr                      m_ros2Service_getMap{nullptr};
    rclcpp::Service<map2d_nws_ros2_msgs::srv::GetMapByName>::SharedPtr     m_ros2Service_getMapByName{nullptr};
    rclcpp::Service<test_msgs::srv::BasicTypes>::SharedPtr                 m_ros2Service_rosCmdParser{nullptr};
};


#endif  //YARP_DEV_MAP2D_NWS_ROS2_H
