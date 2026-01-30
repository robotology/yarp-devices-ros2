/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "Map2D_nws_ros2.h"

#include <chrono>
#include <vector>
#include <cmath>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/GenericVocabs.h>
#include <yarp/math/Math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <mutex>
#include <cstdlib>
#include <fstream>
#include <Ros2Utils.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/version.h>

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace yarp::os;
using namespace std;
using namespace std::placeholders;

namespace {
YARP_LOG_COMPONENT(MAP2D_NWS_ROS2, "yarp.device.map2D_nws_ros2")
}


Map2D_nws_ros2::Map2D_nws_ros2()
{}

bool Map2D_nws_ros2::attach(yarp::dev::PolyDriver* driver)
{
    if (driver->isValid())
    {
        driver->view(m_iMap2D);
        vector<string> maps;
        m_iMap2D->get_map_names(maps);
        m_currentMapName = maps[0];
    }

    if (nullptr == m_iMap2D)
    {
        yCError(MAP2D_NWS_ROS2, "Subdevice passed to attach method is invalid");
        return false;
    }

    //updateRvizMarkers on startup
    if (1)
    {
        updateVizMarkers();
    }

    return true;
}

bool Map2D_nws_ros2::detach()
{
    m_iMap2D = nullptr;
    return true;
}

bool Map2D_nws_ros2::open(yarp::os::Searchable &config)
{
    parseParams(config);

    if(m_name[0] == '/'){
        yCError(MAP2D_NWS_ROS2) << "Nws name parameter cannot begin with an initial /";
        return false;
    }

    m_rpcPortName = "/"+m_name+"/rpc";

    //open rpc port
    if (!m_rpcPort.open(m_rpcPortName))
    {
        yCError(MAP2D_NWS_ROS2, "Failed to open port %s", m_rpcPortName.c_str());
        return false;
    }
    m_rpcPort.setReader(*this);

    //ROS2 configuration
    if(m_node_name[0] == '/'){
        yCError(MAP2D_NWS_ROS2) << "node_name cannot begin with an initial /";
        return false;
    }
    if(m_namespace.empty()) {
        m_node = NodeCreator::createNode(m_node_name);
    } else {
        m_node = NodeCreator::createNode(m_node_name, m_namespace);
    }
    if (m_node == nullptr) {
        yCError(MAP2D_NWS_ROS2) << " opening " << m_node_name << " Node, check your yarp-ROS2 network configuration\n";
        return false;
    }
    rmw_qos_profile_t qos_rmw;
    qos_rmw.history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    qos_rmw.depth=10;
    rmw_time_t time;
    time.sec=10000;
    time.nsec = 0;
    qos_rmw.deadline= time;
    qos_rmw.lifespan=time;
    qos_rmw.liveliness_lease_duration=time;
    qos_rmw.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    qos_rmw.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    qos_rmw.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    qos_rmw.avoid_ros_namespace_conventions = true;
    m_ros2Service_getMap = m_node->create_service<nav_msgs::srv::GetMap>(m_getmap,
                                                                                       std::bind(&Map2D_nws_ros2::getMapCallback,this,_1,_2,_3),
#if RCLCPP_VERSION_GTE(17,0,0)
                                                                                       rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_rmw))
#else
                                                                                       qos_rmw
#endif
                                                                                       );
    m_ros2Service_getMapByName = m_node->create_service<map2d_nws_ros2_msgs::srv::GetMapByName>(m_getmapbyname,
                                                                                                              std::bind(&Map2D_nws_ros2::getMapByNameCallback,this,_1,_2,_3));
    m_ros2Service_rosCmdParser = m_node->create_service<test_msgs::srv::BasicTypes>(m_roscmdparser,
                                                                                                  std::bind(&Map2D_nws_ros2::rosCmdParserCallback,this,_1,_2,_3));

    yCInfo(MAP2D_NWS_ROS2) << "Waiting for device to attach";
    start();

    return true;
}


void Map2D_nws_ros2::run()
{

//    if(!m_spinned)  //This is just a temporary solution.
//    {
//        rclcpp::spin(m_node);
//        m_spinned = true;
//    }
}

bool Map2D_nws_ros2::close()
{
    yCTrace(MAP2D_NWS_ROS2, "Close");
    m_rpcPort.close();
    rclcpp::shutdown();
    return true;
}

bool Map2D_nws_ros2::read(yarp::os::ConnectionReader& connection)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    yarp::os::Bottle in;
    yarp::os::Bottle out;
    bool ok = in.read(connection);
    if (!ok) return false;

    //parse string command
    if(in.get(0).isString())
    {
         std::string ss = in.get(0).asString();
         if (ss == "help")
         {
             yInfo("updateMarkers");
             yInfo("publishMap <name>");
             out.addString("updateMarkers");
             out.addString("publishMap");
         }
         else if (ss == "updateMarkers")
         {
             updateVizMarkers();
             out.addString("ok");
         }
         else if (ss == "publishMap")
         {
             std::string mapname = in.get(1).asString();
             publishMap(mapname);
             out.addString("ok");
         }
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        out.write(*returnToSender);
    }
    else
    {
        yCError(MAP2D_NWS_ROS2) << "Invalid return to sender";
    }
    return true;
}

bool Map2D_nws_ros2::updateVizMarkers()
{
    yCInfo(MAP2D_NWS_ROS2) << "Updating rviz markers. Current map is: " << m_currentMapName;
    if (!m_ros2Publisher_markers)
    {
        m_ros2Publisher_markers = m_node->create_publisher<visualization_msgs::msg::MarkerArray>(m_markers_pub, 10);
    }
    builtin_interfaces::msg::Duration dur;
    dur.sec = 0xFFFFFFFF;
    double yarpTimeStamp = yarp::os::Time::now();
    uint64_t time;
    uint64_t sec_part;
    rclcpp::Time ret;
    time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    sec_part = (time / 1000000000UL);
    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yCWarning(MAP2D_NWS_ROS2) << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }
    visualization_msgs::msg::Marker marker;
    builtin_interfaces::msg::Time    tt;
    yarp::sig::Vector         rpy(3);
    yarp::math::Quaternion    q;
    visualization_msgs::msg::MarkerArray markers;

    std::vector<std::string> locations;
    std::vector<std::string> objects;
    int count = 1;
    m_iMap2D->getLocationsList(locations);
    m_iMap2D->getObjectsList(objects);
    for (auto it : locations)
    {
        yarp::dev::Nav2D::Map2DLocation loc;
        m_iMap2D->getLocation(it, loc);

        if(loc.map_id != m_currentMapName && m_currentMapName != "none")
        {
            continue;
        }

        rpy[0] = 0; //x
        rpy[1] = 0; //y
        rpy[2] = loc.theta / 180 * M_PI; //z
        yarp::sig::Matrix m = yarp::math::rpy2dcm(rpy);
        q.fromRotationMatrix(m);

        marker.header.frame_id    = "map";
        tt.sec                    = (yarp::os::NetUint32) sec_part;
        marker.header.stamp       = tt;
        marker.ns                 = m_markers_pub+"_ns";
        marker.id                 = count;
        marker.type               = visualization_msgs::msg::Marker::ARROW;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = loc.x;
        marker.pose.position.y    = loc.y;
        marker.pose.position.z    = 0;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x            = 1;
        marker.scale.y            = 0.1;
        marker.scale.z            = 0.1;
        marker.color.a            = 1.0;
        marker.color.r            = 0.0;
        marker.color.g            = 1.0;
        marker.color.b            = 0.0;
        marker.lifetime           = dur;
        marker.text               = it;
        markers.markers.push_back(marker);
        count++;
    }
    for (auto it : objects)
    {
        yarp::dev::Nav2D::Map2DObject obj;
        m_iMap2D->getObject(it, obj);

        if(obj.map_id != m_currentMapName && m_currentMapName != "none")
        {
            continue;
        }

        rpy[0] = obj.roll / 180.0 * M_PI; //x
        rpy[1] = obj.pitch / 180.0 * M_PI; //y
        rpy[2] = obj.yaw / 180.0 * M_PI; //z
        yarp::sig::Matrix m = yarp::math::rpy2dcm(rpy);
        q.fromRotationMatrix(m);

        marker.header.frame_id    = "map";
        tt.sec                    = (yarp::os::NetUint32) sec_part;
        marker.header.stamp       = tt;
        marker.ns                 = m_markers_pub+"_ns";
        marker.id                 = count;
        marker.type               = visualization_msgs::msg::Marker::ARROW;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = obj.x;
        marker.pose.position.y    = obj.y;
        marker.pose.position.z    = obj.z;
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x            = 1;
        marker.scale.y            = 0.1;
        marker.scale.z            = 0.1;
        marker.color.a            = 1.0;
        marker.color.r            = 0.0;
        marker.color.g            = 0.0;
        marker.color.b            = 1.0;
        marker.lifetime           = dur;
        marker.text               = it;
        markers.markers.push_back(marker);
        count++;
    }

    m_ros2Publisher_markers->publish(markers);
    return true;
}


void Map2D_nws_ros2::getMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                    const std::shared_ptr<nav_msgs::srv::GetMap::Request> request,
                                    std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
{
    yCWarning(MAP2D_NWS_ROS2) << "Not yet implemented";
    nav_msgs::msg::OccupancyGrid mapToGo;
    mapToGo.header.frame_id = "map";

    response->map = mapToGo;
}

void Map2D_nws_ros2::rosCmdParserCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<test_msgs::srv::BasicTypes::Request> request,
                                     std::shared_ptr<test_msgs::srv::BasicTypes::Response> response)
{
    if(request->string_value == "updatemarkerviz")
    {
        if(updateVizMarkers()) response->set__string_value("done");
        else response->set__string_value("error");
    }
    else
    {
        response->set__string_value("unknown_command");
    }
}

nav_msgs::msg::OccupancyGrid Map2D_nws_ros2::publishMap(std::string mapname)
{
    if (!m_ros2Publisher_map)
    {
        m_ros2Publisher_map = m_node->create_publisher<nav_msgs::msg::OccupancyGrid>(m_getmapbyname+"/pub", 10);
    }
    nav_msgs::msg::OccupancyGrid mapToGo;
    nav_msgs::msg::MapMetaData metaToGo;
    mapToGo.header.frame_id = "map";
    MapGrid2D theMap;
    if(!m_iMap2D->get_map(mapname,theMap))
    {
        mapToGo.header.frame_id = "invalid_frame";
        return mapToGo;
    }
    mapToGo.info.map_load_time = m_node->get_clock()->now();
    mapToGo.header.stamp = m_node->get_clock()->now();
    mapToGo.info.height = theMap.height();
    mapToGo.info.width = theMap.width();

    double DEG2RAD = M_PI/180.0;
    double tmp=0;
    theMap.getResolution(tmp);
    mapToGo.info.resolution=tmp;
    double x, y, t;
    theMap.getOrigin(x,y,t);
    mapToGo.info.origin.position.x=x;
    mapToGo.info.origin.position.y=y;
    yarp::math::Quaternion q;
    yarp::sig::Vector v(4);
    v[0]=0; v[1]=0; v[2]=1; v[3]=t*DEG2RAD;
    q.fromAxisAngle(v);
    mapToGo.info.origin.orientation.x = q.x();
    mapToGo.info.origin.orientation.y = q.y();
    mapToGo.info.origin.orientation.z = q.z();
    mapToGo.info.origin.orientation.w = q.w();
    mapToGo.data.resize(theMap.width()*theMap.height());
    int index=0;
    yarp::dev::Nav2D::XYCell cell;
    for (cell.y=theMap.height(); cell.y-- > 0;)
    {
      for (cell.x=0; cell.x<theMap.width(); cell.x++)
      {
        theMap.getOccupancyData(cell,tmp);
        mapToGo.data[index++]=(int)tmp;
      }
    }

    metaToGo.map_load_time = mapToGo.info.map_load_time;
    metaToGo.height = theMap.height();
    metaToGo.width = theMap.width();
    metaToGo.origin = mapToGo.info.origin;
    metaToGo.resolution = mapToGo.info.resolution;

    if (m_ros2Publisher_map)
    {
        m_ros2Publisher_map->publish(mapToGo);
    }

    return mapToGo;
}

//void Map2D_nws_ros2::prepareMapMsg(MapGrid2D inputMap, nav_msgs::msg::OccupancyGrid &outputMsg)
void Map2D_nws_ros2::getMapByNameCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                          const std::shared_ptr<map2d_nws_ros2_msgs::srv::GetMapByName::Request> request,
                                          std::shared_ptr<map2d_nws_ros2_msgs::srv::GetMapByName::Response> response)
{
    m_currentMapName = request->name;
    auto rmap = publishMap(request->name);
    response->map = rmap;
}
