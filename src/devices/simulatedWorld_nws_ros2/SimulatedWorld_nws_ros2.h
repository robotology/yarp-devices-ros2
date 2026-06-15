/*
 * SPDX-FileCopyrightText: 2026 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_DEV_SIMULATEDWORLD_NWS_ROS2_H
#define YARP_DEV_SIMULATEDWORLD_NWS_ROS2_H

#include <mutex>

#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/ISimulatedWorld.h>
#include <yarp/dev/DeviceDriver.h>

#include "SimulatedWorld_nws_ros2_ParamsParser.h"

#include <Ros2Spinner.h>

#include <rclcpp/rclcpp.hpp>

#include <simulated_world_nws_ros2_msgs/srv/make_sphere.hpp>
#include <simulated_world_nws_ros2_msgs/srv/make_box.hpp>
#include <simulated_world_nws_ros2_msgs/srv/make_cylinder.hpp>
#include <simulated_world_nws_ros2_msgs/srv/make_frame.hpp>
#include <simulated_world_nws_ros2_msgs/srv/make_model.hpp>
#include <simulated_world_nws_ros2_msgs/srv/change_color.hpp>
#include <simulated_world_nws_ros2_msgs/srv/set_pose.hpp>
#include <simulated_world_nws_ros2_msgs/srv/enable_gravity.hpp>
#include <simulated_world_nws_ros2_msgs/srv/enable_collision.hpp>
#include <simulated_world_nws_ros2_msgs/srv/get_pose.hpp>
#include <simulated_world_nws_ros2_msgs/srv/delete_object.hpp>
#include <simulated_world_nws_ros2_msgs/srv/delete_all.hpp>
#include <simulated_world_nws_ros2_msgs/srv/get_list.hpp>
#include <simulated_world_nws_ros2_msgs/srv/attach.hpp>
#include <simulated_world_nws_ros2_msgs/srv/detach.hpp>
#include <simulated_world_nws_ros2_msgs/srv/rename.hpp>

/**
 *  @ingroup dev_impl_nws_ros2
 *
 * \brief `SimulatedWorld_nws_ros2`: A network wrapper server that exposes the ISimulatedWorld interface via ROS2 services.
 *
 * Parameters required by this device are shown in class: SimulatedWorld_nws_ros2_ParamsParser
 */
class SimulatedWorld_nws_ros2 :
        public yarp::dev::DeviceDriver,
        public yarp::dev::WrapperSingle,
        public SimulatedWorld_nws_ros2_ParamsParser
{
public:
    SimulatedWorld_nws_ros2();
    SimulatedWorld_nws_ros2(const SimulatedWorld_nws_ros2&) = delete;
    SimulatedWorld_nws_ros2(SimulatedWorld_nws_ros2&&) noexcept = delete;
    SimulatedWorld_nws_ros2& operator=(const SimulatedWorld_nws_ros2&) = delete;
    SimulatedWorld_nws_ros2& operator=(SimulatedWorld_nws_ros2&&) noexcept = delete;
    ~SimulatedWorld_nws_ros2() override = default;

    // WrapperSingle
    bool attach(yarp::dev::PolyDriver* driver) override;
    bool detach() override;

    // DeviceDriver
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // ROS2 service callbacks
    void makeSphereCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeSphere::Request> request,
                            std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeSphere::Response> response);
    void makeBoxCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeBox::Request> request,
                         std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeBox::Response> response);
    void makeCylinderCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeCylinder::Request> request,
                              std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeCylinder::Response> response);
    void makeFrameCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeFrame::Request> request,
                           std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeFrame::Response> response);
    void makeModelCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeModel::Request> request,
                           std::shared_ptr<simulated_world_nws_ros2_msgs::srv::MakeModel::Response> response);
    void changeColorCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                             const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::ChangeColor::Request> request,
                             std::shared_ptr<simulated_world_nws_ros2_msgs::srv::ChangeColor::Response> response);
    void setPoseCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::SetPose::Request> request,
                         std::shared_ptr<simulated_world_nws_ros2_msgs::srv::SetPose::Response> response);
    void enableGravityCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableGravity::Request> request,
                               std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableGravity::Response> response);
    void enableCollisionCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableCollision::Request> request,
                                 std::shared_ptr<simulated_world_nws_ros2_msgs::srv::EnableCollision::Response> response);
    void getPoseCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetPose::Request> request,
                         std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetPose::Response> response);
    void deleteObjectCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteObject::Request> request,
                              std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteObject::Response> response);
    void deleteAllCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                           const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteAll::Request> request,
                           std::shared_ptr<simulated_world_nws_ros2_msgs::srv::DeleteAll::Response> response);
    void getListCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                         const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetList::Request> request,
                         std::shared_ptr<simulated_world_nws_ros2_msgs::srv::GetList::Response> response);
    void attachCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Attach::Request> request,
                        std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Attach::Response> response);
    void detachCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Detach::Request> request,
                        std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Detach::Response> response);
    void renameCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Rename::Request> request,
                        std::shared_ptr<simulated_world_nws_ros2_msgs::srv::Rename::Response> response);

private:
    yarp::dev::ISimulatedWorld* m_iSim{nullptr};
    rclcpp::Node::SharedPtr m_node;
    Ros2Spinner* m_spinner{nullptr};
    std::mutex m_mutex;

    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::MakeSphere>::SharedPtr m_srv_makeSphere{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::MakeBox>::SharedPtr m_srv_makeBox{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::MakeCylinder>::SharedPtr m_srv_makeCylinder{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::MakeFrame>::SharedPtr m_srv_makeFrame{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::MakeModel>::SharedPtr m_srv_makeModel{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::ChangeColor>::SharedPtr m_srv_changeColor{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::SetPose>::SharedPtr m_srv_setPose{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::EnableGravity>::SharedPtr m_srv_enableGravity{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::EnableCollision>::SharedPtr m_srv_enableCollision{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::GetPose>::SharedPtr m_srv_getPose{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::DeleteObject>::SharedPtr m_srv_deleteObject{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::DeleteAll>::SharedPtr m_srv_deleteAll{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::GetList>::SharedPtr m_srv_getList{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::Attach>::SharedPtr m_srv_attach{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::Detach>::SharedPtr m_srv_detach{nullptr};
    rclcpp::Service<simulated_world_nws_ros2_msgs::srv::Rename>::SharedPtr m_srv_rename{nullptr};
};

#endif // YARP_DEV_SIMULATEDWORLD_NWS_ROS2_H
