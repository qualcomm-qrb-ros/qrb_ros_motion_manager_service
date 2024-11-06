// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_node/simulation_sub_pub.hpp"

using namespace std::placeholders;

namespace qrb_ros
{
namespace motion_service
{

SimulationSubPub::SimulationSubPub(std::shared_ptr<rclcpp::Node> node_handler,
                                   std::shared_ptr<MotionServiceProxy> service_proxy)
{
  RCLCPP_INFO(logger_, "create");
  node_handler_ = node_handler;
  motion_service_proxy_ = service_proxy;
  create_simulation_pose_subscriber();
  create_simulation_velocity_publisher();
}

SimulationSubPub::~SimulationSubPub()
{
  RCLCPP_INFO(logger_, "destroy");
}

void SimulationSubPub::create_simulation_pose_subscriber()
{
  RCLCPP_INFO(logger_, "create_simulation_pose_subscriber");
  if (node_handler_ != nullptr) {
    pose_subscription_ = node_handler_->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&SimulationSubPub::pose_subscriber_callback, this, _1));
  }
}

void SimulationSubPub::create_simulation_velocity_publisher()
{
  RCLCPP_INFO(logger_, "create_simulation_velocity_publisher");
  if (node_handler_ != nullptr) {
    twist_publisher_ = node_handler_->create_publisher<Twist>("/turtle1/cmd_vel", 10);
    publish_twist_cb_ = [&](Twist & twist) { publish_velocity(twist); };
    if (motion_service_proxy_ != nullptr) {
      motion_service_proxy_->register_motion_velocity_callback(publish_twist_cb_);
    }
  }
}

void SimulationSubPub::pose_subscriber_callback(const turtlesim::msg::Pose::SharedPtr msg)
{
  //RCLCPP_INFO(logger_, "pose_subscriber_callback");
  double x = msg->x - 5.544445f;
  double y = msg->y - 5.544445f;
  double angle = msg->theta;

  if (motion_service_proxy_ != nullptr) {
    motion_service_proxy_->update_pose(x, y, angle);
  }
}

void SimulationSubPub::publish_velocity(Twist & twist)
{
  RCLCPP_INFO(logger_, "publish_velocity");
  if (twist_publisher_ != nullptr) {
    twist_publisher_->publish(twist);
  }
}

}  // namespace motion_service
}  // namespace qrb_ros
