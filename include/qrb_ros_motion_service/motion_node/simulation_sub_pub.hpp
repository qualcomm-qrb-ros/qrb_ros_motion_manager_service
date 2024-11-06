// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__SIMULATION_SUB_PUB_HPP_
#define QRB_ROS_MOTION_SERVICE__SIMULATION_SUB_PUB_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <turtlesim/msg/pose.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class SimulationSubPub
{

using Twist = geometry_msgs::msg::Twist;

public:
  SimulationSubPub(std::shared_ptr<rclcpp::Node> node_handler,
                   std::shared_ptr<MotionServiceProxy> service_proxy);

  ~SimulationSubPub();

private:

  void create_simulation_pose_subscriber();

  void create_simulation_velocity_publisher();

  void pose_subscriber_callback(const turtlesim::msg::Pose::SharedPtr msg);

  void publish_velocity(Twist & twist);

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_{ nullptr };
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{ nullptr };

  publish_twist_func_t publish_twist_cb_;

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::simulation_sub_pub") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__SIMULATION_SUB_PUB_HPP_