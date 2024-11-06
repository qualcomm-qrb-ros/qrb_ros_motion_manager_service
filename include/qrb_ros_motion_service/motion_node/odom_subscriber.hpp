// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__ODOM_SUBSCRIBER_HPP_
#define QRB_ROS_MOTION_SERVICE__ODOM_SUBSCRIBER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class OdomSubscriber
{
public:
  OdomSubscriber(std::shared_ptr<rclcpp::Node> node_handler,
                 std::shared_ptr<MotionServiceProxy> service_proxy);

  ~OdomSubscriber();

private:
  void create_odom_subscriber();

  void odom_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> subscriber_{ nullptr };

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::odom_subscriber") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__ODOM_SUBSCRIBER_HPP_