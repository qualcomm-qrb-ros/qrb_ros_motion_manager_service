// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__MOTION_PUBLISHER_HPP_
#define QRB_ROS_MOTION_SERVICE__MOTION_PUBLISHER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "qrb_ros_motion_msgs/msg/motion_status.hpp"
#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class MotionPublisher
{

using Twist = geometry_msgs::msg::Twist;
using MotionStatus = qrb_ros_motion_msgs::msg::MotionStatus;

public:
  MotionPublisher(std::shared_ptr<rclcpp::Node> node_handler,
                  std::shared_ptr<MotionServiceProxy> service_proxy);

  ~MotionPublisher();

private:
  void create_motion_status_publisher();

  void create_motion_velocity_publisher();

  void publish_motion_velocity(Twist& twist);

  void publish_motion_status(MotionStatus& motion_status);

  void reset_motion_velocity();

  void reset_motion_status();

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  std::shared_ptr<rclcpp::Publisher<Twist>> twist_publisher_{ nullptr };
  std::shared_ptr<rclcpp::Publisher<MotionStatus>> motion_status_publisher_{ nullptr };

  publish_motion_status_func_t publish_motion_status_cb_;
  publish_twist_func_t publish_twist_cb_;

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::motion_publisher") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__MOTION_PUBLISHER_HPP_