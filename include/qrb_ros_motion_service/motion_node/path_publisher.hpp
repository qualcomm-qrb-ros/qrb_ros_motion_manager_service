// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__PATH_PUBLISHER_HPP_
#define QRB_ROS_MOTION_SERVICE__PATH_PUBLISHER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class PathPublisher
{

public:
  PathPublisher(std::shared_ptr<rclcpp::Node> node_handler,
                std::shared_ptr<MotionServiceProxy> service_proxy);

  ~PathPublisher();

private:
  void create_planning_path_publisher();

  void create_tracing_path_publisher();

  void publish_planning_path(nav_msgs::msg::Path & path);

  void publish_tracing_path(nav_msgs::msg::Path & path);

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planning_path_publisher_{ nullptr };
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tracing_path_publisher_{ nullptr };

  planning_path_callback_func_t planning_cb_;
  tracing_path_callback_func_t tracing_cb_;

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::path_publisher") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__PATH_PUBLISHER_HPP_