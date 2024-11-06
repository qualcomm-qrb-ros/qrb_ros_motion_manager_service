// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_node/path_publisher.hpp"

namespace qrb_ros
{
namespace motion_service
{

PathPublisher::PathPublisher(std::shared_ptr<rclcpp::Node> node_handler,
                             std::shared_ptr<MotionServiceProxy> service_proxy)
{
  RCLCPP_INFO(logger_, "create");
  node_handler_ = node_handler;
  motion_service_proxy_ = service_proxy;

  create_planning_path_publisher();
  create_tracing_path_publisher();
}

PathPublisher::~PathPublisher()
{
  RCLCPP_INFO(logger_, "destroy");
}

void PathPublisher::create_planning_path_publisher()
{
  RCLCPP_INFO(logger_, "create_planning_path_publisher");
  if (node_handler_ != nullptr) {
    planning_path_publisher_ =
        node_handler_->create_publisher<nav_msgs::msg::Path>("planning_path", 10);
    RCLCPP_INFO(
        logger_, "Create publisher: topic=\"%s\"", planning_path_publisher_->get_topic_name());

    planning_cb_ = [&](nav_msgs::msg::Path & path) { publish_planning_path(path); };
    if (motion_service_proxy_ != nullptr) {
      motion_service_proxy_->register_planning_path_callback(planning_cb_);
    }
  }
}

void PathPublisher::create_tracing_path_publisher()
{
  RCLCPP_INFO(logger_, "create_tracing_path_publisher");
  if (node_handler_ != nullptr) {
    tracing_path_publisher_ =
        node_handler_->create_publisher<nav_msgs::msg::Path>("tracing_path", 10);
    RCLCPP_INFO(
        logger_, "Create publisher: topic=\"%s\"", tracing_path_publisher_->get_topic_name());

    tracing_cb_ = [&](nav_msgs::msg::Path & path) { publish_tracing_path(path); };
    if (motion_service_proxy_ != nullptr) {
      motion_service_proxy_->register_tracing_path_callback(tracing_cb_);
    }
  }
}

void PathPublisher::publish_planning_path(nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(logger_, "publish_planning_path");
  if (planning_path_publisher_ != nullptr) {
    planning_path_publisher_->publish(path);
  }
}

void PathPublisher::publish_tracing_path(nav_msgs::msg::Path & path)
{
  RCLCPP_INFO(logger_, "publish_tracing_path");
  if (tracing_path_publisher_ != nullptr) {
    tracing_path_publisher_->publish(path);
  }
}

}  // namespace motion_service
}  // namespace qrb_ros
