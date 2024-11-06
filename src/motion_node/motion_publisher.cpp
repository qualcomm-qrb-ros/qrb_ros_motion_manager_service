// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_node/motion_publisher.hpp"

namespace qrb_ros
{
namespace motion_service
{

MotionPublisher::MotionPublisher(std::shared_ptr<rclcpp::Node> node_handler,
                                 std::shared_ptr<MotionServiceProxy> service_proxy)
{
  RCLCPP_INFO(logger_, "create");
  node_handler_ = node_handler;
  motion_service_proxy_ = service_proxy;

  create_motion_velocity_publisher();
  create_motion_status_publisher();
}

MotionPublisher::~MotionPublisher()
{
  RCLCPP_INFO(logger_, "destroy");
  reset_motion_velocity();
  reset_motion_status();

}

// ====================== twist publisher ===========================
void MotionPublisher::create_motion_velocity_publisher()
{
  RCLCPP_INFO(logger_, "create_motion_velocity_publisher");
  if (node_handler_ != nullptr) {
    twist_publisher_ = node_handler_->create_publisher<Twist>("cmd_vel", 10);
    publish_twist_cb_ = [&](Twist & twist) { publish_motion_velocity(twist); };
    if (motion_service_proxy_ != nullptr) {
      motion_service_proxy_->register_motion_velocity_callback(publish_twist_cb_);
    }
  }
}

void MotionPublisher::publish_motion_velocity(Twist & twist)
{
  if (twist_publisher_ != nullptr) {
    RCLCPP_INFO(logger_, "publish_motion_velocity(%.2f, %.2f, %.2f)",
      twist.linear.x, twist.linear.y, twist.angular.z);
    twist_publisher_->publish(twist);
  }
}

// ====================== motion status publisher ===========================
void MotionPublisher::create_motion_status_publisher()
{
  RCLCPP_INFO(logger_, "create_motion_status_publisher");
  if (node_handler_ != nullptr) {
    motion_status_publisher_ = node_handler_->create_publisher<MotionStatus>("motion_status", 10);
    publish_motion_status_cb_ = [&](MotionStatus & motion_status) {
      publish_motion_status(motion_status);
    };
    if (motion_service_proxy_ != nullptr) {
      motion_service_proxy_->register_motion_status_callback(publish_motion_status_cb_);
    }
  }
}

void MotionPublisher::publish_motion_status(MotionStatus & motion_status)
{
  if (motion_status_publisher_ != nullptr) {
    RCLCPP_INFO(logger_, "publish_motion_status on_motion:" + motion_status.on_motion);
    motion_status_publisher_->publish(motion_status);
  }
}

void MotionPublisher::reset_motion_velocity()
{
  RCLCPP_INFO(logger_, "reset_motion_velocity");
  Twist twist;
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;
  if (twist_publisher_ != nullptr) {
    twist_publisher_->publish(twist);
  }
}

void MotionPublisher::reset_motion_status()
{
  RCLCPP_INFO(logger_, "reset_motion_status");
  MotionStatus status;
  status.on_motion = false;
  if (motion_status_publisher_ != nullptr) {
    motion_status_publisher_->publish(status);
  }
}

}  // namespace motion_service
}  // namespace qrb_ros
