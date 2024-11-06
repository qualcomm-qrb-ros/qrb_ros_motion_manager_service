// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_node/odom_subscriber.hpp"

using namespace std::placeholders;

namespace qrb_ros
{
namespace motion_service
{

OdomSubscriber::OdomSubscriber(std::shared_ptr<rclcpp::Node> node_handler,
                               std::shared_ptr<MotionServiceProxy> service_proxy)
{
  RCLCPP_INFO(logger_, "create");
  node_handler_ = node_handler;
  motion_service_proxy_ = service_proxy;
  create_odom_subscriber();
}

OdomSubscriber::~OdomSubscriber()
{
  RCLCPP_INFO(logger_, "destroy");
}

void OdomSubscriber::create_odom_subscriber()
{
  RCLCPP_INFO(logger_, "create_odom_subscriber");
  if (node_handler_ != nullptr) {
    auto callback_group =
        node_handler_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = callback_group;

    subscriber_ = node_handler_->create_subscription<nav_msgs::msg::Odometry>("odom",
        rclcpp::SystemDefaultsQoS(), std::bind(&OdomSubscriber::odom_subscriber_callback, this, _1),
        subscription_options);

    RCLCPP_INFO(logger_, "Create subscriber: topic=\"%s\"", subscriber_->get_topic_name());
  }
}

void OdomSubscriber::odom_subscriber_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // RCLCPP_INFO(logger_, "odom_subscriber_callback");
  geometry_msgs::msg::Pose pose = msg->pose.pose;
  if (motion_service_proxy_ != nullptr) {
    motion_service_proxy_->update_pose(pose);
  }
}

}  // namespace motion_service
}  // namespace qrb_ros
