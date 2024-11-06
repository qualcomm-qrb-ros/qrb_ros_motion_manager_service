// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_controller.hpp"

namespace qrb_ros
{
namespace motion_service
{

MotionController::MotionController()
{
  RCLCPP_INFO(logger_, "create");
  init();
}

MotionController::~MotionController()
{
  RCLCPP_INFO(logger_, "destroy");
}

void MotionController::init()
{
  RCLCPP_INFO(logger_, "init");

  node_ = std::make_shared<rclcpp::Node>("motion_service");

  node_->declare_parameter<std::string>("pose_plugin", "odom");
  std::string pose_value;
  node_->get_parameter("pose_plugin", pose_value);
  RCLCPP_INFO(logger_, "Parameter pose_plugin value: %s", pose_value.c_str());

  motion_service_proxy_ = std::make_shared<MotionServiceProxy>();

  if (pose_value.compare("tf") == 0) {
    tf_subscriber_ =
      std::make_shared<TFSubscriber>(node_, motion_service_proxy_);
  } else {
    odom_subscriber_ = std::make_shared<OdomSubscriber>(node_, motion_service_proxy_);
  }

  path_publisher_ = std::make_shared<PathPublisher>(node_, motion_service_proxy_);

  status_monitor_subscriber_ = std::make_shared<StatusMonitorSubscriber>(node_, motion_service_proxy_);

  motion_publisher_ = std::make_shared<MotionPublisher>(node_, motion_service_proxy_);

  // simulation_sub_pub_ = std::make_shared<SimulationSubPub>(node_, motion_service_proxy_);

  arc_motion_action_ = std::make_shared<ArcMotionAction>(node_, motion_service_proxy_);

  circle_motion_action_ = std::make_shared<CircleMotionAction>(node_, motion_service_proxy_);

  line_motion_action_ = std::make_shared<LineMotionAction>(node_, motion_service_proxy_);

  rectangle_motion_action_ = std::make_shared<RectangleMotionAction>(node_, motion_service_proxy_);

  rotation_motion_action_ = std::make_shared<RotationMotionAction>(node_, motion_service_proxy_);

  executor_ = std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>(
      new rclcpp::executors::MultiThreadedExecutor());
  executor_->add_node(node_);
}

}  // namespace motion_service
}  // namespace qrb_ros
