// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_node/tf_subscriber.hpp"

namespace qrb_ros
{
namespace motion_service
{

TFSubscriber::TFSubscriber(std::shared_ptr<rclcpp::Node> node_handler,
                           std::shared_ptr<MotionServiceProxy> service_proxy)
{
  RCLCPP_INFO(logger_, "create");
  node_handler_ = node_handler;
  motion_service_proxy_ = service_proxy;
  create_tf_subscriber();
}

TFSubscriber::~TFSubscriber()
{
  RCLCPP_INFO(logger_, "destroy");
}

void TFSubscriber::create_tf_subscriber()
{
  RCLCPP_INFO(logger_, "init_tf_subscriber");
  if (node_handler_ != nullptr) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_handler_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf2::Quaternion q;
    q.setRPY(0.0f, 0.0f, 0.0f);  // yaw, pitch, roll

    // The center pose of the robot in the radar coordinate system.
    // if source_pose set header.stamp, the tf transform will error.
    source_pose_.pose.position.x = 0.0;
    source_pose_.pose.position.y = 0.0;
    source_pose_.pose.position.z = 0.0;
    source_pose_.pose.orientation.x = q.x();
    source_pose_.pose.orientation.y = q.y();
    source_pose_.pose.orientation.z = q.z();
    source_pose_.pose.orientation.w = q.w();
    source_pose_.header.frame_id = source_frame_;

    std::chrono::milliseconds duration(200);
    timer_ = node_handler_->create_wall_timer(
        duration, std::bind(&TFSubscriber::convert_tf_to_pose, this));
  }
}

void TFSubscriber::convert_tf_to_pose()
{
  RCLCPP_INFO(logger_, "convert_tf_to_pose");
  std::unique_lock<std::mutex> lck(mtx_);
  try {
    // get the transforstamped that pose change from radar coordinate to map coordinate.
    geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

    // transform the robot center pose from radar coordinate to map coordinate.
    geometry_msgs::msg::PoseStamped pose =
        tf_buffer_->transform(source_pose_, target_frame_, std::chrono::seconds(10));

    if (motion_service_proxy_ != nullptr) {
      motion_service_proxy_->update_pose(pose.pose);
    }

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Could not transform %s to %s: %s", source_frame_.c_str(),
        target_frame_.c_str(), ex.what());
    return;
  }
}

}  // namespace motion_service
}  // namespace qrb_ros
