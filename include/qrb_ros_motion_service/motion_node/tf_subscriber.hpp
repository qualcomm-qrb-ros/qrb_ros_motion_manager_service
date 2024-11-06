// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__TF_SUBSCRIBER_HPP_
#define QRB_ROS_MOTION_SERVICE__TF_SUBSCRIBER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class TFSubscriber
{
public:
  TFSubscriber(std::shared_ptr<rclcpp::Node> node_handler,
               std::shared_ptr<MotionServiceProxy> service_proxy);

  ~TFSubscriber();

private:
  void create_tf_subscriber();

  void convert_tf_to_pose();

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{ nullptr };
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  rclcpp::TimerBase::SharedPtr timer_{ nullptr };

  std::string target_frame_ = "map";
  std::string source_frame_ = "base_link";
  geometry_msgs::msg::PoseStamped source_pose_;
  std::mutex mtx_;

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::tf_subscriber") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__TF_SUBSCRIBER_NODE_HPP_