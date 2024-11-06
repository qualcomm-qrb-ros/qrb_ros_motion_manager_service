// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__MOTION_CONTROLLER_HPP_
#define QRB_ROS_MOTION_SERVICE__MOTION_CONTROLLER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"
#include "qrb_ros_motion_service/motion_node/status_monitor.hpp"
#include "qrb_ros_motion_service/motion_node/odom_subscriber.hpp"
#include "qrb_ros_motion_service/motion_node/tf_subscriber.hpp"
#include "qrb_ros_motion_service/motion_node/path_publisher.hpp"
#include "qrb_ros_motion_service/motion_node/motion_publisher.hpp"
// #include "qrb_ros_motion_service/motion_node/simulation_sub_pub.hpp"

#include "qrb_ros_motion_service/motion_node/arc_motion_action.hpp"
#include "qrb_ros_motion_service/motion_node/circle_motion_action.hpp"
#include "qrb_ros_motion_service/motion_node/line_motion_action.hpp"
#include "qrb_ros_motion_service/motion_node/rectangle_motion_action.hpp"
#include "qrb_ros_motion_service/motion_node/rotation_motion_action.hpp"

namespace qrb_ros
{
namespace motion_service
{

class MotionController
{
public:
  MotionController();

  ~MotionController();

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

private:
  void init();

  std::shared_ptr<rclcpp::Node> node_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  std::shared_ptr<TFSubscriber> tf_subscriber_{ nullptr };
  std::shared_ptr<OdomSubscriber> odom_subscriber_{ nullptr };
  std::shared_ptr<StatusMonitorSubscriber> status_monitor_subscriber_{ nullptr };
  std::shared_ptr<PathPublisher> path_publisher_{ nullptr };
  std::shared_ptr<MotionPublisher> motion_publisher_{ nullptr };
  // std::shared_ptr<SimulationSubPub> simulation_sub_pub_{ nullptr };

  std::shared_ptr<ArcMotionAction> arc_motion_action_{ nullptr };
  std::shared_ptr<CircleMotionAction> circle_motion_action_{ nullptr };
  std::shared_ptr<LineMotionAction> line_motion_action_{ nullptr };
  std::shared_ptr<RectangleMotionAction> rectangle_motion_action_{ nullptr };
  std::shared_ptr<RotationMotionAction> rotation_motion_action_{ nullptr };

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::motion_controller") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__MOTION_CONTROLLER_HPP_