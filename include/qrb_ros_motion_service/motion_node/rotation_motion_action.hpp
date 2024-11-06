// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__ROTATION_MOTION_ACTION_HPP_
#define QRB_ROS_MOTION_SERVICE__ROTATION_MOTION_ACTION_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "qrb_ros_motion_msgs/action/rotation_motion.hpp"
#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class RotationMotionAction
{

using RotationMotion = qrb_ros_motion_msgs::action::RotationMotion;
using GoalHandleRotationMotion = rclcpp_action::ServerGoalHandle<RotationMotion>;

public:
  RotationMotionAction(std::shared_ptr<rclcpp::Node> node_handler,
                       std::shared_ptr<MotionServiceProxy> service_proxy);

  ~RotationMotionAction();

private:
  void create_motion_action_server();

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const RotationMotion::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRotationMotion> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleRotationMotion> goal_handle);

  void receive_command_callback(int request_id, bool result);

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  std::shared_ptr<rclcpp_action::Server<RotationMotion>> servers_ptr_{ nullptr };

  std::map<rclcpp_action::GoalUUID, int> request_id_map_;
  std::map<int, std::shared_ptr<GoalHandleRotationMotion>> handle_map_;

  command_callback_func_t command_callback_;

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::rotation_motion_action") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__ROTATION_MOTION_ACTION_HPP_