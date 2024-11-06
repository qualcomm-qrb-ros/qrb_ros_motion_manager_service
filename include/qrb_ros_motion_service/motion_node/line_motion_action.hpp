// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__LINE_MOTION_ACTION_HPP_
#define QRB_ROS_MOTION_SERVICE__LINE_MOTION_ACTION_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "qrb_ros_motion_msgs/action/line_motion.hpp"
#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class LineMotionAction
{

using LineMotion = qrb_ros_motion_msgs::action::LineMotion;
using GoalHandleLineMotion = rclcpp_action::ServerGoalHandle<LineMotion>;

public:
  LineMotionAction(std::shared_ptr<rclcpp::Node> node_handler,
                   std::shared_ptr<MotionServiceProxy> service_proxy);

  ~LineMotionAction();

private:
  void create_motion_action_server();

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const LineMotion::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLineMotion> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleLineMotion> goal_handle);

  void receive_command_callback(int request_id, bool result);

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  std::shared_ptr<rclcpp_action::Server<LineMotion>> servers_ptr_{ nullptr };

  std::map<rclcpp_action::GoalUUID, int> request_id_map_;
  std::map<int, std::shared_ptr<GoalHandleLineMotion>> handle_map_;

  command_callback_func_t command_callback_;

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::line_motion_action") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__LINE_MOTION_ACTION_HPP_