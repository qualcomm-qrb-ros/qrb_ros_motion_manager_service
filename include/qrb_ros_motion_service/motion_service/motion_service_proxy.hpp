// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__MOTION_SERVICE_PROXY_HPP_
#define QRB_ROS_MOTION_SERVICE__MOTION_SERVICE_PROXY_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "qrb_ros_motion_msgs/msg/velocity_level.hpp"
#include "qrb_ros_motion_msgs/msg/motion_status.hpp"
#include "qrb_motion_manager/common/common.hpp"
#include "qrb_motion_manager/motion_service.hpp"

using namespace qrb::motion_manager;

namespace qrb_ros
{
namespace motion_service
{

using MotionService = qrb::motion_manager::MotionService;

enum class MOTION_COMMAND_TYPE
{
  ARC_MOTION_COMMND = 0,
  CIRCLE_MOTION_COMMND = 1,
  LINE_MOTION_COMMND = 2,
  RECTANGLE_MOTION_COMMND = 3,
  ROTATION_MOTION_COMMND = 4,
};

typedef std::function<void(geometry_msgs::msg::Twist & twist)> publish_twist_func_t;
typedef std::function<void(qrb_ros_motion_msgs::msg::MotionStatus & motion_status)> publish_motion_status_func_t;
typedef std::function<void(int request_id, bool result)> command_callback_func_t;
typedef std::function<void(nav_msgs::msg::Path & path)> planning_path_callback_func_t;
typedef std::function<void(nav_msgs::msg::Path & path)> tracing_path_callback_func_t;

class MotionServiceProxy
{
public:
  MotionServiceProxy();

  ~MotionServiceProxy();

  // ====================update pose===========================
  void update_pose(geometry_msgs::msg::Pose);

  void update_pose(double x, double y, double angle);

  // ====================update state==========================
  void update_navigation_status(bool status);

  void update_low_power_status(bool status);

  void update_charging_status(bool status);

  void update_diagnostic_status(bool status);

  // ====================request motion==========================
  int request_arc_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                         double radius, double angle);

  int request_circle_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                            double radius, int laps, bool direction);

  int request_line_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                          double distance);

  int request_rectangle_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                               double width, double height, bool direction);

  int request_rotation_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                              double angle);

  bool request_stop_motion(int request_id);

  // ==================== register callback =====================
  void register_motion_velocity_callback(publish_twist_func_t cb);

  void register_motion_status_callback(publish_motion_status_func_t cb);

  void register_command_callback(command_callback_func_t cb, MOTION_COMMAND_TYPE type);

  void register_planning_path_callback(planning_path_callback_func_t cb);

  void register_tracing_path_callback(tracing_path_callback_func_t cb);

private:
  void register_callback();

  VELOCITY_LEVEL convert_velocity_level(qrb_ros_motion_msgs::msg::VelocityLevel velocity);

  void notify_motion_velocity(motion_velocity velocity);

  void notify_motion_status(bool on_motion);

  void notify_command(int request_id, MOTION_TYPE type, bool result);

  void notify_path(std::vector<motion_pose> motion_path, PATH_TYPE type);

  std::shared_ptr<MotionService> motion_service_{ nullptr };

  publish_twist_func_t twist_cb_;
  publish_motion_status_func_t motion_status_cb_;
  std::map<MOTION_COMMAND_TYPE, command_callback_func_t> command_cb_;
  planning_path_callback_func_t planning_path_cb_;
  tracing_path_callback_func_t tracing_path_cb_;

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::motion_service_proxy") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__MOTION_SERVICE_PROXY_HPP_