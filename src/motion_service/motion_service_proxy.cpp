// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

MotionServiceProxy::MotionServiceProxy()
{
  RCLCPP_INFO(logger_, "create");
  motion_service_ = MotionService::get_instance();

  if (motion_service_ == nullptr) {
    RCLCPP_ERROR(logger_, "MotionService create failed");
  }

  register_callback();
}

MotionServiceProxy::~MotionServiceProxy()
{
  RCLCPP_INFO(logger_, "destroy");
}

void MotionServiceProxy::register_callback()
{
  RCLCPP_INFO(logger_, "register_callback");
  velocity_callback_func_t velocity_cb = [&](motion_velocity velocity) {
    notify_motion_velocity(velocity);
  };
  if (motion_service_ != nullptr) {
    RCLCPP_INFO(logger_, "register_velocity_callback to motion service");
    motion_service_->register_velocity_callback(velocity_cb);
  }

  motion_status_callback_func_t status_cb = [&](bool on_motion) {
    notify_motion_status(on_motion);
  };
  if (motion_service_ != nullptr) {
    RCLCPP_INFO(logger_, "register_motion_status_callback  to motion service");
    motion_service_->register_motion_status_callback(status_cb);
  }

  motion_command_callabck_func_t command_cb = [&](int request_id, MOTION_TYPE type,
                                                  COMMAND_STATUS status) {
    if (status == COMMAND_STATUS::END) {
      notify_command(request_id, type, true);
    } else if (status == COMMAND_STATUS::START_FAILED) {
      notify_command(request_id, type, false);
    }
    // COMMAND_STATUS::ON_START no need to return node action
  };
  if (motion_service_ != nullptr) {
    RCLCPP_INFO(logger_, "register_motion_command_callback  to motion service");
    motion_service_->register_motion_command_callback(command_cb);
  }

  path_callback_func_t path_cb = [&](std::vector<motion_pose> path, PATH_TYPE type) {
    notify_path(path, type);
  };
  if (motion_service_ != nullptr) {
    RCLCPP_INFO(logger_, "register_path_callback  to motion service");
    motion_service_->register_path_callback(path_cb);
  }
}

// =========================== request motion ==========================
int MotionServiceProxy::request_arc_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                                           double radius, double angle)
{
  RCLCPP_INFO(logger_, "request_arc_motion");
  if (motion_service_ != nullptr) {
    VELOCITY_LEVEL velocity_level = convert_velocity_level(velocity);
    return motion_service_->request_arc_motion(velocity_level, radius, angle);
  }
  return 0;
}

int MotionServiceProxy::request_circle_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                                              double radius, int laps, bool direction)
{
  RCLCPP_INFO(logger_, "request_circle_motion");
  if (motion_service_ != nullptr) {
    VELOCITY_LEVEL velocity_level = convert_velocity_level(velocity);
    return motion_service_->request_circle_motion(velocity_level, radius, laps, direction);
  }
  return 0;
}

int MotionServiceProxy::request_line_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                                            double distance)
{
  RCLCPP_INFO(logger_, "request_line_motion");
  if (motion_service_ != nullptr) {
    VELOCITY_LEVEL velocity_level = convert_velocity_level(velocity);
    return motion_service_->request_line_motion(velocity_level, distance);
  }
  return 0;
}

int MotionServiceProxy::request_rectangle_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                                                 double width, double height, bool direction)
{
  RCLCPP_INFO(logger_, "request_rectangle_motion");
  if (motion_service_ != nullptr) {
    VELOCITY_LEVEL velocity_level = convert_velocity_level(velocity);
    return motion_service_->request_rectangle_motion(velocity_level, width, height, direction);
  }
  return 0;
}

int MotionServiceProxy::request_rotation_motion(qrb_ros_motion_msgs::msg::VelocityLevel velocity,
                                                double angle)
{
  RCLCPP_INFO(logger_, "request_rotation_motion");
  if (motion_service_ != nullptr) {
    VELOCITY_LEVEL velocity_level = convert_velocity_level(velocity);
    return motion_service_->request_rotation_motion(velocity_level, angle);
  }
  return 0;
}

bool MotionServiceProxy::request_stop_motion(int request_id)
{
  RCLCPP_INFO(logger_, "request_stop_motion");
  if (motion_service_ != nullptr) {
    return motion_service_->request_stop_motion(request_id);
  }
  return false;
}

// =========================== update pose ==========================
void MotionServiceProxy::update_pose(geometry_msgs::msg::Pose pose)
{
  //RCLCPP_INFO(logger_, "update_pose");
  // convert PoseStamped to motion_pose
  motion_pose motion_pose;
  motion_pose.x = pose.position.x;
  motion_pose.y = pose.position.y;

  tf2::Quaternion q_orig;
  tf2::convert(pose.orientation, q_orig);
  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
  motion_pose.angle = yaw;

  // send to motion service
  if (motion_service_ != nullptr) {
    return motion_service_->update_pose(motion_pose);
  }
}

void MotionServiceProxy::update_pose(double x, double y, double angle)
{
  //RCLCPP_INFO(logger_, "update_pose");
  motion_pose motion_pose;
  motion_pose.x = x;
  motion_pose.y = y;
  motion_pose.angle = angle;

  if (motion_service_ != nullptr) {
    return motion_service_->update_pose(motion_pose);
  }
}

// ====================update state==========================
void MotionServiceProxy::update_navigation_status(bool status)
{
  RCLCPP_INFO(logger_, "update_navigation_status");
  if (motion_service_ != nullptr) {
    return motion_service_->update_navigation_status(status);
  }
}

void MotionServiceProxy::update_low_power_status(bool status)
{
  RCLCPP_INFO(logger_, "update_low_power_status");
  if (motion_service_ != nullptr) {
    return motion_service_->update_low_power_status(status);
  }
}

void MotionServiceProxy::update_charging_status(bool status)
{
  RCLCPP_INFO(logger_, "update_charging_status");
  if (motion_service_ != nullptr) {
    return motion_service_->update_charging_status(status);
  }
}

void MotionServiceProxy::update_diagnostic_status(bool status)
{
  RCLCPP_INFO(logger_, "update_diagnostic_status");
  if (motion_service_ != nullptr) {
    return motion_service_->update_diagnostic_status(status);
  }
}

// =========================== register callback ==========================
void MotionServiceProxy::register_motion_velocity_callback(publish_twist_func_t cb)
{
  RCLCPP_INFO(logger_, "register_motion_velocity_callback");
  twist_cb_ = cb;
}

void MotionServiceProxy::notify_motion_velocity(motion_velocity velocity)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = velocity.line_velocity;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = velocity.angle_velocity;

  if (twist_cb_ != nullptr) {
    twist_cb_(twist);
  }
}

void MotionServiceProxy::register_motion_status_callback(publish_motion_status_func_t cb)
{
  RCLCPP_INFO(logger_, "register_motion_status_callback");
  motion_status_cb_ = cb;
}

void MotionServiceProxy::notify_motion_status(bool on_motion)
{
  qrb_ros_motion_msgs::msg::MotionStatus motion_status;
  motion_status.on_motion = on_motion;

  auto time = rclcpp::Clock().now();
  std_msgs::msg::Header header;
  header.stamp.sec = time.seconds();
  header.stamp.nanosec = time.nanoseconds();
  motion_status.header = header;

  if (motion_status_cb_ != nullptr) {
    motion_status_cb_(motion_status);
  }
}

void MotionServiceProxy::register_command_callback(command_callback_func_t cb,
                                                   MOTION_COMMAND_TYPE type)
{
  RCLCPP_INFO(logger_, "register_command_callback");
  command_cb_[type] = cb;
}

void MotionServiceProxy::notify_command(int request_id, MOTION_TYPE type, bool result)
{
  RCLCPP_INFO(logger_, "notify_command");

  MOTION_COMMAND_TYPE commad_type;

  switch (type) {
    case MOTION_TYPE::ARC_MOTION:
      commad_type = MOTION_COMMAND_TYPE::ARC_MOTION_COMMND;
      break;
    case MOTION_TYPE::CIRCLE_MOTION:
      commad_type = MOTION_COMMAND_TYPE::CIRCLE_MOTION_COMMND;
      break;
    case MOTION_TYPE::LINE_MOTION:
      commad_type = MOTION_COMMAND_TYPE::LINE_MOTION_COMMND;
      break;
    case MOTION_TYPE::RECTANGLE_MOTION:
      commad_type = MOTION_COMMAND_TYPE::RECTANGLE_MOTION_COMMND;
      break;
    case MOTION_TYPE::ROTATION_MOTION:
      commad_type = MOTION_COMMAND_TYPE::ROTATION_MOTION_COMMND;
      break;
    default:
      return;
  }

  if (command_cb_.count(commad_type) > 0) {
    auto cb = command_cb_[commad_type];
    cb(request_id, result);
  }
}

//   // ==================== debug - path publish =====================
void MotionServiceProxy::register_planning_path_callback(planning_path_callback_func_t cb)
{
  RCLCPP_INFO(logger_, "register_planning_path_callback");
  planning_path_cb_ = cb;
}

void MotionServiceProxy::register_tracing_path_callback(tracing_path_callback_func_t cb)
{
  RCLCPP_INFO(logger_, "register_tracing_path_callback");
  tracing_path_cb_ = cb;
}

void MotionServiceProxy::notify_path(std::vector<motion_pose> motion_path, PATH_TYPE type)
{
  RCLCPP_INFO(logger_, "notify_path");
  if (planning_path_cb_ == nullptr && tracing_path_cb_ == nullptr) {
    return;
  }

  nav_msgs::msg::Path path;
  for (auto it : motion_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = it.x;
    pose.pose.position.y = it.y;
    path.poses.push_back(pose);
  }

  auto time = rclcpp::Clock().now();
  std_msgs::msg::Header header;
  header.frame_id = "/map";
  header.stamp.sec = time.seconds();
  header.stamp.nanosec = time.nanoseconds();
  path.header = header;

  if (type == PATH_TYPE::PLANNING_PATH && planning_path_cb_ != nullptr) {
    RCLCPP_INFO(logger_, "notify planning path");
    planning_path_cb_(path);
  } else if (type == PATH_TYPE::TRACING_PATH && tracing_path_cb_ != nullptr) {
    RCLCPP_INFO(logger_, "notify tracing path");
    tracing_path_cb_(path);
  }
}

VELOCITY_LEVEL MotionServiceProxy::convert_velocity_level(
    qrb_ros_motion_msgs::msg::VelocityLevel velocity)
{
  if (velocity.value == velocity.FAST_SPEED) {
    return VELOCITY_LEVEL::FAST_SPEED;
  } else if (velocity.value == velocity.MEDIUM_SPEED) {
    return VELOCITY_LEVEL::MEDIUM_SPEED;
  } else {
    return VELOCITY_LEVEL::SLOW_SPEED;
  }
}

}  // namespace motion_service
}  // namespace qrb_ros
