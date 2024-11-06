// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_MOTION_SERVICE__STATUS_MONITOR_HPP_
#define QRB_ROS_MOTION_SERVICE__STATUS_MONITOR_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "qrb_ros_robot_base_msgs/msg/charger_cmd.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "qrb_ros_amr_msgs/msg/amr_status.hpp"

#include "qrb_ros_motion_service/motion_service/motion_service_proxy.hpp"

namespace qrb_ros
{
namespace motion_service
{

class StatusMonitorSubscriber
{
public:
  StatusMonitorSubscriber(std::shared_ptr<rclcpp::Node> node_handler,
                          std::shared_ptr<MotionServiceProxy> service_proxy);

  ~StatusMonitorSubscriber();

  // TODO AMR status
  const static int IN_ACTIVE = 1;
  const static int IDLE = 2;
  const static int READY = 3;
  const static int ON_AE = 4;
  const static int ON_FOLLOW_PATH = 5;
  const static int FOLLOW_PATH_WAIT = 6;
  const static int ON_P2PNAV = 7;
  const static int P2PNAV_WAIT = 8;
  const static int ON_LOW_POWER = 9;
  const static int LOW_POWER_CHARGING = 10;
  const static int ON_ERROR = 11;
  const static int ON_LOW_POWER_ERROR = 12;
  const static int ON_ME = 13;
  const static int ME_DONE = 14;

private:
  void create_status_subscriber();

  // TODO -- exception subscriber

  void amr_status_subscriber_callback(const qrb_ros_amr_msgs::msg::AMRStatus::SharedPtr msg);

  void charging_subscriber_callback(const qrb_ros_robot_base_msgs::msg::ChargerCmd::SharedPtr msg);

  void battery_subscriber_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_handler_{ nullptr };
  std::shared_ptr<MotionServiceProxy> motion_service_proxy_{ nullptr };
  std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> sub_ptr_{ nullptr };

  rclcpp::Logger logger_{ rclcpp::get_logger("motion_service::status_monitor") };
};

}  // namespace motion_service
}  // namespace qrb_ros

#endif  // QRB_ROS_MOTION_SERVICE__STATUS_MONITOR_HPP_