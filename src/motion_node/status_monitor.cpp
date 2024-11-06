// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_node/status_monitor.hpp"

#define LOW_POWER_LEVEL 20

using namespace std::placeholders;

namespace qrb_ros
{
namespace motion_service
{

StatusMonitorSubscriber::StatusMonitorSubscriber(std::shared_ptr<rclcpp::Node> node_handler,
                                                 std::shared_ptr<MotionServiceProxy> service_proxy)
{
  RCLCPP_INFO(logger_, "create");
  node_handler_ = node_handler;
  motion_service_proxy_ = service_proxy;
  create_status_subscriber();
}

StatusMonitorSubscriber::~StatusMonitorSubscriber()
{
  RCLCPP_INFO(logger_, "destroy");
}

void StatusMonitorSubscriber::create_status_subscriber()
{
  RCLCPP_INFO(logger_, "create_status_subscriber");
  if (node_handler_ != nullptr) {
    auto charing_sub = node_handler_->create_subscription<qrb_ros_robot_base_msgs::msg::ChargerCmd>(
        "charger_cmd", rclcpp::SystemDefaultsQoS(),
        std::bind(&StatusMonitorSubscriber::charging_subscriber_callback, this, _1));
    sub_ptr_.push_back(charing_sub);
    RCLCPP_INFO(logger_, "Create subscriber: topic=\"%s\"", charing_sub->get_topic_name());

    auto battery_sub = node_handler_->create_subscription<sensor_msgs::msg::BatteryState>(
        "battery", rclcpp::SystemDefaultsQoS(),
        std::bind(&StatusMonitorSubscriber::battery_subscriber_callback, this, _1));
    sub_ptr_.push_back(battery_sub);
    RCLCPP_INFO(logger_, "Create subscriber: topic=\"%s\"", battery_sub->get_topic_name());

    auto amr_status_sub = node_handler_->create_subscription<qrb_ros_amr_msgs::msg::AMRStatus>(
        "amr_status", rclcpp::SystemDefaultsQoS(),
        std::bind(&StatusMonitorSubscriber::amr_status_subscriber_callback, this, _1));
    sub_ptr_.push_back(amr_status_sub);
    RCLCPP_INFO(logger_, "Create subscriber: topic=\"%s\"", amr_status_sub->get_topic_name());
  }
}

void StatusMonitorSubscriber::charging_subscriber_callback(
    const qrb_ros_robot_base_msgs::msg::ChargerCmd::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "charging_subscriber_callback");
  bool charging_status;

  if (msg->cmd == qrb_ros_robot_base_msgs::msg::ChargerCmd::START_CHARGING) {
    charging_status = true;
  } else if (msg->cmd == qrb_ros_robot_base_msgs::msg::ChargerCmd::STOP_CHARGING) {
    charging_status = false;
  } else {
    RCLCPP_ERROR(logger_, "chargerCmd msg cmd error");
    return;
  }

  if (motion_service_proxy_ != nullptr) {
    motion_service_proxy_->update_charging_status(charging_status);
  }
}

void StatusMonitorSubscriber::battery_subscriber_callback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "battery_subscriber_callback");
  bool low_power_status;
  auto voltage = msg->voltage;
  if (voltage <= LOW_POWER_LEVEL) {
    low_power_status = true;
  } else {
    low_power_status = false;
  }

  if (motion_service_proxy_ != nullptr) {
    motion_service_proxy_->update_low_power_status(low_power_status);
  }
}

void StatusMonitorSubscriber::amr_status_subscriber_callback(
  const qrb_ros_amr_msgs::msg::AMRStatus::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "amr_status_subscriber_callback");
  int status_id = msg->status_change_id;
  // TODO
  if (status_id != 5) {
    return;
  }

  bool amr_status = false;
  int state = msg->current_state;
  if (state == ON_AE || state == ON_FOLLOW_PATH || state == FOLLOW_PATH_WAIT || state == ON_P2PNAV
    || state == ON_ME || state == ME_DONE) {
    amr_status = true;
  } else {
    amr_status = false;
  }

  if (motion_service_proxy_ != nullptr) {
    motion_service_proxy_->update_navigation_status(amr_status);
  }
}

}  // namespace motion_service
}  // namespace qrb_ros
