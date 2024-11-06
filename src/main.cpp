// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_motion_service/motion_controller.hpp"

int32_t main(int32_t argc, char ** argv)
{
  rclcpp::init(argc, argv);
  qrb_ros::motion_service::MotionController control;
  (control.executor_)->spin();
  rclcpp::shutdown();
  return 0;
}
