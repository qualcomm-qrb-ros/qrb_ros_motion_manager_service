cmake_minimum_required(VERSION 3.8)
project(qrb_ros_motion_service)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories (./include)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp)
ament_auto_find_build_dependencies()

ament_auto_add_executable(motion_service
  src/main.cpp
  src/motion_controller.cpp
  src/motion_node/status_monitor.cpp
  src/motion_node/odom_subscriber.cpp
  src/motion_node/tf_subscriber.cpp
  src/motion_node/path_publisher.cpp
  src/motion_node/motion_publisher.cpp
  # src/motion_node/simulation_sub_pub.cpp
  src/motion_node/arc_motion_action.cpp
  src/motion_node/circle_motion_action.cpp
  src/motion_node/line_motion_action.cpp
  src/motion_node/rotation_motion_action.cpp
  src/motion_node/rectangle_motion_action.cpp
  src/motion_service/motion_service_proxy.cpp
)

target_link_libraries(motion_service
  yaml-cpp
  qrb_motion_manager
)

ament_auto_package()