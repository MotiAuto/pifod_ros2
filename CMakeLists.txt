cmake_minimum_required(VERSION 3.5)
project(pifod_ros2)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(target_pifod_ros2 SHARED
  DIRECTORY src
)
include_directories(${EIGEN3_INCLUDE_DIR})
target_include_directories(target_pifod_ros2
  SYSTEM PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}
)

rclcpp_components_register_node(target_pifod_ros2
  PLUGIN "pifod_ros2::PIFOD_ROS2"
  EXECUTABLE pifod_ros2_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)