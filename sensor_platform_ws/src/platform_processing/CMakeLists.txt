cmake_minimum_required(VERSION 3.8)
project(platform_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




#-----------------------


add_library(imu_sync_component SHARED
  src/imu_sync_component.cpp
)

target_include_directories(imu_sync_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# link the dependencies
ament_target_dependencies(imu_sync_component
  rclcpp
  sensor_msgs
  message_filters
  rclcpp_components
)

ament_target_dependencies(imu_sync_component
  tf2
  tf2_geometry_msgs
)

# Register the components in order to be visible by ROS 2
rclcpp_components_register_node(imu_sync_component
  PLUGIN "platform_processing::ImuSyncComponent"
  EXECUTABLE imu_sync_node
)

# Installation (for "ros2 run" find it)
install(TARGETS imu_sync_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS scripts/mock_imu_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
