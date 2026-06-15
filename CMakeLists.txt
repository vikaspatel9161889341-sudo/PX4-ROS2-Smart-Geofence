cmake_minimum_required(VERSION 3.8)
project(nav_failsafe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# 🎯 Task 4: Velocity Bridge Node Executable
add_executable(velocity_bridge_node src/velocity_bridge_node.cpp)
ament_target_dependencies(velocity_bridge_node rclcpp geometry_msgs px4_msgs)

# 🎯 Task 3: Nav2 Custom Behavior Tree Shared Library Plugin
add_library(battery_emergency_condition_plugin SHARED src/battery_emergency_condition.cpp)
target_compile_definitions(battery_emergency_condition_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(battery_emergency_condition_plugin 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav2_behavior_tree 
  behaviortree_cpp_v3
)

# Install targets
install(TARGETS
  velocity_bridge_node
  battery_emergency_condition_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
