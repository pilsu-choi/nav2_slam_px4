cmake_minimum_required(VERSION 3.8)
project(nav2_dstar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Create library
add_library(nav2_dstar_planner SHARED
  src/dstar_planner.cpp
  src/dstar_lite.cpp
)

# Dependencies
ament_target_dependencies(nav2_dstar_planner
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Export library
pluginlib_export_plugin_description_file(nav2_core dstar_planner_plugin.xml)

# Install
install(TARGETS nav2_dstar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES dstar_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dstar_planner
    test/test_dstar_planner.cpp
  )
  target_link_libraries(test_dstar_planner nav2_dstar_planner)
  ament_target_dependencies(test_dstar_planner
    rclcpp
    nav2_core
    nav2_costmap_2d
    nav2_util
  )
endif()

ament_package()
