cmake_minimum_required(VERSION 3.8)
project(origincar_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(smooth_path_follower src/smooth_path_follower.cpp)
ament_target_dependencies(smooth_path_follower rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros)
target_link_libraries(smooth_path_follower yaml-cpp)

add_executable(multi_point_nav src/multi_point_nav.cpp)
ament_target_dependencies(multi_point_nav action_msgs geometry_msgs nav2_msgs rclcpp rclcpp_action std_msgs tf2 tf2_ros)
target_link_libraries(multi_point_nav yaml-cpp)

add_executable(initial_pose_publisher src/initial_pose_publisher.cpp)
ament_target_dependencies(initial_pose_publisher rclcpp geometry_msgs)

install(TARGETS
  smooth_path_follower
  multi_point_nav
  initial_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_navigation.sh
  scripts/start_qr_navigation.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
