cmake_minimum_required(VERSION 3.8)
project(qr_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(PkgConfig REQUIRED)

pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

add_executable(qr_detector_node src/qr_detector_node.cpp)

target_include_directories(qr_detector_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(qr_detector_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(qr_detector_node
  ${OpenCV_LIBS}
  PkgConfig::ZBAR
)

install(TARGETS qr_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()