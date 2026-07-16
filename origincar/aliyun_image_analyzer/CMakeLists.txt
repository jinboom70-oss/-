cmake_minimum_required(VERSION 3.8)
project(aliyun_image_analyzer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(image_analyzer src/image_analyzer_node.cpp)
target_compile_features(image_analyzer PRIVATE cxx_std_17)
ament_target_dependencies(image_analyzer rclcpp sensor_msgs std_msgs)
target_include_directories(image_analyzer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_analyzer CURL::libcurl ${OpenCV_LIBS})

install(TARGETS
  image_analyzer
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
