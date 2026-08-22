cmake_minimum_required(VERSION 3.8)
project(fa_perception_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common)

add_executable(point_cloud_source_node src/point_cloud_source_node.cpp)
target_link_libraries(point_cloud_source_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  pcl_conversions::pcl_conversions
  ${PCL_LIBRARIES}
)
target_include_directories(point_cloud_source_node PRIVATE ${PCL_INCLUDE_DIRS})

add_executable(point_cloud_processor_node src/point_cloud_processor_node.cpp)
target_link_libraries(point_cloud_processor_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  pcl_conversions::pcl_conversions
  ${PCL_LIBRARIES}
)
target_include_directories(point_cloud_processor_node PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS
  point_cloud_source_node
  point_cloud_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
