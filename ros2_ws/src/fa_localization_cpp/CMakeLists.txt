cmake_minimum_required(VERSION 3.18)
project(fa_localization_cpp LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
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
find_package(nav_msgs REQUIRED)
# Same MPI-before-PCL/VTK ordering issue as fa_perception_cpp -- see that
# package's CMakeLists.txt for why.
find_package(MPI REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common registration segmentation filters sample_consensus)

set(EXTRA_PCL_CONVERSIONS_LIB "")
if(TARGET pcl_conversions::pcl_conversions)
  set(EXTRA_PCL_CONVERSIONS_LIB pcl_conversions::pcl_conversions)
endif()

add_executable(scan_matcher_node src/scan_matcher_node.cpp)
target_link_libraries(scan_matcher_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  nav_msgs::nav_msgs
  ${EXTRA_PCL_CONVERSIONS_LIB}
  ${PCL_LIBRARIES}
)
target_include_directories(scan_matcher_node PRIVATE ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})

install(TARGETS
  scan_matcher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
