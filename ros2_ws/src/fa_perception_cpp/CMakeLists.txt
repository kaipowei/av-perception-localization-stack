cmake_minimum_required(VERSION 3.18)
project(fa_perception_cpp LANGUAGES C CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 17)
endif()
if(NOT CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES native)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# PCL's exported VTK config expects an MPI::MPI_C target to already exist;
# find it explicitly before anything that transitively pulls in PCL/VTK
# (pcl_conversions does, as a side effect of its own find_package(PCL) call).
find_package(MPI REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common)
find_package(CUDAToolkit REQUIRED)

# pcl_conversions is header-only and exports a real CMake target on some
# ROS2 distros (e.g. Lyrical) but not others (e.g. Jazzy, where find_package
# only adds its include dir) -- link it only where it actually exists so the
# same CMakeLists.txt works unmodified across both.
set(EXTRA_PCL_CONVERSIONS_LIB "")
if(TARGET pcl_conversions::pcl_conversions)
  set(EXTRA_PCL_CONVERSIONS_LIB pcl_conversions::pcl_conversions)
endif()

# GPU + CPU voxel-grid downsampler, kept dependency-free (no ROS/PCL types)
# so it can be unit-tested and benchmarked on its own.
add_library(voxel_downsample STATIC
  cuda/voxel_downsample.cu
  cuda/voxel_downsample_cpu.cpp
)
target_include_directories(voxel_downsample PUBLIC cuda)
set_target_properties(voxel_downsample PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)
target_link_libraries(voxel_downsample PUBLIC CUDA::cudart_static)

add_executable(benchmark_voxel cuda/benchmark_voxel.cpp)
target_link_libraries(benchmark_voxel voxel_downsample)

add_executable(point_cloud_source_node src/point_cloud_source_node.cpp)
target_link_libraries(point_cloud_source_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  ${EXTRA_PCL_CONVERSIONS_LIB}
  ${PCL_LIBRARIES}
)
target_include_directories(point_cloud_source_node PRIVATE ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})

add_executable(point_cloud_processor_node src/point_cloud_processor_node.cpp)
target_link_libraries(point_cloud_processor_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  ${EXTRA_PCL_CONVERSIONS_LIB}
  ${PCL_LIBRARIES}
  voxel_downsample
)
target_include_directories(point_cloud_processor_node PRIVATE ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})

install(TARGETS
  point_cloud_source_node
  point_cloud_processor_node
  benchmark_voxel
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
