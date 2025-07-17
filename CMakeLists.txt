cmake_minimum_required(VERSION 3.8)
project(lidar_fusion)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_merger_node src/lidar_merger_node.cpp)
ament_target_dependencies(lidar_merger_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
  ament_index_cpp
)

target_link_libraries(lidar_merger_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  lidar_merger_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python scripts
install(PROGRAMS
  scripts/rate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()