cmake_minimum_required(VERSION 3.16.3...3.20)
project(slam_gmapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(openslam_gmapping REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

add_executable(slam_gmapping src/slam_gmapping.cpp)
ament_target_dependencies(slam_gmapping
  rclcpp
  nav_msgs
  visualization_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  openslam_gmapping
)
target_link_libraries(slam_gmapping)



install(
  TARGETS slam_gmapping
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(slam_gmapping
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav_msgs
  visualization_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  openslam_gmapping
)

ament_package()
