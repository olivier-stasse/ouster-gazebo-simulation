cmake_minimum_required(VERSION 3.5)
SET(PROJECT_NAMESPACE gepetto)
SET(PROJECT_NAME ouster_gazebo_plugins)
SET(PROJECT_DESCRIPTION "Ouster OS1 64 plugins package")
SET(PROJECT_URL "https://gepgitlab.laas.fr/gepetto/ouster-gazebo-simulations")

set(PROJECT_USE_CMAKE_EXPORT TRUE)

# JRL-cmakemodule setup
include("${CMAKE_SOURCE_DIR}/../cmake/base.cmake")

SET(CMAKE_BUILD_TYPE Release)


# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# catkin_package(
#   INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
#   LIBRARIES gazebo_ros_ouster_laser gazebo_ros_ouster_gpu_laser
#   CATKIN_DEPENDS roscpp sensor_msgs gazebo_ros
# )

include_directories(
  include
  ${RCLCPP_INCLUDE_DIRS}
  ${SENSOR_MSGS_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(gazebo_ros_ouster_laser src/GazeboRosOusterLaser.cpp)
target_link_libraries(gazebo_ros_ouster_laser
  ${GAZEBO_LIBRARIES}
  RayPlugin
  )
ament_target_dependencies(gazebo_ros_ouster_laser rclcpp sensor_msgs gazebo_ros)

add_library(gazebo_ros_ouster_gpu_laser src/GazeboRosOusterLaser.cpp)
target_link_libraries(gazebo_ros_ouster_gpu_laser
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)
target_compile_definitions(gazebo_ros_ouster_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)
ament_target_dependencies(gazebo_ros_ouster_gpu_laser rclcpp sensor_msgs gazebo_ros)

install(TARGETS gazebo_ros_ouster_laser gazebo_ros_ouster_gpu_laser
  EXPORT ${TARGETS_EXPORT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)
