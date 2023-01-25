cmake_minimum_required(VERSION 3.9.1)
SET(PROJECT_NAMESPACE gepetto)
SET(PROJECT_NAME ouster_os1_64)
SET(PROJECT_DESCRIPTION "Ouster OS1 64 package")
SET(PROJECT_URL "https://gepgitlab.laas.fr/gepetto/ouster-gazebo-simulations")

SET(CMAKE_VERBOSE_MAKEFILE ON)

# Project configuration
#set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CUSTOM_HEADER_DIR ${PROJECT_NAME})


# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_SOURCE_DIR}/cmake")
IF(NOT EXISTS "${CMAKE_SOURCE_DIR}/cmake/base.cmake")
  IF(${CMAKE_VERSION} VERSION_LESS "3.14.0")
    MESSAGE(FATAL_ERROR "\nPlease run the following command first:\ngit submodule update --init\n")
  ELSE()
    include(FetchContent)
    FetchContent_Declare("jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  ENDIF()
ENDIF()

# JRL-cmakemodule setup
include("${CMAKE_SOURCE_DIR}/../cmake/base.cmake")
#include("${JRL_CMAKE_MODULES}/boost.cmake")
#include("${JRL_CMAKE_MODULES}/python.cmake")

SET(CMAKE_BUILD_TYPE Release)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------
find_package(rclcpp)

foreach(dir launch meshes urdf worlds)
  install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
endforeach()

install(FILES package.xml DESTINATION share/${PROJECT_NAME} )

#ament_package()
