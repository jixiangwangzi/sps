# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5) # Ships with Ubuntu 14.04 (Trusty)

project(cartographer_ros)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release)
endif()

set(PACKAGE_DEPENDENCIES
  cartographer_ros_msgs
  builtin_interfaces
  geometry_msgs
  nav_msgs
  pcl_conversions
  rosbag2_cpp
  rclcpp
#   roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  urdf
  visualization_msgs
  sps_common_msgs
)
message(STATUS "current platform: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "current nvidia platform: ${NVIDIA_BOARD_PLATFORM}")
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
  set(PLATFORM_MODE "glite_ccu")
  message(STATUS "This is glite ccu platform")
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(PLATFORM_MODE "x86_64")
  message(STATUS "This is x86_64 platform")
endif()


if(WIN32)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()

find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(cartographer_ros_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(absl REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sps_common_msgs REQUIRED)
# find_package(OpenCV REQUIRED)


message(STATUS " >>>>>>> PLATFORM_MODE is : ${PLATFORM_MODE}")
# For yet unknown reason, if Boost is find_packaged() after find_package(cartographer),
# some Boost libraries including Thread are set to static, despite Boost_USE_STATIC_LIBS,
# which causes linking problems on windows due to shared/static Thread clashing.
# PCL also finds Boost. Work around by moving before find_package(cartographer).
find_package(Boost REQUIRED COMPONENTS system iostreams)

find_package(cartographer REQUIRED
    # PATHS "/vendor/ginger_robot/install/share/lidar_slam/dependency_lib/${PLATFORM_MODE}/carto_lib/share/cartographer"
    # NO_DEFAULT_PATH
)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")

option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()
google_enable_testing()
set(CARTOGRAPHER_GMOCK_LIBRARIES ${GMOCK_LIBRARIES})



include(FindPkgConfig)

# file(GLOB absl_LIBRARIES
# RELATIVE ${PROJECT_SOURCE_DIR}/dependency_lib/${PLATFORM_MODE}/absl_lib/lib
# /vendor/ginger_robot/install/share/lidar_slam/dependency_lib/${PLATFORM_MODE}/absl_lib/lib/*.a
# /usr/local/lib/libabsl*.a
# )

find_package(LuaGoogle REQUIRED)

find_library(CAIRO_LIBRARIES cairo)

if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
    "."
    ${PCL_INCLUDE_DIRS}
    # ${Eigen3_DIRS}
    # ${cartographer_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
)

set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${CARTOGRAPHER_GMOCK_LIBRARIES})

#TODO:Ceres

# if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
#   if (${PLATFORM_MODE} STREQUAL "jetson_tx2")
#     unset(CERES_LIBRARIES)
#     set(CERES_LIBRARIES "${PROJECT_SOURCE_DIR}/dependency_lib/${PLATFORM_MODE}/ceres_lib/lib/libceres.a")
#   endif()
# endif()
# message("Ceres lib: ${CERES_LIBRARIES}")



file(GLOB_RECURSE ALL_SRCS "cartographer_ros/*.cc" "cartographer_ros/*.h")
file(GLOB_RECURSE ALL_TESTS "cartographer_ros/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer_ros/*_main.cc")
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer_ros/cartographer_grpc/*")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

list(REMOVE_ITEM ALL_SRCS "${CMAKE_SOURCE_DIR}/cartographer_ros/assets_writer.cc" "${CMAKE_SOURCE_DIR}/cartographer_ros/assets_writer.h")
list(REMOVE_ITEM ALL_SRCS "${CMAKE_SOURCE_DIR}/cartographer_ros/offline_node.cc" "${CMAKE_SOURCE_DIR}/cartographer_ros/offline_node.h")
list(REMOVE_ITEM ALL_SRCS "${CMAKE_SOURCE_DIR}/cartographer_ros/playable_bag.cc" "${CMAKE_SOURCE_DIR}/cartographer_ros/playable_bag.h")

if (NOT ${BUILD_GRPC})
list(REMOVE_ITEM ALL_SRCS ${ALL_GRPC_FILES})
list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()

message(STATUS "All src files:${ALL_SRCS}")

add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})

target_link_libraries(${PROJECT_NAME}  cartographer)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC #
    ${LUA_INCLUDE_DIR}
)

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

# Eigen
# target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
#   "${EIGEN3_INCLUDE_DIR}")

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME}  ${Boost_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(DIRECTORY launch urdf configuration_files
    DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/tf_remove_frames.py
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install source headers.
file(GLOB_RECURSE HDRS "cartographer_ros/*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      ${INSTALL_DIR}
  )
endforeach()

ament_export_include_directories(".")

ament_export_libraries(
    ${PROJECT_NAME} 
    cartographer 
    protobuf 
    ${LUA_LIBRARIES} 
    ${CAIRO_LIBRARIES} 
    ${CERES_LIBRARIES} 
    ${absl_LIBRARIES}
)

ament_export_dependencies(
    ${PACKAGE_DEPENDENCIES}
    PCL
    EIGEN3
    Boost
    absl
    # urdfdom_headers
    cartographer
)
ament_package()
