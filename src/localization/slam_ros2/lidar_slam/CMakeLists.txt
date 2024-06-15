cmake_minimum_required(VERSION 3.5)
project(lidar_slam)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sps_common_msgs REQUIRED)

find_package(GRAPHICSMAGICKCPP REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(cartographer_ros REQUIRED)
find_package(gflags REQUIRED)
find_package(GLOG QUIET)
find_package(TBB REQUIRED)
find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Lua REQUIRED)


set(absl_LIBRARIES
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility
  )


set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  sps_common_msgs
  yaml_cpp_vendor
  cartographer_ros
  gflags
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${CERES_INCLUDES}
  ${GRAPHICSMAGICKCPP_INCLUDE_DIRS}
  # ${LUA_INCLUDE_DIR}
)

set(sps_slam
#   src/GingerCartographerMapping.cpp
   src/SpsCartographerLoc.cpp
 )

add_executable(lidar_slam_node src/main.cpp src/slam_node.cpp ${sps_slam})
ament_target_dependencies(lidar_slam_node ${dependencies})
target_link_libraries(lidar_slam_node  
  # ginger_slam 
  ${absl_LIBRARIES} 
  ${GLOG_LIBRARIES} ${GRAPHICSMAGICKCPP_LIBRARIES})

install(TARGETS lidar_slam_node 
  DESTINATION 
  lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(include)
# ament_export_libraries(ginger_slam)
ament_export_dependencies(${dependencies})
ament_package()
