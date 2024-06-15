cmake_minimum_required(VERSION 3.8)
project(sensor_vel_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav2_util REQUIRED)
#find_package(nav2_lifecycle_manager REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  #nav2_lifecycle_manager
  nav2_util
)

set(sensor_filter_executable_name sensor_filter)
set(sensor_filter_library_name ${sensor_filter_executable_name}_core)

add_library(${sensor_filter_library_name} SHARED
  src/laser_filter.cpp
  src/sensor_filter.cpp
)

add_executable(${sensor_filter_executable_name}
  src/sensor_filter_main.cpp
)

ament_target_dependencies(${sensor_filter_library_name}
  ${dependencies}
)

target_link_libraries(${sensor_filter_executable_name}
  ${sensor_filter_library_name}
)

rclcpp_components_register_nodes(${sensor_filter_library_name} "sensor_vel_filters::SensorFilter")

set(pointcloud_filter_executable_name pointcloud_filter)
set(pointcloud_filter_library_name ${pointcloud_filter_executable_name}_core)

add_library(${pointcloud_filter_library_name} SHARED
  src/pointcloud_filter.cpp
)

add_executable(${pointcloud_filter_executable_name}
  src/pointcloud_filter_main.cpp
)

ament_target_dependencies(${pointcloud_filter_library_name}
  ${dependencies}
)

target_link_libraries(${pointcloud_filter_executable_name}
  ${pointcloud_filter_library_name}
)

rclcpp_components_register_nodes(${pointcloud_filter_library_name} "sensor_vel_filters::PointCloudFilter")

### install ###

install(TARGETS ${sensor_filter_library_name} ${pointcloud_filter_library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${sensor_filter_executable_name} ${pointcloud_filter_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
