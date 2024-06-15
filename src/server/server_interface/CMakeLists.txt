cmake_minimum_required(VERSION 3.5)
project(server_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sps_common_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(system_manager REQUIRED)
find_package(cm_msgs REQUIRED)


include_directories(
  include
)

set(executable_name server_interface)

file(GLOB_RECURSE service_srcs src/api_service/*.cpp)
file(GLOB_RECURSE publisher_srcs src/api_publisher/*.cpp)
file(GLOB_RECURSE subscribe_srcs src/api_subscriber/*.cpp)


add_executable(${executable_name}
  src/main.cpp
  src/server_interface_core.cpp
)

set(library_name ${executable_name}_lib)

add_library(${library_name} SHARED
  ${service_srcs}
  ${publisher_srcs}
  ${subscribe_srcs}
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  std_srvs
  nav_msgs
  tf2_ros
  sps_common_msgs
  behaviortree_cpp_v3
  system_manager
  cm_msgs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  libjsoncpp.so
  ${library_name}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
