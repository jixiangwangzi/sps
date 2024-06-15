cmake_minimum_required(VERSION 3.5)
project(monitor_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sps_common_msgs REQUIRED)
find_package(cm_msgs REQUIRED)

include_directories(
  include
)

set(executable_name monitor_manager)

add_executable(${executable_name}
  src/monitor_manager.cpp
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  nav_msgs
  sps_common_msgs
  cm_msgs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name})


install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
