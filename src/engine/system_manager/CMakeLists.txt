cmake_minimum_required(VERSION 3.5)
project(system_manager)

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
find_package(yaml_cpp_vendor REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cm_msgs REQUIRED)

include_directories(
  include
)


set(library_name ${PROJECT_NAME})


file(GLOB_RECURSE service_srcs src/service/*.cpp)
file(GLOB_RECURSE publisher_srcs src/publisher/*.cpp)
file(GLOB_RECURSE client_srcs src/service_client/*.cpp)
file(GLOB_RECURSE subscribe_srcs src/subscriber/*.cpp)

add_library(${library_name} SHARED
  src/robot_naviengine.cpp
  src/robot_bt.cpp
  src/simple_node.cpp
  src/sps_fsm.cpp
  src/naviengine_node.cpp
  src/bt_node_impl.cpp
  src/task_manager.cpp
  src/local_manager.cpp
  ${service_srcs}
  ${client_srcs}
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
  yaml_cpp_vendor
  nav2_msgs
  cm_msgs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)


ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)


install(DIRECTORY bt_resource
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
