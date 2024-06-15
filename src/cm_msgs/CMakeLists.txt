cmake_minimum_required(VERSION 3.5)
project(cm_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/DockState.msg"
"msg/SpsUltrasonic.msg"
"msg/BatteryState.msg"
"msg/VirtualWall.msg"
"msg/Nav2State.msg"
"msg/PointArray.msg"
"msg/Virtualwalls.msg"
"msg/InitialSlamPose.msg"
"msg/SpsNaviState.msg"
"msg/LocalState.msg"
"msg/SpsSlamReport.msg"
"msg/WaitResumeFeedback.msg"
"msg/SpsHWError.msg"
"msg/SpsDropCollision.msg"
"msg/BitMap.msg"
"msg/SpsRdmAlarm.msg"
"srv/Nav2.srv"
"srv/NaviToPose.srv"
"srv/NaviPoses.srv"
"srv/ResumeFollowWaypoints.srv"
"srv/TaskNaviCmd.srv"
"srv/SpsLoadMap.srv"
"srv/SetPoseInitialize.srv"
"srv/AutoMove.srv"
"srv/SpsSetSlamModel.srv"
"srv/SpsParamsData.srv"
"srv/SpsNaviVel.srv"
"srv/GetPosesMileage.srv"
DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

install(
  FILES mappings.yaml parameter_bridge.yaml
  DESTINATION share/${PROJECT_NAME}
)
