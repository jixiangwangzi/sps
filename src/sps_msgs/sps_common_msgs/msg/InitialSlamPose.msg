uint8 scene_type
uint8 optimize_type
geometry_msgs/PoseWithCovarianceStamped[] initial_pose

#场景类型
uint8 UNKNOWN = 0                       #未知类型(一般为地图原点)
uint8 CHARGING_PILE = 1                 #充电桩
uint8 LIFT_INTERIOR= 2                  #电梯内点
uint8 START_UP_WITH_SAVED_POSE = 3      #开机启动(用上次保存的位姿)
uint8 SWITCH_MAP_WITH_SAVED_POSE = 4    #切换地图(用上次保存的位姿)
uint8 CLOUD_INSTRUCTION = 5             #云端指令

uint8 NO_OPTIMIZE  = 0 # 不需要被优化
uint8 OPTIMIZE_LIDAR  = 1 # 在SLAM中会将initial_pose作为初始位姿，再次进行优化
uint8 OPTIMIZE_CAMERA  = 2
uint8 OPTIMIZE_ALL = 3
