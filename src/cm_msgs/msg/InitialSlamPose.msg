uint8 need_optimize
geometry_msgs/PoseWithCovarianceStamped initial_pose


uint8 NO_OPTIMIZE  = 0 # 不需要被优化
uint8 OPTIMIZE  = 1 # 在SLAM中会将initial_pose作为初始位姿，再次进行优化
