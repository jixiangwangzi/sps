std_msgs/Header header
uint8 state
uint32 current_waypoint_index
geometry_msgs/PoseStamped current_pose

uint8 UNDEFINED = 0
uint8 WAITING   = 1
uint8 RESUMING  = 2
uint8 RESUMED   = 3
