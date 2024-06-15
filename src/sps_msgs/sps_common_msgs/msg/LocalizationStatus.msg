std_msgs/Header header
geometry_msgs/Pose pose
uint8 status
float32 score
uint8 specific_causes
bool need_map_update

uint8 IDLE = 0
uint8 INITIALIZING = 1
uint8 RUNNING = 2
uint8 WEAK = 3
uint8 LOST = 4
uint8 BLOCKED = 5

uint8 NONE = 0
uint8 OUT_MAP = 1
uint8 LOC_JUMP = 2
uint8 LIDAR_VISUAL_LOC_WEAK = 3
uint8 LIDAR_LOC_WEAK = 4
uint8 VISUAL_LOC_WEAK = 5
uint8 LIDAR_DATA_LOST = 6
uint8 VISUAL_POSE_DATA_LOST = 7
uint8 ODOM_DATA_LOST = 8
uint8 IMU_DATA_LOST = 9
uint8 GPS_DATA_LOST = 10
uint8 BAROMETER_DATA_LOST = 11