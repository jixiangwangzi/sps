std_msgs/Header header
uint8 localization_state
string map_uuid
string map_name
float32 confidence
float32 threshold
bool map_update
int32 grid_x
int32 grid_y
float32 grid_theta
float32 world_x
float32 world_y
float32 world_theta

uint8 NOT_INITIALIZED = 0
uint8 INITIALIZING = 1
uint8 RUNNING = 2
uint8 WEAK = 3
uint8 LOST = 4
uint8 BLOCKED = 5
uint8 OUT_MAP = 6