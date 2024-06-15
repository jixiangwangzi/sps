std_msgs/Header header
string task_id
uint8 navi_type
uint8 navi_state
string navi_map
string navi_path
uint8 navi_detailed_state
float32 linear_speed
float32 angular_speed
float32 distance_remaining
float32 estimated_time_remaining
float32 mileage

uint8 IDLE  = 0
uint8 ACTIVE  = 11
uint8 SUCCESS = 12
uint8 FAILED  = 13
uint8 CANCELED = 14
uint8 PAUSE = 15
uint8 EMERGENCY = 16
uint8 RUNNING = 17