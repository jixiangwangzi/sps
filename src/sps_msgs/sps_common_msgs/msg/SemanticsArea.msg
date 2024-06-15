std_msgs/Header header
string name
float64 slope
int32 type
float64 speed
float64 yaw
geometry_msgs/Point32[] points

int32 UNDEFINED_AREA           = -1  # 未定义区域
int32 GENERAL_SPEED_LIMIT_AREA =  0  # 一般限速区
int32 STEEP_SLOPE_AREA         =  1  # 陡坡区域
int32 CLOSE_SONAR_AREA         =  2  # 关闭超声区
int32 LONG_CORRIDOR_AREA       =  3  # 长走廊区
int32 NARROW_CHANNEL_AREA      =  4  # 窄通道区
int32 SPECIAL_INDUCTION_AREA   =  5  # 特殊感应去
