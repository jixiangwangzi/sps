builtin_interfaces/Time stamp
string map_uuid
string map_name
string map_version
uint8 link_type
int32 result # only for uplink

uint8 DOWNLINK = 0 #下发
uint8 UPLINK = 1   #上报
int32 SUCCESS = 901
int32 FAILED = -901
int32 MAP_FILE_INVALID = -902 #地图文件错误
int32 MAP_NAME_INVALID = -903 #topic接收的地图uuid&name 和文件读取到的不匹配
int32 MAP_VERSION_INVALID = -904 #topic接收的地图版本和文件读取到的地图版本不匹配
