std_msgs/Header   header
string   device_name              # option: alarm module name
string[] device_list              # Alarm module list, SCA[1~4] / ECU / Camera etc.
uint32   alarm_code               # Alarm Code
uint32[] alarm_type               # Alarm Type
uint8[]  alarm_severity           # Severity
uint32[] alarm_spec               # SpecificProblem
string[] additional_info          # additional Info

# Alarm Code Predefine
uint32 ALM_COLLISION      = 31   # Collision
uint32 ALM_WHEEL          = 39   # Wheel exception
uint32 ALM_CAMERA_OFFLINE = 40   # Camera offline
uint32 ALM_SCA_INIT_LOST  = 41   # SCA initial failed or lost
uint32 ALM_SCA_OVER_TEMP  = 46   # SCA over temperature
uint32 ALM_BATTERY_LOW    = 48   # Battery is low
uint32 ALM_CHARGE_FAILED  = 69   # Charge failed
uint32 ALM_ECU_OFFLINE    = 87   # ECU offline
uint32 ALM_MAP_NOT_EXIST  = 300  # Map or poi not exist
uint32 ALM_NAVI_LOST      = 302  # Navi lost
uint32 ALM_NAVI_PATH_FAIL = 303  # Navi point path failed，导航刚开始规划不出路径
uint32 ALM_EMERGENCY_STOP = 307  # Emergency stop
uint32 ALM_NAVI_PATH_INVALID = 308  # 导航过程中，由于丢失，围堵等原因，出现规划路径失败，且超过2分钟
uint32 ALM_NAVI_TRAPPED      = 309  # 导航过程中，停留时间过长，15秒内移动距离小于50厘米
uint32 ALM_NAVI_DROP_DANGER  = 310  # 检测到附近有跌落风险


uint32 ALM_NAVI_LIDAR_ABNORMAL= 42  # 雷达故障
uint32 ALM_NAVI_ODOM_ABNORMAL = 95  # 轮速计故障
uint32 ALM_NAVI_IMU_ABNORMAL = 38   # 惯性测量单元故障
uint32 ALM_NAVI_LOC_WEAK = 313      # 定位质量弱
uint32 ALM_NAVI_LOC_INT_LOST = 315    # 定位初始化异常

uint32 ALM_HDMAP_LOAD_VISON_MAP_FAIL = 314
uint32 ALM_HDMAP_NO_VISON_MAP = 316
uint32 ALM_HDMAP_VLOC_ABNORMAL = 317







# Alarm Type Predefine
uint8  ALM_COMMUNICATIONS   = 2  # A. Communications: 2
uint8  ALM_QUALITY          = 3  # B. Quality of Service (QoS): 3
uint8  ALM_PROCESSING_ERROR = 4  # C. Processing Error: 4
uint8  ALM_EQUIPMENT        = 5  # D. Equipment: 5
uint8  ALM_ENVIRONMENTAL    = 6  # E. Environmental: 6

# Alarm Severity Predefine
uint8  ALM_CRITICAL = 1          # A. Critical: 1
uint8  ALM_MAJOR    = 2          # B. Major: 2
uint8  ALM_MINOR    = 3          # C. Minor: 3
uint8  ALM_WARNING  = 4          # D. Warning: 4
uint8  ALM_CLEARED  = 5          # E. Cleared: 5
