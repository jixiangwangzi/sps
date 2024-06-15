int8 initial_state

int8 FAILED = -1  # 初始化失败
int8 NONE = 0  # 未设置地图，未下发初始位姿
int8 NOT_INITIALZED = 1  # 设置地图（正在设置或者设置完成），未下发位姿
int8 SET_INITIAL_POSE = 2  # 正在设置下发位姿（包括位姿正确性检测）
int8 BUILD_CONSTRAINT = 3   # 初始化位姿与地图进行匹配
int8 SUCCESS = 4  # 初始化成功