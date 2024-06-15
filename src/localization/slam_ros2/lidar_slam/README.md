# Ginger Navigation

## Descripton

包括地图和导航接口模块。地图模块用于建图时发布地图以及图像坐标，导航模块提供对外的导航接口。

### Version

### V2.1.0

- add virtual wall layer

### V2.0.0

- switch amcl and slam dynamically

### V1.1.1

- optimize pose reporting, report pose when move 5cm, turn 0.1rad, or 1s
- when SaveMap called, just stop update map, don't shutdown SlamGmapping

#### V1.1.0

- integrate SlamGMapping

#### V1.0.1

- /bit_map data字段编码为png, 大幅减小建图时的数据传输
- mapping_tools可以发布目标点

#### V1.0.0

- 实现接口: /StartMapping, /SaveMap, /NaviTo, /NaviCmd, /bit_map, /pixel_pose
- 接口调试OK
