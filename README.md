# cleanbot_course_project

ROS Noetic 扫地机器人课程项目，基于 AWS RoboMaker Small House 场景。

## 系统架构

```
Gazebo 仿真 (小家场景 + 扫地机器人)
    |
    ├── gmapping (SLAM 建图)
    ├── move_base (代价地图导航 + DWA 局部规划)
    ├── room_coverage_cleaner.py (房间覆盖清扫)
    └── RViz (可视化)
```

## 快速启动

### 1. 编译

```bash
cd /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 2. 启动仿真 + 导航

```bash
bash start_robot.sh living_room
```

这会启动 Gazebo、gmapping、move_base、room_coverage_cleaner 和 RViz。

### 3. 发送清扫命令（另开终端）

```bash
source /opt/ros/noetic/setup.bash
source /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project/devel/setup.bash
rostopic pub /cleanbot/clean_room std_msgs/String "data: 'living_room'" --once
```

可用房间：`living_room`、`kitchen`、`bedroom`、`study`、`all`

## 当前已知问题

### 1. SLAM 建图不稳定（核心问题）

使用 `robot_vacuum_house_demo.launch`（gmapping 模式）时：

- **机器人静止时**：gmapping 能正常建图，RViz 中可以看到代价地图
- **机器人移动后**：gmapping 地图会迅速退化，RViz 变为全黄（未知区域）
- **原因**：AWS RoboMaker Small House 世界模型的墙壁碰撞几何不完整。激光雷达大部分射线穿透墙壁返回 `inf`，720 个采样中通常只有 9-25 个有效读数。机器人移动后 gmapping 无法维持扫描匹配，地图失效
- **诊断数据**：
  - `/scan` 话题：720 samples，仅 ~20 个有效读数，其余为 `inf`
  - `/map`：98304 cells，通常仅 ~2% 被探索（free: ~1600, occupied: ~150, unknown: ~96000）
  - TF 树完整：`map → odom → base_link → lidar_link`

### 2. 导航撞障碍物

由于 SLAM 地图不完整，move_base 的全局代价地图缺少障碍物信息：

- 机器人会直接撞上沙发、茶几等家具
- 撞击后机器人卡住，雷达被遮挡，进一步恶化地图质量
- 恢复行为（conservative_reset + rotate_recovery）清除代价地图后无法重建

### 3. 房间覆盖范围

`rooms.yaml` 中的房间多边形已根据实际房屋布局更新，但由于导航问题，机器人无法完成完整覆盖。

## 已尝试的优化

以下参数已调整但未能解决核心问题：

| 文件 | 修改内容 | 效果 |
|------|----------|------|
| `move_base.yaml` | 增大 inflation_radius、occdist_scale，添加恢复行为 | 无明显改善 |
| `costmap_common.yaml` | 增大 footprint_padding | 无明显改善 |
| `local_costmap.yaml` | inflation_radius 0.32→0.55 | 无明显改善 |
| `global_costmap.yaml` | inflation_radius 0.35→0.55 | 无明显改善 |
| `gmapping.yaml` | 扩大地图边界，降低 minimumScore | 建图略有改善但仍不稳定 |
| `robot_vacuum.xacro` | 尝试 270°/360° 雷达 FOV | 360° 完全失效（射线打到自身），270° 无改善 |
| `rooms.yaml` | 更新房间多边形覆盖完整房间 | 未验证（受导航问题限制） |

## 建议的后续方向

1. **使用静态地图方案**：先手动建图保存，然后用 `map_server + AMCL` 替代 gmapping（`static_room_cleaning_demo.launch` 已有此框架，但需要有效的地图文件）
2. **更换仿真世界**：使用碰撞几何更完整的世界模型（如 TurtleBot3 的默认世界）
3. **增加虚拟墙壁**：在 Gazebo world 文件中添加简单的 box 碰撞体作为墙壁，让雷达能检测到

## 主要文件

- `start_robot.sh` — 一键启动脚本
- `src/cleanbot_course_project/launch/robot_vacuum_house_demo.launch` — 主 launch（gmapping 模式）
- `src/cleanbot_course_project/launch/static_room_cleaning_demo.launch` — 静态地图 launch（AMCL 模式）
- `src/cleanbot_course_project/launch/slam_room_cleaning_demo.launch` — SLAM 清扫 launch
- `src/cleanbot_course_project/scripts/room_coverage_cleaner.py` — 房间覆盖清扫逻辑
- `src/cleanbot_course_project/config/rooms.yaml` — 房间范围配置
- `src/cleanbot_course_project/config/move_base.yaml` — 导航参数
- `src/cleanbot_course_project/config/gmapping.yaml` — SLAM 参数
- `src/cleanbot_course_project/config/navigation.rviz` — RViz 可视化配置
- `src/cleanbot_course_project/urdf/robot_vacuum.xacro` — 机器人模型（差速驱动 + 激光雷达）

## 依赖

```bash
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-move-base \
  ros-noetic-map-server \
  ros-noetic-amcl \
  ros-noetic-slam-gmapping \
  ros-noetic-dwa-local-planner \
  ros-noetic-navfn \
  ros-noetic-costmap-2d \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-plugins \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  python3-opencv \
  python3-yaml \
  python3-numpy
```
