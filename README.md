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

## ArUco任务板演示

这部分用于课程作业中的“OpenCV目标识别、跟踪、SLAM建图、自主导航”稳定演示，但不再使用 YOLO 或训练模型，而是改成基于 ArUco 任务板的视觉标识方案。

### 为什么采用 ArUco 任务板

- 普通目标检测通常依赖数据集、标注和训练流程，调参成本高。
- 本课程重点是 ROS、OpenCV、SLAM、Navigation 的系统集成，ArUco 更适合做稳定、可解释、可重复的视觉触发器。
- OpenCV 原生支持 ArUco 检测，能直接展示角点检测、ID 读取和姿态估计。
- 任务板 ID 与 YAML 任务表映射后，可以把“看见 marker”直接转换成“执行哪一个任务”的语义决策。

### 系统流程

1. 启动 Gazebo 和 TurtleBot3 Waffle Pi。
2. 机器人先读取任务板 ArUco ID。
3. 程序根据 `config/aruco_tasks.yaml` 解析出目标房间、导航点和目标 ArUco ID。
4. 如果导航已接好，机器人通过 `move_base` 去目标房间。
5. 到达后原地旋转，用摄像头搜索目标 ArUco ID。
6. 找到后停止机器人，并发布任务完成结果。

### 运行顺序

```bash
# 1) 生成 ArUco 模型资产
rosrun cleanbot_course_project generate_aruco_assets.py

# 2) 启动 Gazebo + Waffle Pi
roslaunch cleanbot_course_project tb3_waffle_pi_house.launch

# 3) 生成任务板和目标板
roslaunch cleanbot_course_project spawn_aruco_boards.launch task_id:=101 randomize:=false

# 4) 启动 ArUco 检测
roslaunch cleanbot_course_project aruco_task_detector.launch

# 5) 查看检测图像
rqt_image_view
# 选择 /cleanbot/aruco/debug_image

# 6) 查看检测结果
rostopic echo /cleanbot/aruco/detections

# 7) 启动状态机
rosrun cleanbot_course_project aruco_task_state_machine.py
```

如果导航暂时还没接好，可以先把状态机参数 `use_navigation` 设为 `false`，这样它只会做“读取任务板 -> 解析任务 -> 搜索目标板”的闭环测试，不会卡在 move_base 初始化。

### 配置文件

- [`src/cleanbot_course_project/config/aruco_tasks.yaml`](src/cleanbot_course_project/config/aruco_tasks.yaml) 定义任务 ID、目标对象、房间和导航点。
- [`src/cleanbot_course_project/config/aruco_detector.yaml`](src/cleanbot_course_project/config/aruco_detector.yaml) 定义摄像头、发布话题、搜索速度和超时。

### 重要说明

任务板编号使用 101/102/103/104，目标板编号使用 201/202/203/204。这个编号范围超出了 `DICT_4X4_50` 的容量，所以实现里使用的是 `DICT_4X4_250`。如果字典不够大，ArUco marker ID 根本生成不出来，这也是之前最容易踩的坑。

### 可写进报告的说明

1. 为什么采用 ArUco 任务板：
   - 普通目标检测对数据集和模型依赖较高。
   - 本课程重点是 ROS、OpenCV、SLAM、Navigation 的系统集成。
   - ArUco 提供稳定、可解释、可重复的目标识别方式。
   - OpenCV 原生支持 ArUco 检测，便于展示角点检测和姿态估计思想。

2. 视觉模块：
   - 订阅摄像头图像。
   - OpenCV 灰度化和 ArUco 角点检测。
   - 输出 marker ID、角点坐标、中心点、检测图像。
   - 通过 marker ID 与任务表映射实现语义识别。

3. Gazebo 模块：
   - 参考 UCAR 的 Gazebo 工程组织方式。
   - 使用 `empty_world.launch` 加载 world。
   - 使用 `xacro -> robot_description -> spawn_model` 生成机器人。
   - 使用 `/gazebo/spawn_sdf_model` 动态生成任务板和目标板。
   - 任务板支持随机生成，提高演示灵活性。

4. 导航模块：
   - SLAM 建图阶段使用 gmapping。
   - 导航阶段使用 `map_server + AMCL + move_base`。
   - 状态机根据任务板解析结果向 `move_base` 发送目标点。

5. 系统创新点：
   - 重点强调模块化、稳定性、可复现实验流程。
   - 不要写成深度学习目标检测系统。
   - 可以写成“基于视觉标识的任务驱动导航系统”。

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
