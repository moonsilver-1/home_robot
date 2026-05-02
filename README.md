# cleanbot_course_project

这是一个 ROS Noetic 扫地机器人课程项目。当前主流程是：

1. 启动 Gazebo 小家场景和扫地机器人。
2. 启动静态地图、AMCL 定位、`move_base` 代价地图导航。
3. 启动房间清扫节点。
4. 你发送房间名，机器人按该房间的覆盖路径去清扫。

当前可用房间名：

- `living_room`：客厅
- `kitchen`：厨房
- `bedroom`：卧室
- `study`：书房
- `all`：按配置顺序清扫所有房间

## 最推荐的用法

在 Windows PowerShell 里进入项目目录：

```powershell
cd D:\coding\inkwork-term2\cleanbot_course_project_noetic\cleanbot_course_project
```

第一次使用或改过代码后，先编译：

```powershell
wsl -d Ubuntu-20.04-Mirror -- bash -lc 'cd /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project && source /opt/ros/noetic/setup.bash && catkin_make'
```

启动完整系统：

```powershell
.\run_cleanbot_full_demo.ps1
```

等待 Gazebo 小家和机器人都出来后，另开一个 PowerShell 窗口，发送清扫指令：

```powershell
.\run_clean_room.ps1 living_room
```

换房间只改最后一个参数：

```powershell
.\run_clean_room.ps1 kitchen
.\run_clean_room.ps1 bedroom
.\run_clean_room.ps1 study
.\run_clean_room.ps1 all
```

注意：`run_clean_room.ps1` 只负责“发房间指令”，不会自动启动 Gazebo 和导航系统。必须先运行 `run_cleanbot_full_demo.ps1`。

## 一条命令自动启动并清扫

如果你想省事，可以让它启动后自动清扫一个房间：

```powershell
.\run_cleanbot_auto_clean.ps1 living_room
```

其他房间：

```powershell
.\run_cleanbot_auto_clean.ps1 kitchen
.\run_cleanbot_auto_clean.ps1 bedroom
.\run_cleanbot_auto_clean.ps1 study
.\run_cleanbot_auto_clean.ps1 all
```

## WSL 里怎么用

如果你已经进入了 Ubuntu WSL 终端，就不要再输入 `wsl -d ...`。`wsl` 是 Windows PowerShell 里的命令，不是 Ubuntu 里的命令。

WSL 终端里这样启动：

```bash
cd /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project
bash ./run_cleanbot_full_demo.sh
```

另开一个 WSL 终端发房间指令：

```bash
cd /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project
bash ./run_clean_room.sh living_room
```

自动启动并清扫：

```bash
cd /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project
bash ./run_cleanbot_auto_clean.sh living_room
```

## 当前导航方案

默认启动脚本现在使用：

```bash
roslaunch cleanbot_course_project static_room_cleaning_demo.launch gui:=true
```

这个 launch 会启动：

- AWS RoboMaker Small House 小家世界
- 扫地机器人 URDF 模型
- `map_server` 静态地图
- `amcl` 定位
- `move_base` 全局/局部代价地图
- `room_coverage_cleaner.py` 房间覆盖清扫节点

清扫节点会优先把房间路径点交给 `move_base`。如果 `move_base` 在仿真里因为代价地图恢复行为卡住，节点会自动切到 `/odom + /cmd_vel` 的兜底点到点控制，保证机器人继续执行清扫动作，而不是只发布一个字符串后原地不动。

## 我这边已经跑过的测试

编译通过：

```bash
catkin_make
```

客厅 headless 验收脚本通过，测试命令：

```bash
bash ./test_room_cleaning_headless.sh living_room 75 false
```

关键日志结果：

```text
planned:living_room:4
goal_sent:living_room_row_00_0
goal_reached:living_room_row_00_0
goal_sent:living_room_row_00_1
fallback_cmd_vel:living_room_row_00_1
goal_reached:living_room_row_00_1
goal_sent:living_room_row_01_0
goal_reached:living_room_row_01_0
goal_sent:living_room_row_01_1
goal_reached:living_room_row_01_1
finished:living_room
```

测试时机器人里程计从大约 `(1.55, 0.55)` 移动到客厅内部路径点附近，说明不是只发命令，机器人确实在移动清扫。

你也可以自己跑同一个测试：

```powershell
wsl -d Ubuntu-20.04-Mirror -- bash -lc 'cd /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project && source /opt/ros/noetic/setup.bash && source devel/setup.bash && bash ./test_room_cleaning_headless.sh living_room 75 false'
```

## 依赖安装

如果缺依赖，在 WSL Ubuntu 里安装：

```bash
sudo apt update
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

## 常见问题

### 在 WSL 里输入 `wsl -d ...` 提示 command not found

这是正常的。`wsl -d Ubuntu-20.04-Mirror -- ...` 要在 Windows PowerShell 里运行。

如果你已经在 Ubuntu WSL 里，就直接运行：

```bash
cd /mnt/d/coding/inkwork-term2/cleanbot_course_project_noetic/cleanbot_course_project
bash ./run_cleanbot_full_demo.sh
```

### 只看到 `publishing and latching message for 3.0 seconds`

这只说明房间命令发出去了，不代表完整系统已经启动。

先确认另一个窗口里已经跑着：

```powershell
.\run_cleanbot_full_demo.ps1
```

然后再发：

```powershell
.\run_clean_room.ps1 living_room
```

### 机器人不动或很快进入恢复旋转

现在默认起点已经放在客厅较开阔的位置，并且清扫节点带有 `/cmd_vel` 兜底控制。如果仍然不动，先跑：

```bash
bash ./test_room_cleaning_headless.sh living_room 75 false
```

看输出里有没有 `goal_reached` 和 `finished:living_room`。

## 主要文件

- `run_cleanbot_full_demo.ps1` / `run_cleanbot_full_demo.sh`：启动完整系统
- `run_clean_room.ps1` / `run_clean_room.sh`：发送房间清扫指令
- `run_cleanbot_auto_clean.ps1` / `run_cleanbot_auto_clean.sh`：启动后自动清扫指定房间
- `test_room_cleaning_headless.sh`：无界面验收测试脚本
- `src/cleanbot_course_project/launch/static_room_cleaning_demo.launch`：默认完整导航清扫 launch
- `src/cleanbot_course_project/config/rooms.yaml`：房间范围和覆盖路径配置
- `src/cleanbot_course_project/scripts/room_coverage_cleaner.py`：房间清扫逻辑
- `src/cleanbot_course_project/config/move_base.yaml`：导航参数
- `src/cleanbot_course_project/config/amcl.yaml`：定位参数
