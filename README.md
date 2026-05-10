# 运行说明

运行前先
source ./devel/setup.bash  

可以使用
rqt_image_view
查看图像


# 键盘控制
## 单个机器人运行
rosrun turtlebot3_teleop turtlebot3_teleop_key

## 两个机器人运行（leadrer）
roslaunch cleanbot_course_project leader_teleop.launch


# 场景
## 单机器人 Gazebo + 小房子场景 + 机器人：
roslaunch cleanbot_course_project tb3_waffle_pi_house.launch
## 多个机器人运行(场景)
roslaunch cleanbot_course_project leader_follower_demo.launch

roslaunch cleanbot_course_project dual_waffle_pi_house.launch

# 任务运行

## 图像采集
roslaunch cleanbot_course_project data_collection.launch 


## 目标识别
roslaunch cleanbot_course_project object_recognition.launch


## 实时跟踪


# slam建图

```bash
roslaunch cleanbot_course_project robot_vacuum_house_demo.launch
```

这个启动后会：
- 启动 Gazebo 小房子场景和机器人
- 启动 `slam_gmapping`
- 在 RViz 里边走边建图

建图完成后可以用 `map_saver` 保存：
```bash
rosrun map_server map_saver -f ~/home_robot/src/cleanbot_course_project/map/my_map
```

# 自主导航
启动导航
roslaunch cleanbot_course_project room_navigation_demo.launch
房间命令
rosrun cleanbot_course_project room_navigation_console.py

# 3D点云
3D点云
roslaunch cleanbot_course_project pointcloud_view.launch


目标检测：蓝色椅子
roslaunch cleanbot_course_project blue_chair_nav.launch
