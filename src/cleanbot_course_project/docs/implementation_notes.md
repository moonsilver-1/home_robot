# 目前已经完成

## 识别物体：
 包括识别蓝色球和椅子

1. 启动gazebo
```
  roslaunch cleanbot_course_project tb3_waffle_pi_house.launch
```


2. 打开目标识别
```
   roslaunch cleanbot_course_project object_recognition.launch 
```


3. 键盘操控

```
rosrun turtlebot3_teleop turtlebot3_teleop_key
```

## 双车跟随

```
roslaunch cleanbot_course_project leader_follower_demo.launch
```

这个版本里前车会默认显示成红色，后车会通过摄像头识别红色目标并持续跟随。

## 摄像头识别跟随

先把 leader 的参考图放到 `datasets/object_samples/leader/`，例如：

```
datasets/object_samples/leader/001.png
datasets/object_samples/leader/002.png
```


导航
roslaunch cleanbot_course_project static_room_cleaning_demo.launch


l是客厅，k是厨房，b是卧室