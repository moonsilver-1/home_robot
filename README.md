# cleanbot_course_project

This project is a ROS Noetic course extension for a home cleaning robot.
It adds image preprocessing, ArUco detection, coverage scanning, person following, RGB-D target localization, and arm preset motions on top of a Patrol_Robot-style navigation stack.

## Environment

- Ubuntu 20.04
- ROS Noetic
- Python 3
- Gazebo Classic
- RViz
- `catkin_make`

## Dependencies

Install the main ROS and system packages:

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-move-base \
  ros-noetic-slam-gmapping \
  ros-noetic-map-server \
  ros-noetic-amcl \
  ros-noetic-trajectory-msgs \
  ros-noetic-visualization-msgs \
  ros-noetic-actionlib \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-rosbash \
  ros-noetic-roslint \
  ros-noetic-rosdep \
  ros-noetic-roslaunch \
  ros-noetic-robot-state-publisher \
  ros-noetic-joy \
  ros-noetic-teleop-twist-keyboard \
  python3-opencv \
  python3-yaml \
  python3-numpy
```

Optional, if you want the standalone ArUco ROS package in addition to the OpenCV-based node:

```bash
sudo apt install -y ros-noetic-aruco-ros
```

## Workspace Setup

```bash
mkdir -p ~/cleanbot_ws/src
cd ~/cleanbot_ws/src
git clone <this-repo-url> cleanbot_course_project
cd ~/cleanbot_ws
catkin_make
source devel/setup.bash
```

If you are using WSL, open the Ubuntu WSL shell first, then run the commands above inside Linux.

## Launch Files

### Vision

Starts image preprocessing and ArUco detection.

```bash
roslaunch cleanbot_course_project vision_demo.launch
```

Useful preview tools:

```bash
rqt_image_view /image_preprocess_node/gray
rqt_image_view /image_preprocess_node/edge
rqt_image_view /aruco_marker_detector/debug_image
```

### Coverage

Runs the move_base-based scan task manager.

```bash
roslaunch cleanbot_course_project coverage_demo.launch
```

### Person Following

Starts the detection-to-velocity controller.
The node is safe by default because `enable` defaults to `false`.

```bash
roslaunch cleanbot_course_project person_follow_demo.launch enable:=true
```

### RGB-D Localization

Converts a detected target into a 3D point and a RViz marker.

```bash
roslaunch cleanbot_course_project target_3d_demo.launch
```

### Arm Presets

Publishes preset joint trajectories for a demo arm controller.

```bash
roslaunch cleanbot_course_project arm_demo.launch
```

### Unified Demo

Starts the modular demo with launch arguments.

```bash
roslaunch cleanbot_course_project demo.launch
roslaunch cleanbot_course_project full_demo.launch
```

### Patrol_Robot Integration

Launch the cleanbot modules with common Patrol_Robot-style topics:

```bash
roslaunch cleanbot_course_project cleanbot_on_patrol_robot.launch
```

If Patrol_Robot uses different topic names, override the launch arguments instead of editing the nodes.

## Topic Summary

| Node | Input Topic | Output Topic | Type | Meaning |
| --- | --- | --- | --- | --- |
| `image_preprocess_node.py` | `/camera/rgb/image_raw` | `gray`, `blur`, `edge`, `hsv_mask` | `sensor_msgs/Image` | RGB preprocessing outputs for debugging and downstream CV |
| `aruco_marker_detector.py` | `/camera/rgb/image_raw` | `debug_image`, `markers` | `sensor_msgs/Image`, `std_msgs/String` | ArUco debug image and JSON marker report |
| `coverage_task_manager.py` | `move_base_action`, `scan_goals_file` | `status`, `scan_path`, `scan_markers` | `std_msgs/String`, `nav_msgs/Path`, `visualization_msgs/MarkerArray` | Coverage scan task status and RViz path markers |
| `person_follow_node.py` | `detection_topic` | `cmd_vel`, `status` | `std_msgs/String`, `geometry_msgs/Twist` | Safe person-follow controller using detection JSON |
| `target_3d_locator.py` | `detection_topic`, `depth_topic`, `camera_info_topic` | `target_point`, `target_marker` | `geometry_msgs/PointStamped`, `visualization_msgs/Marker` | RGB-D 3D localization of the selected target |
| `arm_preset_action_node.py` | `command_topic` | `arm_controller_topic` | `std_msgs/String`, `trajectory_msgs/JointTrajectory` | Demo arm preset publisher |

## Parameter Summary

### Image Preprocess

- `input_image_topic`
- `gray_topic`
- `blur_topic`
- `edge_topic`
- `hsv_mask_topic`
- `canny_low`
- `canny_high`
- `hsv_lower`
- `hsv_upper`

### ArUco Detector

- `input_image_topic`
- `debug_image_topic`
- `marker_topic`
- `aruco_dictionary`
- `camera_frame`

### Coverage Manager

- `move_base_action`
- `scan_goals_file`
- `goal_timeout`
- `retry_count`
- `frame_id`
- `status_topic`
- `path_marker_topic`
- `dry_run`

### Person Follow

- `detection_topic`
- `cmd_vel_topic`
- `target_class`
- `enable`
- `min_confidence`
- `desired_distance`
- `max_linear_speed`
- `max_angular_speed`
- `angular_kp`
- `linear_kp`
- `lost_timeout`
- `search_when_lost`

### RGB-D Locator

- `detection_topic`
- `depth_topic`
- `camera_info_topic`
- `target_class`
- `point_topic`
- `marker_topic`
- `depth_window_size`
- `camera_frame`
- `depth_scale`

### Arm Presets

- `arm_controller_topic`
- `presets_file`
- `command_topic`

## Config Files

- `config/scan_goals.yaml`: scan point list for coverage navigation
- `config/follow_params.yaml`: safe person-follow defaults
- `config/room_markers.yaml`: ArUco marker to room semantic mapping
- `config/arm_presets.yaml`: joint names and preset trajectories

## Build and Test

```bash
catkin_make
bash test/smoke_noetic.sh
```

If `rospack find cleanbot_course_project` fails, make sure you have sourced `devel/setup.bash` after building the workspace.

## Common Issues

### `cv_bridge` or OpenCV import error

Install the system OpenCV and bridge packages:

```bash
sudo apt install -y ros-noetic-cv-bridge python3-opencv python3-numpy
```

### ArUco API mismatch

The detector node handles both older `DetectorParameters_create()` and newer `ArucoDetector` APIs. If `cv2.aruco` is missing completely, install `python3-opencv` from Ubuntu or rebuild OpenCV with contrib modules.

### `move_base` action unavailable

Start the navigation stack first and check that the action server matches `move_base_action`, usually `/move_base`.

### Depth image missing

Make sure the depth camera publishes both `depth_topic` and `camera_info_topic`.

### Arm controller topic mismatch

Set `arm_controller_topic` and `command_topic` from launch arguments instead of editing the node code.

## Notes for Patrol_Robot

This repository is designed to adapt through launch arguments and remaps.
The default topics assume a common simulation layout:

- `/camera/rgb/image_raw`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`
- `/camera/depth/points`
- `/scan`
- `/map`
- `/odom`
- `/cmd_vel`
- `/move_base`

The actual Patrol_Robot topic names still need to be confirmed in your local copy of that project before final demo tuning.
