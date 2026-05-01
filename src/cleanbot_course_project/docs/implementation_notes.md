# Implementation Notes

## Included Nodes

1. `image_preprocess_node.py`
   - Subscribes to RGB images.
   - Publishes gray, blur, edge, and HSV mask images.
   - Useful for `rqt_image_view` and quick camera debugging.

2. `aruco_marker_detector.py`
   - Subscribes to RGB images.
   - Publishes a debug image and a JSON marker report.
   - Uses the OpenCV ArUco API and supports newer and older OpenCV 4 variants.

3. `coverage_task_manager.py`
   - Reads scan goals from YAML.
   - Sends `move_base` action goals in sequence.
   - Publishes a `nav_msgs/Path` and RViz markers.

4. `person_follow_node.py`
   - Consumes JSON detections.
   - Produces a conservative `cmd_vel` command.
   - Disabled by default for safety.

5. `target_3d_locator.py`
   - Consumes JSON detections, depth images, and camera info.
   - Publishes `geometry_msgs/PointStamped` and an RViz marker.

6. `arm_preset_action_node.py`
   - Reads arm presets from YAML.
   - Publishes `trajectory_msgs/JointTrajectory` to the arm controller topic.

## Patrol_Robot Integration

The project is intended to be launched alongside Patrol_Robot through remaps and launch arguments.
The default topics assume a common simulation layout, but the actual Patrol_Robot topic names still need to be confirmed in your local copy of that repository.

## Safety Notes

- Person following is disabled by default.
- Coverage uses `move_base` action goals, not a direct velocity stream.
- Arm control only publishes preset trajectories, not full manipulation or visual servoing.
