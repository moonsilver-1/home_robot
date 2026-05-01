#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Arm preset publisher for ROS Noetic."""

import os

import rospy
import yaml
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _read_yaml(path):
    if not path:
        return {}
    if not os.path.isabs(path):
        path = os.path.abspath(path)
    if not os.path.exists(path):
        rospy.logwarn("Preset file not found: %s", path)
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else {}


DEFAULT_PRESETS = {
    "joint_names": ["joint1", "joint2", "joint3", "joint4"],
    "presets": {
        "stow": {"positions": [0.0, -0.2, 0.1, 0.0], "duration": 2.0},
        "scan_pose": {"positions": [0.0, -0.55, 0.65, 0.0], "duration": 2.5},
        "clean_pose": {"positions": [0.2, -0.7, 0.85, 0.1], "duration": 2.5},
        "pick_sim_pose": {"positions": [0.15, -0.45, 0.5, 0.0], "duration": 2.5},
    },
}


class ArmPresetActionNode:
    def __init__(self):
        self.arm_controller_topic = rospy.get_param("~arm_controller_topic", "/arm_controller/command")
        self.command_topic = rospy.get_param("~command_topic", "/cleanbot_arm/command")
        self.presets_file = rospy.get_param("~presets_file", "")

        data = DEFAULT_PRESETS.copy()
        loaded = _read_yaml(self.presets_file)
        if loaded:
            data.update({k: v for k, v in loaded.items() if k in ("joint_names", "presets")})
        self.joint_names = list(data.get("joint_names", DEFAULT_PRESETS["joint_names"]))
        self.presets = dict(data.get("presets", DEFAULT_PRESETS["presets"]))

        self.traj_pub = rospy.Publisher(self.arm_controller_topic, JointTrajectory, queue_size=1)
        rospy.Subscriber(self.command_topic, String, self._callback, queue_size=10)
        rospy.loginfo(
            "arm_preset_action_node ready. topic=%s command_topic=%s presets=%s",
            self.arm_controller_topic,
            self.command_topic,
            sorted(self.presets.keys()),
        )

    def _publish_preset(self, preset_name):
        preset = self.presets.get(preset_name)
        if preset is None:
            rospy.logwarn("Unknown arm preset: %s", preset_name)
            return

        positions = preset.get("positions", [])
        duration = float(preset.get("duration", 2.0))
        if len(positions) != len(self.joint_names):
            rospy.logwarn(
                "Preset %s has %d positions but %d joints are configured",
                preset_name,
                len(positions),
                len(self.joint_names),
            )
            return

        if self.traj_pub.get_num_connections() == 0:
            rospy.logwarn_throttle(
                5.0,
                "No subscribers on %s. Preset %s will still be published.",
                self.arm_controller_topic,
                preset_name,
            )

        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = rospy.Duration(duration)
        traj.points.append(point)
        self.traj_pub.publish(traj)
        rospy.loginfo("Published arm preset %s", preset_name)

    def _callback(self, msg):
        preset_name = msg.data.strip()
        if not preset_name:
            rospy.logwarn("Empty arm preset command received.")
            return
        self._publish_preset(preset_name)


def main():
    rospy.init_node("arm_preset_action_node")
    ArmPresetActionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
