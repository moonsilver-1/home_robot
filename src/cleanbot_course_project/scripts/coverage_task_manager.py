#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Coverage task manager that sends scan goals to move_base."""

import math
import os

import actionlib
import rospy
import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(float(yaw) * 0.5)
    q.w = math.cos(float(yaw) * 0.5)
    return q


def _read_yaml(path):
    if not path:
        return {}
    if not os.path.isabs(path):
        path = os.path.abspath(path)
    if not os.path.exists(path):
        rospy.logerr("scan_goals_file does not exist: %s", path)
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        rospy.logerr("scan_goals_file must contain a mapping: %s", path)
        return {}
    return data


class CoverageTaskManager:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.scan_goals_file = rospy.get_param("~scan_goals_file", "")
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.retry_count = int(rospy.get_param("~retry_count", 1))
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.status_topic = rospy.get_param("~status_topic", "~status")
        self.path_marker_topic = rospy.get_param("~path_marker_topic", "~scan_path")

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher(self.path_marker_topic, Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("~scan_markers", MarkerArray, queue_size=1, latch=True)
        self.client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)

        config = _read_yaml(self.scan_goals_file)
        self.scan_goals = rospy.get_param("~scan_goals", config.get("scan_goals", []))
        if not self.scan_goals:
            raise rospy.ROSInitException("No scan goals configured")

        self._publish_path()
        rospy.loginfo(
            "coverage_task_manager ready with %d goals, action=%s, dry_run=%s",
            len(self.scan_goals),
            self.move_base_action,
            self.dry_run,
        )

    def _publish_status(self, text):
        self.status_pub.publish(String(data=text))
        rospy.loginfo("%s", text)

    def _goal_pose(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(goal.get("x", 0.0))
        pose.pose.position.y = float(goal.get("y", 0.0))
        pose.pose.position.z = float(goal.get("z", 0.0))
        pose.pose.orientation = _yaw_to_quaternion(goal.get("yaw", 0.0))
        return pose

    def _publish_path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()

        markers = MarkerArray()
        for index, goal in enumerate(self.scan_goals):
            pose = self._goal_pose(goal)
            path.poses.append(pose)

            marker = Marker()
            marker.header = pose.header
            marker.ns = "scan_goals"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.pose.position.z = 0.08
            marker.scale.x = marker.scale.y = marker.scale.z = 0.16
            marker.color.a = 0.95
            marker.color.r = 0.1
            marker.color.g = 0.55
            marker.color.b = 1.0
            markers.markers.append(marker)

            label = Marker()
            label.header = pose.header
            label.ns = "scan_labels"
            label.id = 1000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = pose.pose
            label.pose.position.z = 0.35
            label.scale.z = 0.18
            label.color.a = 1.0
            label.color.r = label.color.g = label.color.b = 0.05
            label.text = goal.get("name", "goal_%d" % index)
            markers.markers.append(label)

        self.path_pub.publish(path)
        self.marker_pub.publish(markers)

    def _send_goal(self, goal_dict):
        pose = self._goal_pose(goal_dict)
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose = pose
        self.client.send_goal(mb_goal)
        return self.client.wait_for_result(rospy.Duration(self.goal_timeout))

    def run(self):
        if self.dry_run:
            self._publish_status("dry_run")
            for index, goal in enumerate(self.scan_goals):
                rospy.loginfo(
                    "[dry_run] goal %d/%d %s -> (%.3f, %.3f, %.3f)",
                    index + 1,
                    len(self.scan_goals),
                    goal.get("name", "goal_%d" % index),
                    float(goal.get("x", 0.0)),
                    float(goal.get("y", 0.0)),
                    float(goal.get("yaw", 0.0)),
                )
            return

        self._publish_status("waiting_for_move_base")
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("move_base action server %s is not available", self.move_base_action)
            self._publish_status("move_base_unavailable")
            return

        self._publish_status("running")
        for index, goal in enumerate(self.scan_goals):
            if rospy.is_shutdown():
                break

            name = goal.get("name", "goal_%d" % index)
            for attempt in range(self.retry_count + 1):
                if rospy.is_shutdown():
                    break

                rospy.loginfo(
                    "Sending scan goal %d/%d (%s), attempt %d/%d",
                    index + 1,
                    len(self.scan_goals),
                    name,
                    attempt + 1,
                    self.retry_count + 1,
                )
                self._publish_status("goal_sent:%s" % name)
                finished = self._send_goal(goal)
                if finished and self.client.get_state() == GoalStatus.SUCCEEDED:
                    self._publish_status("goal_reached:%s" % name)
                    break

                rospy.logwarn(
                    "Goal %s failed or timed out (attempt %d/%d)",
                    name,
                    attempt + 1,
                    self.retry_count + 1,
                )
                self.client.cancel_goal()
                if attempt < self.retry_count:
                    rospy.sleep(1.0)
            else:
                self._publish_status("goal_failed:%s" % name)

        self._publish_status("finished")


def main():
    rospy.init_node("coverage_task_manager")
    node = CoverageTaskManager()
    node.run()


if __name__ == "__main__":
    main()
