#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Room-level coverage cleaner for ROS Noetic."""

import math
import os

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _read_yaml(path):
    if not path:
        return {}
    if not os.path.isabs(path):
        path = os.path.abspath(path)
    if not os.path.exists(path):
        rospy.logerr("rooms_file does not exist: %s", path)
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        rospy.logerr("rooms_file must contain a mapping: %s", path)
        return {}
    return data


def _yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(float(yaw) * 0.5)
    q.w = math.cos(float(yaw) * 0.5)
    return q


def _quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _point(point):
    return float(point.get("x", 0.0)), float(point.get("y", 0.0))


def _scanline_segments(polygon, y_value):
    xs = []
    points = [_point(item) for item in polygon]
    for index, first in enumerate(points):
        second = points[(index + 1) % len(points)]
        x1, y1 = first
        x2, y2 = second
        if y1 == y2:
            continue
        low = min(y1, y2)
        high = max(y1, y2)
        if y_value < low or y_value >= high:
            continue
        ratio = (y_value - y1) / (y2 - y1)
        xs.append(x1 + ratio * (x2 - x1))
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2)]


def generate_room_goals(room_name, room_config, default_step=0.45, default_yaw=0.0):
    polygon = room_config.get("polygon", [])
    if len(polygon) < 3:
        raise ValueError("room %s needs at least 3 polygon points" % room_name)

    step = float(room_config.get("step", default_step))
    if step <= 0.0:
        raise ValueError("room %s has invalid step %.3f" % (room_name, step))

    points = [_point(item) for item in polygon]
    min_y = min(y for _x, y in points)
    max_y = max(y for _x, y in points)

    goals = []
    y_value = min_y + step * 0.5
    row = 0
    while y_value < max_y:
        segments = _scanline_segments(polygon, y_value)
        if row % 2 == 1:
            segments = list(reversed(segments))
        for x_start, x_end in segments:
            if x_end - x_start < 0.05:
                continue
            left = x_start + step * 0.5
            right = x_end - step * 0.5
            if left > right:
                center = 0.5 * (x_start + x_end)
                left = right = center
            pair = [(left, y_value), (right, y_value)]
            if row % 2 == 1:
                pair.reverse()
            for point_index, (x_value, y_point) in enumerate(pair):
                goals.append(
                    {
                        "name": "%s_row_%02d_%d" % (room_name, row, point_index),
                        "x": x_value,
                        "y": y_point,
                        "yaw": default_yaw if point_index == 0 else math.pi,
                    }
                )
        row += 1
        y_value += step
    return goals


class RoomCoverageCleaner:
    def __init__(self):
        self.rooms_file = rospy.get_param("~rooms_file", "")
        self.command_topic = rospy.get_param("~command_topic", "/cleanbot/clean_room")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.retry_count = int(rospy.get_param("~retry_count", 1))
        self.dry_run = bool(rospy.get_param("~dry_run", False))
        self.autostart_room = rospy.get_param("~autostart_room", "")
        self.autostart_delay = float(rospy.get_param("~autostart_delay", 0.5))
        self.enable_cmd_vel_fallback = bool(rospy.get_param("~enable_cmd_vel_fallback", True))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.fallback_goal_timeout = float(rospy.get_param("~fallback_goal_timeout", 25.0))
        self.fallback_xy_tolerance = float(rospy.get_param("~fallback_xy_tolerance", 0.16))
        self.fallback_max_linear = float(rospy.get_param("~fallback_max_linear", 0.16))
        self.fallback_max_angular = float(rospy.get_param("~fallback_max_angular", 0.8))
        self.status_topic = rospy.get_param("~status_topic", "~status")
        self.path_topic = rospy.get_param("~path_topic", "~cleaning_path")
        self.marker_topic = rospy.get_param("~marker_topic", "~cleaning_markers")
        self.current_pose = None

        config = _read_yaml(self.rooms_file)
        self.frame_id = rospy.get_param("~frame_id", config.get("frame_id", "map"))
        self.default_step = float(rospy.get_param("~default_step", config.get("default_step", 0.45)))
        self.default_yaw = float(rospy.get_param("~default_yaw", config.get("default_yaw", 0.0)))
        self.rooms = config.get("rooms", {})
        if not self.rooms:
            raise rospy.ROSInitException("No rooms configured")

        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)
        rospy.Subscriber(self.command_topic, String, self._command_cb, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)

        rospy.loginfo(
            "room_coverage_cleaner ready. rooms=%s command_topic=%s dry_run=%s",
            sorted(self.rooms.keys()),
            self.command_topic,
            self.dry_run,
        )
        if self.autostart_room:
            rospy.Timer(rospy.Duration(self.autostart_delay), self._autostart_once, oneshot=True)

    def _autostart_once(self, _event):
        self.clean_room(self.autostart_room)

    def _odom_cb(self, msg):
        pose = msg.pose.pose
        self.current_pose = (
            float(pose.position.x),
            float(pose.position.y),
            _quaternion_to_yaw(pose.orientation),
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
        pose.pose.orientation = _yaw_to_quaternion(goal.get("yaw", self.default_yaw))
        return pose

    def _goals_for_request(self, room_name):
        requested = room_name.strip()
        if requested == "all":
            goals = []
            for name in sorted(self.rooms.keys()):
                goals.extend(generate_room_goals(name, self.rooms[name], self.default_step, self.default_yaw))
            return goals
        if requested not in self.rooms:
            raise KeyError("Unknown room: %s" % requested)
        return generate_room_goals(requested, self.rooms[requested], self.default_step, self.default_yaw)

    def _publish_path(self, goals):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = rospy.Time.now()
        markers = MarkerArray()
        for index, goal in enumerate(goals):
            pose = self._goal_pose(goal)
            path.poses.append(pose)

            marker = Marker()
            marker.header = pose.header
            marker.ns = "room_cleaning_goals"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose.pose
            marker.pose.position.z = 0.08
            marker.scale.x = marker.scale.y = marker.scale.z = 0.14
            marker.color.a = 0.95
            marker.color.r = 0.0
            marker.color.g = 0.65
            marker.color.b = 0.25
            markers.markers.append(marker)

        self.path_pub.publish(path)
        self.marker_pub.publish(markers)

    def _send_goal(self, goal):
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose = self._goal_pose(goal)
        self.client.send_goal(mb_goal)
        return self.client.wait_for_result(rospy.Duration(self.goal_timeout))

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def _drive_goal_with_cmd_vel(self, goal):
        if not self.enable_cmd_vel_fallback:
            return False
        if self.current_pose is None:
            rospy.logwarn("cmd_vel fallback requested but no odom has been received yet")
            return False

        target_x = float(goal.get("x", 0.0))
        target_y = float(goal.get("y", 0.0))
        deadline = rospy.Time.now() + rospy.Duration(self.fallback_goal_timeout)
        rate = rospy.Rate(12.0)
        self._publish_status("fallback_cmd_vel:%s" % goal.get("name", "goal"))

        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.current_pose is None:
                rate.sleep()
                continue
            x_value, y_value, yaw = self.current_pose
            dx = target_x - x_value
            dy = target_y - y_value
            distance = math.hypot(dx, dy)
            if distance <= self.fallback_xy_tolerance:
                self._stop_base()
                return True

            target_yaw = math.atan2(dy, dx)
            yaw_error = _normalize_angle(target_yaw - yaw)
            cmd = Twist()
            if abs(yaw_error) > 0.30:
                cmd.angular.z = max(
                    -self.fallback_max_angular,
                    min(self.fallback_max_angular, 1.6 * yaw_error),
                )
            else:
                cmd.linear.x = min(self.fallback_max_linear, max(0.05, 0.7 * distance))
                cmd.angular.z = max(-0.45, min(0.45, 1.2 * yaw_error))
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self._stop_base()
        return False

    def clean_room(self, room_name):
        try:
            goals = self._goals_for_request(room_name)
        except Exception as exc:
            self._publish_status("invalid_room:%s" % room_name)
            rospy.logwarn("%s", exc)
            return

        if not goals:
            self._publish_status("no_goals:%s" % room_name)
            return

        self._publish_path(goals)
        self._publish_status("planned:%s:%d" % (room_name, len(goals)))

        if self.dry_run:
            for index, goal in enumerate(goals):
                rospy.loginfo(
                    "[dry_run] cleaning goal %d/%d %s -> (%.3f, %.3f, %.3f)",
                    index + 1,
                    len(goals),
                    goal["name"],
                    float(goal["x"]),
                    float(goal["y"]),
                    float(goal["yaw"]),
                )
            self._publish_status("dry_run_finished:%s" % room_name)
            return

        self._publish_status("waiting_for_move_base")
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("move_base action server %s is not available", self.move_base_action)
            self._publish_status("move_base_unavailable")
            return

        for index, goal in enumerate(goals):
            name = goal.get("name", "goal_%d" % index)
            reached = False
            for attempt in range(self.retry_count + 1):
                if rospy.is_shutdown():
                    return
                self._publish_status("goal_sent:%s" % name)
                finished = self._send_goal(goal)
                if finished and self.client.get_state() == GoalStatus.SUCCEEDED:
                    self._publish_status("goal_reached:%s" % name)
                    reached = True
                    break
                self.client.cancel_goal()
                rospy.logwarn("Cleaning goal %s failed or timed out", name)
                if self._drive_goal_with_cmd_vel(goal):
                    self._publish_status("goal_reached:%s" % name)
                    reached = True
                    break
            if not reached:
                self._publish_status("goal_failed:%s" % name)
                return
        self._publish_status("finished:%s" % room_name)

    def _command_cb(self, msg):
        room_name = msg.data.strip()
        if not room_name:
            self._publish_status("empty_command")
            return
        self.clean_room(room_name)


def main():
    rospy.init_node("room_coverage_cleaner")
    RoomCoverageCleaner()
    rospy.spin()


if __name__ == "__main__":
    main()
