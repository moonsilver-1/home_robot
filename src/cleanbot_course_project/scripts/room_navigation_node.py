#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading
from pathlib import Path

import actionlib
import rospy
import yaml

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class RoomNavigationNode:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.command_topic = rospy.get_param("~command_topic", "/cleanbot/room_navigation/command")
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 90.0))
        self.retry_count = int(rospy.get_param("~retry_count", 1))
        self.start_x = float(rospy.get_param("~start_x", -1.5))
        self.start_y = float(rospy.get_param("~start_y", 0.0))
        self.start_yaw = float(rospy.get_param("~start_yaw", 0.0))
        self.rooms_config = Path(rospy.get_param("~rooms_config", self.default_rooms_config()))
        self.scan_goals_config = Path(rospy.get_param("~scan_goals_config", self.default_scan_goals_config()))

        self.initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/cleanbot/room_navigation/status", String, queue_size=10)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.command_sub = rospy.Subscriber(self.command_topic, String, self.command_callback, queue_size=10)

        self.rooms = self.load_rooms()
        self.scan_goals = self.load_scan_goals()
        self.command_aliases = {
            "l": "living_room",
            "living_room": "living_room",
            "客厅": "living_room",
            "k": "kitchen",
            "kitchen": "kitchen",
            "厨房": "kitchen",
            "b": "bedroom",
            "bedroom": "bedroom",
            "卧室": "bedroom",
            "study": "study",
            "书房": "study",
        }

        self.pending_room = None
        self.pending_lock = threading.Lock()

        rospy.on_shutdown(self.cancel_goal)

        rospy.loginfo("Room navigation node started.")
        rospy.loginfo("Command topic: %s", self.command_topic)
        rospy.loginfo("Commands: l=living_room, k=kitchen, b=bedroom")
        rospy.loginfo("Rooms config: %s", str(self.rooms_config))
        rospy.loginfo("Scan goals config: %s", str(self.scan_goals_config))
        rospy.loginfo("Start pose: x=%.3f y=%.3f yaw=%.3f", self.start_x, self.start_y, self.start_yaw)

    @staticmethod
    def default_rooms_config():
        from rospkg import RosPack

        return str(Path(RosPack().get_path("cleanbot_course_project")) / "config/rooms.yaml")

    @staticmethod
    def default_scan_goals_config():
        from rospkg import RosPack

        return str(Path(RosPack().get_path("cleanbot_course_project")) / "config/scan_goals.yaml")

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def load_yaml(self, path):
        if not path.exists():
            rospy.logwarn("YAML file not found: %s", str(path))
            return {}
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def load_rooms(self):
        data = self.load_yaml(self.rooms_config)
        return data.get("rooms", {})

    def load_scan_goals(self):
        data = self.load_yaml(self.scan_goals_config)
        goals = {}
        for item in data.get("scan_goals", []):
            name = item.get("name")
            if name:
                goals[name] = item
        return goals

    @staticmethod
    def polygon_centroid(points):
        if len(points) < 3:
            return 0.0, 0.0

        area = 0.0
        cx = 0.0
        cy = 0.0

        for i in range(len(points)):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % len(points)]
            cross = x1 * y2 - x2 * y1
            area += cross
            cx += (x1 + x2) * cross
            cy += (y1 + y2) * cross

        area *= 0.5
        if abs(area) < 1e-9:
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            return sum(xs) / len(xs), sum(ys) / len(ys)

        cx /= (6.0 * area)
        cy /= (6.0 * area)
        return cx, cy

    def resolve_room_command(self, raw_value):
        value = str(raw_value).strip()
        if not value:
            return None
        value_key = value.lower()
        if value_key in self.command_aliases:
            return self.command_aliases[value_key]
        if value_key in self.rooms or value_key in self.scan_goals:
            return value_key
        if value in self.rooms or value in self.scan_goals:
            return value
        return None

    def get_room_goal(self, room_name):
        preferred_names = [
            f"{room_name}_entry",
            f"{room_name}_center",
            f"{room_name}_corner",
        ]
        for name in preferred_names:
            if name in self.scan_goals:
                goal = self.scan_goals[name]
                return {
                    "name": name,
                    "x": float(goal["x"]),
                    "y": float(goal["y"]),
                    "yaw": float(goal.get("yaw", 0.0)),
                    "source": "scan_goals",
                }

        room = self.rooms.get(room_name)
        if not room:
            return None

        polygon = room.get("polygon", [])
        points = [(float(p["x"]), float(p["y"])) for p in polygon if "x" in p and "y" in p]
        if not points:
            return None

        x, y = self.polygon_centroid(points)
        return {
            "name": room_name,
            "x": float(x),
            "y": float(y),
            "yaw": 0.0,
            "source": "rooms_centroid",
        }

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = self.start_x
        msg.pose.pose.position.y = self.start_y
        msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, self.start_yaw)
        msg.pose.pose.orientation = Quaternion(*q)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
        ]
        self.initial_pose_pub.publish(msg)
        rospy.loginfo("Published initial pose to /initialpose.")

    def cancel_goal(self):
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass

    def command_callback(self, msg):
        room_name = self.resolve_room_command(msg.data)
        if room_name is None:
            rospy.logwarn("Unknown room command: %s", msg.data)
            self.status_pub.publish(String(data=f"INVALID:{msg.data}"))
            return

        with self.pending_lock:
            self.pending_room = room_name

        rospy.loginfo("Received command [%s] -> %s", msg.data, room_name)
        self.status_pub.publish(String(data=f"COMMAND:{msg.data}:{room_name}"))

    def wait_for_goal_result(self, room_name, timeout):
        start_time = rospy.Time.now()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            with self.pending_lock:
                pending_room = self.pending_room

            if pending_room and pending_room != room_name:
                rospy.loginfo("Preempting [%s] for new room command [%s].", room_name, pending_room)
                self.client.cancel_goal()
                return GoalStatus.PREEMPTED

            state = self.client.get_state()
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED,
                         GoalStatus.RECALLED, GoalStatus.LOST):
                return state

            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return None
            rate.sleep()

        return None

    def send_room_goal(self, room_name):
        goal_info = self.get_room_goal(room_name)
        if goal_info is None:
            rospy.logerr("Unknown room or missing waypoint: %s", room_name)
            self.status_pub.publish(String(data=f"FAILED:{room_name}:unknown_room"))
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_info["x"]
        goal.target_pose.pose.position.y = goal_info["y"]
        goal.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, goal_info["yaw"])
        goal.target_pose.pose.orientation = Quaternion(*q)

        rospy.loginfo(
            "Sending room goal [%s] from %s: x=%.3f y=%.3f yaw=%.3f",
            room_name,
            goal_info["source"],
            goal_info["x"],
            goal_info["y"],
            goal_info["yaw"],
        )
        self.status_pub.publish(String(data=f"SENT:{room_name}:{goal_info['source']}"))
        self.client.send_goal(goal)
        result_state = self.wait_for_goal_result(room_name, self.goal_timeout)

        if result_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached room: %s", room_name)
            self.status_pub.publish(String(data=f"SUCCEEDED:{room_name}"))
            return True

        if result_state is None:
            rospy.logwarn("Timed out waiting for room goal: %s", room_name)
            self.status_pub.publish(String(data=f"TIMEOUT:{room_name}"))
        else:
            rospy.logwarn("Room goal finished with state %s: %s", result_state, room_name)
            self.status_pub.publish(String(data=f"FAILED:{room_name}:{result_state}"))
        return False

    def run(self):
        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(30.0)):
            raise RuntimeError("move_base action server is not available")

        self.publish_initial_pose()
        rospy.sleep(2.0)
        rospy.loginfo("Ready for room commands on %s", self.command_topic)
        self.status_pub.publish(String(data="READY"))

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            with self.pending_lock:
                room_name = self.pending_room

            if room_name is None:
                rate.sleep()
                continue

            attempts = 0
            success = False
            while attempts <= self.retry_count and not rospy.is_shutdown():
                attempts += 1
                if self.send_room_goal(room_name):
                    success = True
                    break

                if attempts <= self.retry_count:
                    rospy.loginfo(
                        "Retrying room [%s], attempt %d/%d",
                        room_name,
                        attempts + 1,
                        self.retry_count + 1,
                    )

                with self.pending_lock:
                    if self.pending_room != room_name:
                        break

            with self.pending_lock:
                if self.pending_room == room_name:
                    self.pending_room = None

            if success:
                self.status_pub.publish(String(data=f"READY:{room_name}"))
            else:
                rospy.logwarn("Room command [%s] did not complete.", room_name)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("room_navigation_node")
    node = RoomNavigationNode()
    try:
        node.run()
    except Exception as exc:
        rospy.logerr("Room navigation node failed: %s", exc)
        raise
