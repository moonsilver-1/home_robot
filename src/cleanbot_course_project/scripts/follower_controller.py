#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import rospy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class FollowerController:
    def __init__(self):
        self.leader_model_name = rospy.get_param("~leader_model_name", "leader")
        self.follower_model_name = rospy.get_param("~follower_model_name", "follower")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/follower/cmd_vel")

        self.target_distance = float(rospy.get_param("~target_distance", 0.8))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.55))

        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.22))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.8))

        self.linear_kp = float(rospy.get_param("~linear_kp", 0.45))
        self.angular_kp = float(rospy.get_param("~angular_kp", 1.2))

        self.angle_deadband = float(rospy.get_param("~angle_deadband", 0.08))
        self.distance_deadband = float(rospy.get_param("~distance_deadband", 0.08))

        self.lost_timeout = float(rospy.get_param("~lost_timeout", 1.0))
        self.control_rate = float(rospy.get_param("~control_rate", 20.0))

        self.latest_states = None
        self.last_states_time = None

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/cleanbot/follow/status", String, queue_size=10)

        self.sub = rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.model_states_callback,
            queue_size=1
        )

        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo("Follower controller started.")
        rospy.loginfo("Leader model: %s", self.leader_model_name)
        rospy.loginfo("Follower model: %s", self.follower_model_name)
        rospy.loginfo("Follower cmd_vel topic: %s", self.cmd_vel_topic)

    def model_states_callback(self, msg):
        self.latest_states = msg
        self.last_states_time = rospy.Time.now()

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def yaw_from_pose(pose):
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def get_pose_by_name(self, msg, name):
        if name not in msg.name:
            return None
        return msg.pose[msg.name.index(name)]

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def publish_status(self, state, distance=None, angle_error=None, linear_cmd=0.0, angular_cmd=0.0):
        data = {
            "state": state,
            "leader_model": self.leader_model_name,
            "follower_model": self.follower_model_name,
            "distance": distance,
            "angle_error": angle_error,
            "linear_cmd": linear_cmd,
            "angular_cmd": angular_cmd
        }
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def compute_cmd(self):
        if self.latest_states is None or self.last_states_time is None:
            self.stop_robot()
            self.publish_status("WAITING_MODEL_STATES")
            return

        age = (rospy.Time.now() - self.last_states_time).to_sec()
        if age > self.lost_timeout:
            self.stop_robot()
            self.publish_status("MODEL_STATES_TIMEOUT")
            return

        leader_pose = self.get_pose_by_name(self.latest_states, self.leader_model_name)
        follower_pose = self.get_pose_by_name(self.latest_states, self.follower_model_name)

        if leader_pose is None or follower_pose is None:
            self.stop_robot()
            self.publish_status("MODEL_NOT_FOUND")
            rospy.logwarn_throttle(
                2.0,
                "Cannot find [%s] or [%s] in /gazebo/model_states",
                self.leader_model_name,
                self.follower_model_name
            )
            return

        leader_x = leader_pose.position.x
        leader_y = leader_pose.position.y
        follower_x = follower_pose.position.x
        follower_y = follower_pose.position.y
        follower_yaw = self.yaw_from_pose(follower_pose)

        dx = leader_x - follower_x
        dy = leader_y - follower_y

        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - follower_yaw)

        twist = Twist()

        if distance < self.stop_distance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            state = "TOO_CLOSE_STOP"
        else:
            distance_error = distance - self.target_distance

            if abs(distance_error) < self.distance_deadband:
                linear_cmd = 0.0
            else:
                linear_cmd = self.linear_kp * distance_error

            if abs(angle_error) < self.angle_deadband:
                angular_cmd = 0.0
            else:
                angular_cmd = self.angular_kp * angle_error

            # 转向偏差太大时，降低前进速度，避免斜着撞上去
            if abs(angle_error) > 0.75:
                linear_cmd *= 0.2
            elif abs(angle_error) > 0.35:
                linear_cmd *= 0.55

            twist.linear.x = self.clamp(linear_cmd, 0.0, self.max_linear_speed)
            twist.angular.z = self.clamp(angular_cmd, -self.max_angular_speed, self.max_angular_speed)

            state = "FOLLOWING"

        self.cmd_pub.publish(twist)
        self.publish_status(
            state=state,
            distance=distance,
            angle_error=angle_error,
            linear_cmd=twist.linear.x,
            angular_cmd=twist.angular.z
        )

    def run(self):
        rate = rospy.Rate(self.control_rate)

        while not rospy.is_shutdown():
            self.compute_cmd()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("follower_controller")
    controller = FollowerController()
    controller.run()
