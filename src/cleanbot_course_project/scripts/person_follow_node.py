#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Person following controller that consumes JSON detections."""

import json
import math
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


class PersonFollowNode:
    def __init__(self):
        self.detection_topic = rospy.get_param("~detection_topic", "/yolo/detections_json")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.status_topic = rospy.get_param("~status_topic", "~status")
        self.target_class = rospy.get_param("~target_class", "person")
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.35))
        self.desired_distance = float(rospy.get_param("~desired_distance", 1.2))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.18))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 0.7))
        self.angular_kp = float(rospy.get_param("~angular_kp", 1.4))
        self.linear_kp = float(rospy.get_param("~linear_kp", 0.35))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 1.0))
        self.enable = bool(rospy.get_param("~enable", False))
        self.search_when_lost = bool(rospy.get_param("~search_when_lost", False))
        self.search_angular_speed = float(rospy.get_param("~search_angular_speed", 0.15))
        self.center_tolerance_px = float(rospy.get_param("~center_tolerance_px", 25.0))
        self.area_distance_constant = float(rospy.get_param("~area_distance_constant", 18000.0))
        self.default_image_width = float(rospy.get_param("~image_width", 640.0))
        self.default_image_height = float(rospy.get_param("~image_height", 480.0))

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)

        self.last_detection_time = 0.0
        self.last_status = ""
        self.latest_image_width = self.default_image_width
        self.latest_image_height = self.default_image_height

        rospy.Subscriber(self.detection_topic, String, self._callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._watchdog)
        self._publish_status("disabled" if not self.enable else "waiting")
        rospy.loginfo(
            "person_follow_node listening on %s and publishing to %s",
            self.detection_topic,
            self.cmd_vel_topic,
        )

    def _publish_status(self, status):
        if status != self.last_status:
            rospy.loginfo("person_follow status: %s", status)
            self.last_status = status
        self.status_pub.publish(String(data=status))

    def _stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _select_person(self, data):
        candidates = []
        for det in data.get("detections", []):
            class_name = det.get("class", det.get("name", ""))
            confidence = _safe_float(det.get("confidence", det.get("score", 0.0)))
            bbox = det.get("bbox")
            if class_name != self.target_class or confidence < self.min_confidence or not bbox:
                continue
            if len(bbox) != 4:
                continue
            x1, y1, x2, y2 = [_safe_float(v) for v in bbox]
            candidates.append(
                {
                    "confidence": confidence,
                    "bbox": [x1, y1, x2, y2],
                    "depth": det.get("depth"),
                }
            )

        for det in data.get("objects", []):
            class_name = det.get("class", det.get("name", ""))
            confidence = _safe_float(det.get("confidence", det.get("score", 0.0)))
            if class_name != self.target_class or confidence < self.min_confidence:
                continue
            bbox = [
                _safe_float(det.get("xmin", 0.0)),
                _safe_float(det.get("ymin", 0.0)),
                _safe_float(det.get("xmax", 0.0)),
                _safe_float(det.get("ymax", 0.0)),
            ]
            candidates.append(
                {
                    "confidence": confidence,
                    "bbox": bbox,
                    "depth": det.get("depth"),
                }
            )

        if not candidates:
            return None
        return max(candidates, key=lambda item: item["confidence"])

    def _estimate_distance(self, bbox, depth_value):
        if depth_value is not None:
            try:
                distance = float(depth_value)
                if math.isfinite(distance) and distance > 0.0:
                    return distance
            except Exception:
                pass

        x1, y1, x2, y2 = bbox
        area = max(1.0, (x2 - x1) * (y2 - y1))
        return max(0.01, self.area_distance_constant / math.sqrt(area))

    def _command_from_detection(self, detection, image_width):
        x1, y1, x2, y2 = detection["bbox"]
        center_x = 0.5 * (x1 + x2)
        width = max(1.0, float(image_width))
        center_error = (center_x - width * 0.5) / (width * 0.5)
        if abs((center_x - width * 0.5)) <= self.center_tolerance_px:
            center_error = 0.0

        distance = self._estimate_distance(detection["bbox"], detection.get("depth"))
        distance_error = distance - self.desired_distance

        cmd = Twist()
        cmd.angular.z = _clamp(-self.angular_kp * center_error, -self.max_angular_speed, self.max_angular_speed)
        if distance_error > 0.0:
            cmd.linear.x = _clamp(self.linear_kp * distance_error, 0.0, self.max_linear_speed)
        else:
            cmd.linear.x = 0.0
        return cmd, distance, center_error

    def _callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse detection JSON: %s", exc)
            return

        self.latest_image_width = float(data.get("image_width", self.default_image_width))
        self.latest_image_height = float(data.get("image_height", self.default_image_height))

        if not self.enable:
            self._publish_status("disabled")
            self._stop_robot()
            return

        detection = self._select_person(data)
        if detection is None:
            self.last_detection_time = self.last_detection_time or time.time()
            self._publish_status("lost")
            if self.search_when_lost:
                cmd = Twist()
                cmd.angular.z = _clamp(self.search_angular_speed, 0.0, self.max_angular_speed)
                self.cmd_pub.publish(cmd)
            else:
                self._stop_robot()
            return

        self.last_detection_time = time.time()
        cmd, distance, center_error = self._command_from_detection(detection, self.latest_image_width)
        self.cmd_pub.publish(cmd)
        self._publish_status("following")
        rospy.logdebug(
            "person_follow target distance=%.3f center_error=%.3f cmd=(%.3f, %.3f)",
            distance,
            center_error,
            cmd.linear.x,
            cmd.angular.z,
        )

    def _watchdog(self, _event):
        if not self.enable:
            return
        if self.last_detection_time <= 0.0:
            return
        if time.time() - self.last_detection_time <= self.lost_timeout:
            return

        self._publish_status("lost")
        if self.search_when_lost:
            cmd = Twist()
            cmd.angular.z = _clamp(self.search_angular_speed, 0.0, self.max_angular_speed)
            self.cmd_pub.publish(cmd)
        else:
            self._stop_robot()


def main():
    rospy.init_node("person_follow_node")
    PersonFollowNode()
    rospy.spin()


if __name__ == "__main__":
    main()
