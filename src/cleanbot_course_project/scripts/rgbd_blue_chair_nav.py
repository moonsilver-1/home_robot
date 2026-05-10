#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math

import actionlib
import cv2
import numpy as np
import rospy
import tf2_ros

from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler, quaternion_matrix
from visualization_msgs.msg import Marker


class RGBDBlueChairNavigator:
    def __init__(self):
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/depth/camera_info")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_rgb_optical_frame")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.min_area = float(rospy.get_param("~min_area", 500.0))
        self.min_depth = float(rospy.get_param("~min_depth", 0.1))
        self.max_depth = float(rospy.get_param("~max_depth", 10.0))
        self.approach_distance = float(rospy.get_param("~approach_distance", 0.9))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.25))
        self.resend_distance = float(rospy.get_param("~resend_distance", 0.35))
        self.visual_target_area_ratio = float(rospy.get_param("~visual_target_area_ratio", 0.08))
        self.visual_center_deadband = float(rospy.get_param("~visual_center_deadband", 0.08))
        self.visual_area_deadband = float(rospy.get_param("~visual_area_deadband", 0.02))
        self.visual_center_kp = float(rospy.get_param("~visual_center_kp", 0.9))
        self.visual_distance_kp = float(rospy.get_param("~visual_distance_kp", 0.8))
        self.visual_max_linear_speed = float(rospy.get_param("~visual_max_linear_speed", 0.10))
        self.visual_max_angular_speed = float(rospy.get_param("~visual_max_angular_speed", 0.50))
        self.visual_steering_sign = float(rospy.get_param("~visual_steering_sign", -1.0))
        self.show_debug = self._as_bool(rospy.get_param("~show_debug", False))

        self.chair_hsv_lower = self._parse_hsv_param("~chair_hsv_lower", [100, 80, 20])
        self.chair_hsv_upper = self._parse_hsv_param("~chair_hsv_upper", [130, 255, 170])

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.info = None
        self.last_distance = None
        self.last_goal_xy = None
        self.active_goal = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.annotated_pub = rospy.Publisher("~annotated_image", Image, queue_size=1)
        self.chair_point_pub = rospy.Publisher("~chair_point", PointStamped, queue_size=1)
        self.chair_info_pub = rospy.Publisher("~chair_info", String, queue_size=10)
        self.chair_marker_pub = rospy.Publisher("~chair_marker", Marker, queue_size=1)
        self.goal_marker_pub = rospy.Publisher("~goal_marker", Marker, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.sub_rgb = rospy.Subscriber(self.rgb_topic, Image, self.cb_rgb, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1, buff_size=2**24)
        self.sub_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("RGB-D blue chair navigator started.")
        rospy.loginfo("RGB topic: %s", self.rgb_topic)
        rospy.loginfo("Depth topic: %s", self.depth_topic)
        rospy.loginfo("Camera info topic: %s", self.camera_info_topic)
        rospy.loginfo("Camera frame: %s", self.camera_frame)
        rospy.loginfo("Map frame: %s", self.map_frame)
        rospy.loginfo("Base frame: %s", self.base_frame)
        rospy.loginfo("Blue chair HSV lower: %s", self.chair_hsv_lower.tolist())
        rospy.loginfo("Blue chair HSV upper: %s", self.chair_hsv_upper.tolist())

        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()
        rospy.loginfo("move_base is ready.")

    def on_shutdown(self):
        try:
            self.client.cancel_all_goals()
        except Exception:
            pass
        self.cmd_pub.publish(Twist())
        if self.show_debug:
            cv2.destroyAllWindows()

    @staticmethod
    def _parse_hsv_param(name, default):
        raw = rospy.get_param(name, default)
        if isinstance(raw, str):
            parts = [p.strip() for p in raw.split(",") if p.strip()]
            if len(parts) != 3:
                raise rospy.ROSException(f"Parameter {name} must have 3 comma-separated values")
            values = [int(p) for p in parts]
        else:
            values = [int(v) for v in raw]
        return np.array(values, dtype=np.uint8)

    @staticmethod
    def _as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("true", "1", "yes", "on")
        return bool(value)

    @staticmethod
    def _depth_to_meters(depth_img):
        depth = np.asarray(depth_img)
        if depth.dtype == np.uint16 or np.nanmax(depth) > 50.0:
            return depth.astype(np.float32) / 1000.0
        return depth.astype(np.float32)

    @staticmethod
    def _apply_transform(transform, x, y, z):
        q = transform.transform.rotation
        t = transform.transform.translation
        mat = quaternion_matrix([q.x, q.y, q.z, q.w])
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z
        p = np.array([x, y, z, 1.0], dtype=np.float64)
        out = mat.dot(p)
        return float(out[0]), float(out[1]), float(out[2])

    def _lookup_point(self, target_frame, source_frame, x, y, z):
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.5))
        return self._apply_transform(transform, x, y, z)

    def _lookup_pose(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.5))
        x, y, z = self._apply_transform(transform, 0.0, 0.0, 0.0)
        q = transform.transform.rotation
        return x, y, z, q

    def cb_rgb(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "RGB conversion failed: %s", exc)

    def cb_depth(self, msg):
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Depth conversion failed: %s", exc)

    def cb_info(self, msg):
        self.info = msg

    def _find_target(self, rgb_img):
        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.chair_hsv_lower, self.chair_hsv_upper)
        mask = cv2.medianBlur(mask, 5)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, mask

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area:
            return None, mask

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return None, mask

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        filled_mask = np.zeros_like(mask)
        cv2.drawContours(filled_mask, [contour], -1, 255, thickness=cv2.FILLED)
        return {
            "contour": contour,
            "center": (cx, cy),
            "area": area,
            "mask": mask,
            "filled_mask": filled_mask,
        }, mask

    def _sample_depth(self, depth_img, center_xy=None, target_mask=None):
        depth_m = self._depth_to_meters(depth_img)
        h_d, w_d = depth_m.shape[:2]
        if center_xy is None:
            center_xy = (w_d // 2, h_d // 2)

        cx, cy = center_xy
        if self.rgb is not None:
            h_rgb, w_rgb = self.rgb.shape[:2]
        else:
            h_rgb, w_rgb = h_d, w_d

        if w_rgb != w_d or h_rgb != h_d:
            x = int(np.clip(cx * (w_d / float(w_rgb)), 0, w_d - 1))
            y = int(np.clip(cy * (h_d / float(h_rgb)), 0, h_d - 1))
        else:
            x = int(np.clip(cx, 0, w_d - 1))
            y = int(np.clip(cy, 0, h_d - 1))

        if target_mask is not None:
            if target_mask.shape[:2] != depth_m.shape[:2]:
                target_mask = cv2.resize(
                    target_mask,
                    (w_d, h_d),
                    interpolation=cv2.INTER_NEAREST,
                )
            values = depth_m[target_mask > 0]
            values = values[np.isfinite(values)]
            values = values[(values >= self.min_depth) & (values <= self.max_depth)]
            if values.size > 0:
                return float(np.median(values)), (x, y)

        def candidate_values(radius):
            x0 = max(0, x - radius)
            x1 = min(w_d, x + radius + 1)
            y0 = max(0, y - radius)
            y1 = min(h_d, y + radius + 1)
            window = depth_m[y0:y1, x0:x1]
            window = window[np.isfinite(window)]
            window = window[(window >= self.min_depth) & (window <= self.max_depth)]
            return window

        for radius in (1, 3, 5, 7, 9, 11):
            values = candidate_values(radius)
            if values.size > 0:
                return float(np.median(values)), (x, y)

        depth = float(depth_m[y, x])
        if not np.isfinite(depth) or depth < self.min_depth or depth > self.max_depth:
            return None, (x, y)
        return depth, (x, y)

    def _publish_marker(self, pub, frame_id, marker_id, x, y, z, color, scale=0.12):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = frame_id
        marker.ns = "rgbd_blue_chair_nav"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.4)
        pub.publish(marker)

    def _make_goal(self, chair_map, robot_map):
        vx = chair_map[0] - robot_map[0]
        vy = chair_map[1] - robot_map[1]
        dist = math.hypot(vx, vy)
        if dist < 1e-6:
            return None

        if dist > self.approach_distance:
            ux = vx / dist
            uy = vy / dist
            goal_x = chair_map[0] - ux * self.approach_distance
            goal_y = chair_map[1] - uy * self.approach_distance
        else:
            goal_x = chair_map[0]
            goal_y = chair_map[1]

        yaw = math.atan2(chair_map[1] - goal_y, chair_map[0] - goal_x)
        return goal_x, goal_y, yaw

    def _send_goal(self, goal_x, goal_y, goal_yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(goal_x)
        goal.target_pose.pose.position.y = float(goal_y)
        goal.target_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, goal_yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        self.client.send_goal(goal)
        self.active_goal = True
        self.last_goal_xy = (float(goal_x), float(goal_y))
        rospy.loginfo("Sent navigation goal: x=%.3f y=%.3f yaw=%.3f", goal_x, goal_y, goal_yaw)

    @staticmethod
    def _clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def _visual_approach_cmd(self, target, frame_shape):
        x, y, w, h = cv2.boundingRect(target["contour"])
        fw = float(frame_shape[1])
        fh = float(frame_shape[0])
        target_cx = x + 0.5 * w
        frame_cx = 0.5 * fw
        center_error = (target_cx - frame_cx) / max(frame_cx, 1.0)
        area_ratio = float((w * h) / max(fw * fh, 1.0))
        area_error = self.visual_target_area_ratio - area_ratio

        twist = Twist()
        if abs(center_error) >= self.visual_center_deadband:
            twist.angular.z = self._clamp(
                self.visual_steering_sign * self.visual_center_kp * center_error,
                -self.visual_max_angular_speed,
                self.visual_max_angular_speed,
            )
        if abs(area_error) >= self.visual_area_deadband:
            twist.linear.x = self._clamp(
                self.visual_distance_kp * area_error,
                0.0,
                self.visual_max_linear_speed,
            )
        if abs(center_error) > 0.35:
            twist.linear.x *= 0.5
        return twist, center_error, area_ratio

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            nav_state = self.client.get_state()
            self.active_goal = nav_state in (
                GoalStatus.PENDING,
                GoalStatus.ACTIVE,
                GoalStatus.PREEMPTING,
                GoalStatus.RECALLING,
            )

            if self.rgb is None or self.depth is None or self.info is None:
                rate.sleep()
                continue

            target, mask = self._find_target(self.rgb)
            annotated = self.rgb.copy()
            info = {
                "target_found": False,
                "navigating": self.active_goal,
                "frame_id": self.map_frame,
            }

            if target is None:
                self.cmd_pub.publish(Twist())
                self.chair_info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                annotated_msg.header.stamp = rospy.Time.now()
                annotated_msg.header.frame_id = self.camera_frame
                self.annotated_pub.publish(annotated_msg)
                if self.show_debug:
                    cv2.imshow("blue_chair_mask", mask)
                    cv2.imshow("blue_chair_annotated", annotated)
                    cv2.waitKey(1)
                rate.sleep()
                continue

            cx, cy = target["center"]
            bx, by, bw, bh = cv2.boundingRect(target["contour"])
            depth, (dx, dy) = self._sample_depth(self.depth, (cx, cy), target.get("filled_mask"))
            cv2.drawContours(annotated, [target["contour"]], -1, (255, 180, 0), 2)
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh), (255, 120, 0), 2)
            cv2.circle(annotated, (cx, cy), 8, (0, 255, 0), 2)
            cv2.putText(
                annotated,
                f"blue chair area={target['area']:.1f}",
                (bx, max(20, by - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if depth is None:
                self.client.cancel_goal()
                self.active_goal = False
                visual_twist, center_error, area_ratio = self._visual_approach_cmd(target, self.rgb.shape)
                self.cmd_pub.publish(visual_twist)
                info.update(
                    {
                        "target_found": True,
                        "target_visible": True,
                        "center_pixel": {"x": int(cx), "y": int(cy)},
                        "depth_pixel": {"x": int(dx), "y": int(dy)},
                        "area": float(target["area"]),
                        "distance_camera": None,
                        "distance_map": None,
                        "visual_center_error": float(center_error),
                        "visual_area_ratio": float(area_ratio),
                        "visual_cmd": {
                            "linear_x": float(visual_twist.linear.x),
                            "angular_z": float(visual_twist.angular.z),
                        },
                    }
                )
                cv2.putText(
                    annotated,
                    f"DEPTH INVALID fallback v={visual_twist.linear.x:.2f} w={visual_twist.angular.z:.2f}",
                    (bx, min(annotated.shape[0] - 12, by + bh + 22)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                rospy.loginfo_throttle(3.0, "Blue chair detected, but local depth is invalid.")
            else:
                self.cmd_pub.publish(Twist())
                fx = float(self.info.K[0])
                fy = float(self.info.K[4])
                cx0 = float(self.info.K[2])
                cy0 = float(self.info.K[5])
                x_cam = (cx - cx0) * depth / fx
                y_cam = (cy0 - cy) * depth / fy
                chair_camera_distance = float(math.sqrt(x_cam * x_cam + y_cam * y_cam + depth * depth))
                chair_map = None
                robot_map = None
                goal_map = None

                try:
                    chair_map = self._lookup_point(self.map_frame, self.info.header.frame_id or self.camera_frame, x_cam, y_cam, depth)
                    robot_map = self._lookup_pose(self.map_frame, self.base_frame)[:3]
                except Exception as exc:
                    rospy.logwarn_throttle(2.0, "TF lookup failed: %s", exc)

                if chair_map is not None and robot_map is not None:
                    robot_dist_map = math.hypot(chair_map[0] - robot_map[0], chair_map[1] - robot_map[1])
                    goal = self._make_goal(chair_map, robot_map)
                    if goal is not None:
                        goal_map = goal
                        if (
                            self.last_goal_xy is None
                            or math.hypot(goal_map[0] - self.last_goal_xy[0], goal_map[1] - self.last_goal_xy[1]) > self.resend_distance
                            or self.client.get_state() in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST)
                        ):
                            self.client.cancel_goal()
                            self._send_goal(*goal_map)

                    self._publish_marker(self.chair_marker_pub, self.map_frame, 0, chair_map[0], chair_map[1], 0.15, (0.0, 0.2, 1.0), scale=0.18)
                    if goal_map is not None:
                        self._publish_marker(self.goal_marker_pub, self.map_frame, 1, goal_map[0], goal_map[1], 0.12, (0.0, 1.0, 0.2), scale=0.14)

                    cv2.putText(
                        annotated,
                        f"cam={chair_camera_distance:.2f}m map={robot_dist_map:.2f}m",
                        (bx, min(annotated.shape[0] - 12, by + bh + 22)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

                    if self.last_distance is None or abs(robot_dist_map - self.last_distance) > 0.1:
                        rospy.loginfo(
                            "Blue chair distance: camera=%.2f m, map=%.2f m, chair_map=(%.2f, %.2f), goal_map=(%s)",
                            chair_camera_distance,
                            robot_dist_map,
                            chair_map[0],
                            chair_map[1],
                            "none" if goal_map is None else f"{goal_map[0]:.2f}, {goal_map[1]:.2f}, {goal_map[2]:.2f}",
                        )
                        self.last_distance = robot_dist_map

                    pt = PointStamped()
                    pt.header.stamp = rospy.Time.now()
                    pt.header.frame_id = self.map_frame
                    pt.point.x = float(chair_map[0])
                    pt.point.y = float(chair_map[1])
                    pt.point.z = float(chair_map[2])
                    self.chair_point_pub.publish(pt)

                    info.update(
                        {
                            "target_found": True,
                            "target_visible": True,
                            "center_pixel": {"x": int(cx), "y": int(cy)},
                            "depth_pixel": {"x": int(dx), "y": int(dy)},
                            "area": float(target["area"]),
                            "distance_camera": chair_camera_distance,
                            "distance_map": robot_dist_map,
                            "chair_map": {
                                "x": float(chair_map[0]),
                                "y": float(chair_map[1]),
                                "z": float(chair_map[2]),
                            },
                            "goal_map": None
                            if goal_map is None
                            else {
                                "x": float(goal_map[0]),
                                "y": float(goal_map[1]),
                                "yaw": float(goal_map[2]),
                            },
                            "navigation_state": int(nav_state),
                        }
                    )
                else:
                    info.update(
                        {
                            "target_found": True,
                            "target_visible": True,
                            "center_pixel": {"x": int(cx), "y": int(cy)},
                            "depth_pixel": {"x": int(dx), "y": int(dy)},
                            "area": float(target["area"]),
                            "distance_camera": chair_camera_distance,
                            "distance_map": None,
                            "chair_map": None,
                            "goal_map": None,
                        }
                    )

                self._publish_marker(self.chair_marker_pub, self.camera_frame, 0, x_cam, y_cam, depth, (0.0, 0.2, 1.0), scale=0.12)

            nav_state = self.client.get_state()
            self.active_goal = nav_state in (
                GoalStatus.PENDING,
                GoalStatus.ACTIVE,
                GoalStatus.PREEMPTING,
                GoalStatus.RECALLING,
            )
            info["navigation_state"] = int(nav_state)
            info["navigating"] = self.active_goal
            self.chair_info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))

            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header.stamp = rospy.Time.now()
            annotated_msg.header.frame_id = self.camera_frame
            self.annotated_pub.publish(annotated_msg)

            if self.show_debug:
                cv2.imshow("blue_chair_mask", mask)
                cv2.imshow("blue_chair_annotated", annotated)
                cv2.waitKey(1)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rgbd_blue_chair_nav")
    RGBDBlueChairNavigator().run()
