#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from pathlib import Path

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class RGBDTargetDistanceNode:
    def __init__(self):
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/depth/camera_info")
        self.frame_id = rospy.get_param("~frame_id", "camera_rgb_optical_frame")

        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.min_area = float(rospy.get_param("~min_area", 500.0))
        self.min_depth = float(rospy.get_param("~min_depth", 0.1))
        self.max_depth = float(rospy.get_param("~max_depth", 10.0))
        self.contour_mode = rospy.get_param("~contour_mode", "largest")
        self.missing_depth_value = float(rospy.get_param("~missing_depth_value", -1.0))
        self.show_debug = self._as_bool(rospy.get_param("~show_debug", False))

        self.target_hsv_lower = self._parse_hsv_param("~target_hsv_lower", [90, 25, 20])
        self.target_hsv_upper = self._parse_hsv_param("~target_hsv_upper", [140, 255, 220])

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.info = None
        self.last_distance = None

        self.target_point_pub = rospy.Publisher("~target_point", PointStamped, queue_size=1)
        self.target_info_pub = rospy.Publisher("~target_info", String, queue_size=10)
        self.annotated_pub = rospy.Publisher("~annotated_image", Image, queue_size=1)
        self.marker_pub = rospy.Publisher("~target_marker", Marker, queue_size=1)

        self.sub_rgb = rospy.Subscriber(self.rgb_topic, Image, self.cb_rgb, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1, buff_size=2**24)
        self.sub_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("RGB-D target distance node started.")
        rospy.loginfo("RGB topic: %s", self.rgb_topic)
        rospy.loginfo("Depth topic: %s", self.depth_topic)
        rospy.loginfo("Camera info topic: %s", self.camera_info_topic)
        rospy.loginfo("Frame id: %s", self.frame_id)
        rospy.loginfo("HSV lower: %s", self.target_hsv_lower.tolist())
        rospy.loginfo("HSV upper: %s", self.target_hsv_upper.tolist())

    def on_shutdown(self):
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

    def _publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.info.header.frame_id or self.frame_id
        marker.ns = "rgbd_target_distance"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.2)
        self.marker_pub.publish(marker)

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

    @staticmethod
    def _depth_to_meters(depth_img):
        depth = np.asarray(depth_img)
        if depth.dtype == np.uint16 or np.nanmax(depth) > 50.0:
            return depth.astype(np.float32) / 1000.0
        return depth.astype(np.float32)

    def _find_target(self, rgb_img):
        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.target_hsv_lower, self.target_hsv_upper)
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
        return {
            "contour": contour,
            "center": (cx, cy),
            "area": area,
            "mask": mask,
        }, mask

    def _sample_depth(self, depth_img, center_xy):
        depth_m = self._depth_to_meters(depth_img)
        h_d, w_d = depth_m.shape[:2]
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

        depth = float(depth_m[y, x])
        if not np.isfinite(depth) or depth < self.min_depth or depth > self.max_depth:
            return None, (x, y)
        return depth, (x, y)

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            if self.rgb is None or self.depth is None or self.info is None:
                rate.sleep()
                continue

            target, mask = self._find_target(self.rgb)
            annotated = self.rgb.copy()
            info = {
                "target_found": False,
                "frame_id": self.frame_id,
            }

            if target is not None:
                (cx, cy) = target["center"]
                depth, (dx, dy) = self._sample_depth(self.depth, (cx, cy))

                cv2.circle(annotated, (cx, cy), 8, (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    f"area={target['area']:.1f}",
                    (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                if depth is not None:
                    fx = float(self.info.K[0])
                    fy = float(self.info.K[4])
                    cx0 = float(self.info.K[2])
                    cy0 = float(self.info.K[5])

                    x = (cx - cx0) * depth / fx
                    y = (cy0 - cy) * depth / fy
                    distance = float(np.sqrt(x * x + y * y + depth * depth))

                    pt = PointStamped()
                    pt.header.stamp = rospy.Time.now()
                    pt.header.frame_id = self.info.header.frame_id or self.frame_id
                    pt.point.x = float(x)
                    pt.point.y = float(y)
                    pt.point.z = float(depth)
                    self.target_point_pub.publish(pt)
                    self._publish_marker(x, y, depth)

                    if self.last_distance is None or abs(distance - self.last_distance) > 0.1:
                        rospy.loginfo(
                            "Target distance: %.2f m at pixel (%d, %d), depth pixel (%d, %d)",
                            distance,
                            cx,
                            cy,
                            dx,
                            dy,
                        )
                        self.last_distance = distance

                    info.update(
                        {
                            "target_found": True,
                            "center_pixel": {"x": int(cx), "y": int(cy)},
                            "depth_pixel": {"x": int(dx), "y": int(dy)},
                            "position": {
                                "x": float(x),
                                "y": float(y),
                                "z": float(depth),
                            },
                            "distance": distance,
                            "area": float(target["area"]),
                        }
                    )

                    cv2.putText(
                        annotated,
                        f"d={distance:.2f}m",
                        (cx + 10, cy + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                else:
                    info.update(
                        {
                            "target_found": True,
                            "center_pixel": {"x": int(cx), "y": int(cy)},
                            "depth_pixel": {"x": int(dx), "y": int(dy)},
                            "distance": self.missing_depth_value,
                            "area": float(target["area"]),
                        }
                    )

            else:
                rospy.loginfo_throttle(3.0, "No target detected")

            msg = String(data=json.dumps(info, ensure_ascii=False))
            self.target_info_pub.publish(msg)

            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header.stamp = rospy.Time.now()
            annotated_msg.header.frame_id = self.info.header.frame_id or self.frame_id
            self.annotated_pub.publish(annotated_msg)

            if self.show_debug:
                debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                cv2.imshow("rgbd_target_mask", debug)
                cv2.imshow("rgbd_target_annotated", annotated)
                cv2.waitKey(1)

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rgbd_target_distance")
    RGBDTargetDistanceNode().run()
