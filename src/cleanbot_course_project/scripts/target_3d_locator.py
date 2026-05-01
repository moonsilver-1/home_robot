#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RGB-D target locator node for ROS Noetic."""

import json
import math
import threading

import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker

try:
    import numpy as np
except ImportError as exc:
    raise SystemExit("numpy is required. Install python3-numpy.") from exc


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return float(default)


class Target3DLocator:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.depth_image = None
        self.depth_header = None
        self.camera_info = None

        self.detection_topic = rospy.get_param("~detection_topic", "/yolo/detections_json")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/depth/camera_info")
        self.target_class = rospy.get_param("~target_class", "person")
        self.point_topic = rospy.get_param("~point_topic", "~target_point")
        self.marker_topic = rospy.get_param("~marker_topic", "~target_marker")
        self.depth_window_size = int(rospy.get_param("~depth_window_size", 5))
        self.camera_frame = rospy.get_param("~camera_frame", "")
        self.depth_scale = float(rospy.get_param("~depth_scale", 1.0))

        self.point_pub = rospy.Publisher(self.point_topic, PointStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)

        rospy.Subscriber(self.depth_topic, Image, self._depth_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self._info_cb, queue_size=1)
        rospy.Subscriber(self.detection_topic, String, self._det_cb, queue_size=1)

        rospy.loginfo(
            "target_3d_locator listening on %s, %s, %s",
            self.detection_topic,
            self.depth_topic,
            self.camera_info_topic,
        )

    def _depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert depth image: %s", exc)
            return

        depth_array = np.asarray(depth)
        if depth_array.dtype.kind in ("u", "i"):
            depth_array = depth_array.astype(np.float32) * 0.001
        else:
            depth_array = depth_array.astype(np.float32)
        if self.depth_scale != 1.0:
            depth_array = depth_array * self.depth_scale

        with self.lock:
            self.depth_image = depth_array
            self.depth_header = msg.header

    def _info_cb(self, msg):
        with self.lock:
            self.camera_info = msg

    def _select_detection(self, data):
        candidates = []
        for det in data.get("detections", []):
            class_name = det.get("class", det.get("name", ""))
            bbox = det.get("bbox")
            if class_name != self.target_class or not bbox or len(bbox) != 4:
                continue
            confidence = _safe_float(det.get("confidence", det.get("score", 0.0)))
            x1, y1, x2, y2 = [_safe_float(v) for v in bbox]
            candidates.append(
                {
                    "confidence": confidence,
                    "bbox": [x1, y1, x2, y2],
                }
            )

        for det in data.get("objects", []):
            class_name = det.get("class", det.get("name", ""))
            if class_name != self.target_class:
                continue
            confidence = _safe_float(det.get("confidence", det.get("score", 0.0)))
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
                }
            )

        if not candidates:
            return None
        return max(candidates, key=lambda item: item["confidence"])

    def _depth_window(self, depth_image, u, v):
        radius = max(0, int(self.depth_window_size))
        height, width = depth_image.shape[:2]
        x1 = max(0, u - radius)
        x2 = min(width, u + radius + 1)
        y1 = max(0, v - radius)
        y2 = min(height, v + radius + 1)
        window = depth_image[y1:y2, x1:x2].astype(np.float32)
        window = window[np.isfinite(window)]
        window = window[window > 0.0]
        if window.size == 0:
            return None
        return float(np.mean(window))

    def _publish_marker(self, header, point):
        marker = Marker()
        marker.header = header
        marker.ns = "target_3d"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.12
        marker.color.a = 0.95
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.1
        self.marker_pub.publish(marker)

    def _det_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse detection JSON: %s", exc)
            return

        detection = self._select_detection(data)
        if detection is None:
            return

        with self.lock:
            depth_image = self.depth_image
            depth_header = self.depth_header
            camera_info = self.camera_info

        if depth_image is None:
            rospy.logwarn_throttle(2.0, "Waiting for depth image...")
            return
        if camera_info is None:
            rospy.logwarn_throttle(2.0, "Waiting for camera_info...")
            return

        bbox = detection["bbox"]
        u = int(round(0.5 * (bbox[0] + bbox[2])))
        v = int(round(0.5 * (bbox[1] + bbox[3])))
        height, width = depth_image.shape[:2]
        if u < 0 or v < 0 or u >= width or v >= height:
            return

        z = self._depth_window(depth_image, u, v)
        if z is None or not math.isfinite(z) or z <= 0.0:
            rospy.logwarn_throttle(2.0, "No valid depth around target center.")
            return

        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        cx = float(camera_info.K[2])
        cy = float(camera_info.K[5])
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        point = PointStamped()
        point.header = depth_header or msg.header
        if not point.header.frame_id:
            point.header.frame_id = self.camera_frame or (camera_info.header.frame_id or "camera_link")
        point.point.x = x
        point.point.y = y
        point.point.z = z
        self.point_pub.publish(point)
        self._publish_marker(point.header, point)


def main():
    rospy.init_node("target_3d_locator")
    Target3DLocator()
    rospy.spin()


if __name__ == "__main__":
    main()
