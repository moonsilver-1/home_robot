#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ArUco detector node for ROS Noetic."""

import json

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

try:
    import cv2
    import numpy as np
except ImportError as exc:
    raise SystemExit("OpenCV and numpy are required. Install python3-opencv and python3-numpy.") from exc


def _dictionary_id(dictionary_name):
    if not hasattr(cv2, "aruco"):
        return None
    name = str(dictionary_name).strip()
    if hasattr(cv2.aruco, name):
        return getattr(cv2.aruco, name)
    return getattr(cv2.aruco, "DICT_4X4_50")


def _make_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


class ArucoMarkerDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.input_topic = rospy.get_param("~input_image_topic", "/camera/rgb/image_raw")
        self.debug_topic = rospy.get_param("~debug_image_topic", "~debug_image")
        self.marker_topic = rospy.get_param("~marker_topic", "~markers")
        self.aruco_dictionary = rospy.get_param("~aruco_dictionary", "DICT_4X4_50")
        self.camera_frame = rospy.get_param("~camera_frame", "")

        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, String, queue_size=10)

        self.aruco_ready = hasattr(cv2, "aruco")
        self.detector = None
        self.parameters = None
        self.dictionary = None
        if self.aruco_ready:
            dict_id = _dictionary_id(self.aruco_dictionary)
            if dict_id is None:
                self.aruco_ready = False
            else:
                self.dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
                self.parameters = _make_detector_parameters()
                if hasattr(cv2.aruco, "ArucoDetector"):
                    self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        else:
            rospy.logwarn_throttle(5.0, "cv2.aruco is not available in this OpenCV build.")

        rospy.Subscriber(
            self.input_topic, Image, self._callback, queue_size=1, buff_size=2**24
        )
        rospy.loginfo(
            "aruco_marker_detector listening on %s and publishing %s, %s",
            self.input_topic,
            self.debug_topic,
            self.marker_topic,
        )

    def _detect(self, gray):
        if not self.aruco_ready:
            return [], None
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
            return corners, ids
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )
        return corners, ids

    def _callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        output = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        detections = []

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(output, corners, ids)
            for marker_id, pts in zip(ids.flatten().tolist(), corners):
                pts_array = np.asarray(pts[0], dtype=float)
                center_x = float(pts_array[:, 0].mean())
                center_y = float(pts_array[:, 1].mean())
                detections.append(
                    {
                        "id": int(marker_id),
                        "center": [center_x, center_y],
                        "corners": pts_array.tolist(),
                    }
                )

        marker_msg = {
            "stamp": msg.header.stamp.to_sec() if msg.header.stamp else 0.0,
            "frame_id": msg.header.frame_id or self.camera_frame,
            "markers": detections,
        }
        self.marker_pub.publish(String(data=json.dumps(marker_msg, ensure_ascii=False)))

        try:
            debug = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to publish debug image: %s", exc)
            return
        debug.header = msg.header
        if not debug.header.frame_id and self.camera_frame:
            debug.header.frame_id = self.camera_frame
        self.debug_pub.publish(debug)


def main():
    rospy.init_node("aruco_marker_detector")
    ArucoMarkerDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
