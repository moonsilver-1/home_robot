#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Image preprocessing node for ROS Noetic."""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

try:
    import cv2
except ImportError as exc:
    raise SystemExit(
        "OpenCV is not installed. Install python3-opencv and ros-noetic-cv-bridge."
    ) from exc


def _parse_int_list(value, default):
    try:
        if isinstance(value, str):
            value = value.strip().strip("[]")
            return [int(x.strip()) for x in value.split(",") if x.strip()]
        return [int(x) for x in value]
    except Exception:
        return list(default)


class ImagePreprocessNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.input_topic = rospy.get_param("~input_image_topic", "/camera/rgb/image_raw")
        self.gray_topic = rospy.get_param("~gray_topic", "~gray")
        self.blur_topic = rospy.get_param("~blur_topic", "~blur")
        self.edge_topic = rospy.get_param("~edge_topic", "~edge")
        self.hsv_mask_topic = rospy.get_param("~hsv_mask_topic", "~hsv_mask")
        self.canny_low = int(rospy.get_param("~canny_low", 60))
        self.canny_high = int(rospy.get_param("~canny_high", 150))
        self.blur_kernel = int(rospy.get_param("~blur_kernel", 5))
        if self.blur_kernel < 1:
            self.blur_kernel = 5
        if self.blur_kernel % 2 == 0:
            self.blur_kernel += 1
        self.hsv_lower = _parse_int_list(rospy.get_param("~hsv_lower", [0, 80, 60]), [0, 80, 60])
        self.hsv_upper = _parse_int_list(rospy.get_param("~hsv_upper", [10, 255, 255]), [10, 255, 255])

        self.gray_pub = rospy.Publisher(self.gray_topic, Image, queue_size=1)
        self.blur_pub = rospy.Publisher(self.blur_topic, Image, queue_size=1)
        self.edge_pub = rospy.Publisher(self.edge_topic, Image, queue_size=1)
        self.mask_pub = rospy.Publisher(self.hsv_mask_topic, Image, queue_size=1)

        self.sub = rospy.Subscriber(
            self.input_topic, Image, self._callback, queue_size=1, buff_size=2**24
        )
        rospy.loginfo(
            "image_preprocess_node listening on %s and publishing %s, %s, %s, %s",
            self.input_topic,
            self.gray_topic,
            self.blur_topic,
            self.edge_topic,
            self.hsv_mask_topic,
        )

    def _publish_image(self, publisher, image, encoding, header):
        msg = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
        msg.header = header
        publisher.publish(msg)

    def _callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "cv_bridge conversion failed: %s", exc)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (self.blur_kernel, self.blur_kernel), 0)
        edges = cv2.Canny(blur, self.canny_low, self.canny_high)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, tuple(self.hsv_lower), tuple(self.hsv_upper))

        header = msg.header
        self._publish_image(self.gray_pub, gray, "mono8", header)
        self._publish_image(self.blur_pub, blur, "mono8", header)
        self._publish_image(self.edge_pub, edges, "mono8", header)
        self._publish_image(self.mask_pub, mask, "mono8", header)


def main():
    rospy.init_node("image_preprocess_node")
    ImagePreprocessNode()
    rospy.spin()


if __name__ == "__main__":
    main()
