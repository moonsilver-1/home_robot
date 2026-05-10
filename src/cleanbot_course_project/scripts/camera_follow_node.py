#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from pathlib import Path

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraFollowNode:
    """Vision-based follower that tracks a target robot from the camera image."""

    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/follower/camera/rgb/image_raw")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/follower/cmd_vel")
        self.debug_view = bool(rospy.get_param("~debug_view", True))
        self.enable_color_tracking = bool(rospy.get_param("~enable_color_tracking", True))

        self.target_class = rospy.get_param("~target_class", "leader")
        self.template_dir = self.resolve_template_dir(
            Path(rospy.get_param("~template_dir", str(Path.home() / "home_robot/datasets/object_samples/leader")))
        )

        # ORB template matching parameters.
        self.min_score = float(rospy.get_param("~min_score", 0.45))
        self.ratio_test = float(rospy.get_param("~ratio_test", 0.75))
        self.min_good_matches = int(rospy.get_param("~min_good_matches", 12))
        self.min_inliers = int(rospy.get_param("~min_inliers", 8))
        self.min_inlier_ratio = float(rospy.get_param("~min_inlier_ratio", 0.55))
        self.max_reproj_error = float(rospy.get_param("~max_reproj_error", 6.0))
        self.template_scales = rospy.get_param("~template_scales", [0.70, 0.85, 1.00, 1.15, 1.30])
        self.max_templates = int(rospy.get_param("~max_templates", 20))

        # Control gains.
        self.center_deadband = float(rospy.get_param("~center_deadband", 0.08))
        self.area_deadband = float(rospy.get_param("~area_deadband", 0.015))
        self.target_area_ratio = float(rospy.get_param("~target_area_ratio", 0.055))
        self.center_kp = float(rospy.get_param("~center_kp", 1.20))
        self.distance_kp = float(rospy.get_param("~distance_kp", 3.20))
        self.max_linear_speed = float(rospy.get_param("~max_linear_speed", 0.18))
        self.max_angular_speed = float(rospy.get_param("~max_angular_speed", 1.00))
        self.search_angular_speed = float(rospy.get_param("~search_angular_speed", 0.35))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 0.9))
        self.control_rate = float(rospy.get_param("~control_rate", 15.0))
        self.min_area = int(rospy.get_param("~min_area", 450))
        self.green_hsv_lower_1 = np.array(rospy.get_param("~green_hsv_lower_1", [35, 60, 35]), dtype=np.uint8)
        self.green_hsv_upper_1 = np.array(rospy.get_param("~green_hsv_upper_1", [85, 255, 255]), dtype=np.uint8)
        self.green_hsv_lower_2 = np.array(rospy.get_param("~green_hsv_lower_2", [35, 60, 35]), dtype=np.uint8)
        self.green_hsv_upper_2 = np.array(rospy.get_param("~green_hsv_upper_2", [85, 255, 255]), dtype=np.uint8)

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=int(rospy.get_param("~orb_features", 1400)))
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self.templates = self.load_templates()

        self.latest_detection = None
        self.latest_detection_time = None

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/cleanbot/follow/status", String, queue_size=10)
        self.image_pub = rospy.Publisher("/cleanbot/follow/annotated_image", Image, queue_size=1)

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo("Camera follow node started.")
        rospy.loginfo("Image topic: %s", self.image_topic)
        rospy.loginfo("Cmd vel topic: %s", self.cmd_vel_topic)
        rospy.loginfo("Template dir: %s", str(self.template_dir))
        rospy.loginfo("Target class: %s", self.target_class)
        rospy.loginfo("Color tracking enabled: %s", self.enable_color_tracking)
        rospy.loginfo("Loaded template classes: %s", list(self.templates.keys()))

    def resolve_template_dir(self, configured_dir):
        candidates = [
            configured_dir,
            Path.home() / "home_robot/datasets/object_samples",
        ]

        for path in candidates:
            if path.exists() and path.is_dir():
                rospy.loginfo("Using template directory: %s", str(path))
                return path

        rospy.logwarn("No template directory found in candidates: %s", [str(p) for p in candidates])
        return configured_dir

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

    def preprocess(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        return gray

    def load_templates(self):
        templates = {}

        if not self.template_dir.exists():
            rospy.logwarn("Template directory does not exist: %s", str(self.template_dir))
            return templates

        for class_dir in sorted(self.template_dir.iterdir()):
            if not class_dir.is_dir():
                continue

            class_name = class_dir.name
            templates[class_name] = []

            image_paths = []
            for suffix in ["*.jpg", "*.jpeg", "*.png", "*.bmp"]:
                image_paths.extend(sorted(class_dir.glob(suffix)))

            for path in image_paths[: self.max_templates]:
                img = cv2.imread(str(path))
                if img is None:
                    rospy.logwarn("Failed to read template: %s", str(path))
                    continue

                if img.shape[0] < 15 or img.shape[1] < 15:
                    rospy.logwarn("Template too small, skipped: %s", str(path))
                    continue

                gray = self.preprocess(img)
                kp, des = self.orb.detectAndCompute(gray, None)
                if des is None or len(kp) < 8:
                    rospy.logwarn("Too few ORB features in template: %s", str(path))
                    continue

                templates[class_name].append(
                    {
                        "class_name": class_name,
                        "path": str(path),
                        "image": img,
                        "gray": gray,
                        "kp": kp,
                        "des": des,
                        "size": (img.shape[1], img.shape[0]),
                    }
                )

            rospy.loginfo("Loaded %d templates for class [%s]", len(templates[class_name]), class_name)

        return {k: v for k, v in templates.items() if len(v) > 0}

    def match_one_template(self, template, frame_kp, frame_des, frame_shape):
        matches = self.matcher.knnMatch(template["des"], frame_des, k=2)

        good = []
        for pair in matches:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < self.ratio_test * n.distance:
                good.append(m)

        if len(good) < self.min_good_matches:
            return None

        src_pts = np.float32([template["kp"][m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([frame_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        if H is None or mask is None:
            return None

        inlier_mask = mask.ravel().astype(bool)
        inlier_count = int(np.sum(inlier_mask))
        inlier_ratio = float(inlier_count / max(1, len(good)))

        if inlier_count < self.min_inliers or inlier_ratio < self.min_inlier_ratio:
            return None

        projected_inliers = cv2.perspectiveTransform(src_pts[inlier_mask], H)
        reproj_error = float(
            np.mean(
                np.linalg.norm(
                    projected_inliers[:, 0, :] - dst_pts[inlier_mask][:, 0, :],
                    axis=1,
                )
            )
        )

        if reproj_error > self.max_reproj_error:
            return None

        w, h = template["size"]
        corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        projected = cv2.perspectiveTransform(corners, H)
        xs = projected[:, 0, 0]
        ys = projected[:, 0, 1]

        x1, y1 = int(np.min(xs)), int(np.min(ys))
        x2, y2 = int(np.max(xs)), int(np.max(ys))

        fh, fw = frame_shape[:2]
        x1 = max(0, min(fw - 1, x1))
        y1 = max(0, min(fh - 1, y1))
        x2 = max(0, min(fw - 1, x2))
        y2 = max(0, min(fh - 1, y2))

        bw = x2 - x1
        bh = y2 - y1
        if bw < 15 or bh < 15:
            return None

        match_score = min(1.0, len(good) / 40.0)
        inlier_score = min(1.0, inlier_ratio)
        error_score = max(0.0, 1.0 - reproj_error / max(self.max_reproj_error, 1e-6))
        score = 0.40 * match_score + 0.35 * inlier_score + 0.25 * error_score

        return {
            "label": template["class_name"],
            "bbox": [int(x1), int(y1), int(bw), int(bh)],
            "score": float(min(1.0, score)),
            "match_count": int(len(good)),
            "inliers": int(inlier_count),
            "inlier_ratio": float(inlier_ratio),
            "reproj_error": float(reproj_error),
            "template_path": template["path"],
            "method": "orb_template_homography",
        }

    def match_templates(self, frame):
        if self.target_class not in self.templates:
            return []

        frame_gray = self.preprocess(frame)
        frame_kp, frame_des = self.orb.detectAndCompute(frame_gray, None)
        if frame_des is None or len(frame_kp) < 10:
            return []

        detections = []
        best_det = None

        for tpl in self.templates[self.target_class]:
            det = self.match_one_template(tpl, frame_kp, frame_des, frame.shape)
            if det is None or det["score"] < self.min_score:
                continue

            det["confidence"] = det["score"]
            if best_det is None or det["score"] > best_det["score"]:
                best_det = det

        if best_det is not None:
            detections.append(best_det)

        return detections

    def detect_green_target_by_color(self, frame):
        if not self.enable_color_tracking:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.green_hsv_lower_1, self.green_hsv_upper_1)
        mask2 = cv2.inRange(hsv, self.green_hsv_lower_2, self.green_hsv_upper_2)
        mask = cv2.bitwise_or(mask1, mask2)

        mask = cv2.medianBlur(mask, 5)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w < 15 or h < 15:
                continue

            aspect = w / float(h + 1e-6)
            bbox_fill = area / float(w * h + 1e-6)

            if 0.45 <= aspect <= 2.5 and bbox_fill > 0.20:
                confidence = min(1.0, 0.45 + area / 12000.0 + 0.25 * bbox_fill)
                detections.append(
                    {
                        "label": self.target_class,
                        "bbox": [int(x), int(y), int(w), int(h)],
                        "score": float(confidence),
                        "confidence": float(confidence),
                        "method": "green_color_mask",
                    }
                )

        detections = sorted(detections, key=lambda d: d["confidence"], reverse=True)
        return detections[:1]

    def compute_cmd(self, det, frame_shape):
        x, y, w, h = det["bbox"]
        fw = float(frame_shape[1])
        fh = float(frame_shape[0])

        target_cx = x + w * 0.5
        frame_cx = fw * 0.5
        center_error = (target_cx - frame_cx) / max(frame_cx, 1.0)

        area_ratio = float((w * h) / max(fw * fh, 1.0))
        area_error = self.target_area_ratio - area_ratio

        twist = Twist()

        if abs(center_error) >= self.center_deadband:
            twist.angular.z = self.clamp(-self.center_kp * center_error, -self.max_angular_speed, self.max_angular_speed)

        if abs(area_error) >= self.area_deadband:
            twist.linear.x = self.clamp(self.distance_kp * area_error, -self.max_linear_speed, self.max_linear_speed)

        # 如果目标偏得很厉害，优先转向，减少冲撞。
        if abs(center_error) > 0.55:
            twist.linear.x *= 0.25
        elif abs(center_error) > 0.30:
            twist.linear.x *= 0.55

        return twist, center_error, area_ratio

    @staticmethod
    def draw_crosshair(frame):
        vis = frame.copy()
        h, w = vis.shape[:2]
        cx = w // 2
        cy = h // 2
        cv2.line(vis, (cx, 0), (cx, h), (255, 255, 0), 1)
        cv2.line(vis, (0, cy), (w, cy), (255, 255, 0), 1)
        return vis

    def draw(self, frame, detections, twist, state, center_error=None, area_ratio=None):
        vis = self.draw_crosshair(frame)

        cv2.rectangle(vis, (0, 0), (vis.shape[1], 46), (0, 0, 0), -1)
        header = f"{state}"
        if detections:
            det = detections[0]
            header = f"{det['label']} score={det['score']:.2f} {state}"
        cv2.putText(vis, header, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

        if center_error is not None and area_ratio is not None:
            info = f"cx_err={center_error:.2f} area={area_ratio:.3f} v={twist.linear.x:.2f} w={twist.angular.z:.2f}"
            cv2.putText(vis, info, (12, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA)

        for det in detections:
            x, y, w, h = det["bbox"]
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f"{det['label']} {det['score']:.2f}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return vis

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_status(self, state, detection=None, twist=None, center_error=None, area_ratio=None):
        data = {
            "state": state,
            "image_topic": self.image_topic,
            "cmd_vel_topic": self.cmd_vel_topic,
            "target_class": self.target_class,
            "detection": detection,
            "center_error": center_error,
            "area_ratio": area_ratio,
            "linear_cmd": 0.0 if twist is None else twist.linear.x,
            "angular_cmd": 0.0 if twist is None else twist.angular.z,
        }
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logerr_throttle(2.0, "Image conversion failed: %s", exc)
            return

        detections = self.detect_green_target_by_color(frame)
        detection_source = "color"
        if not detections:
            detections = self.match_templates(frame)
            detection_source = "template"
        now = rospy.Time.now()

        if detections:
            det = detections[0]
            twist, center_error, area_ratio = self.compute_cmd(det, frame.shape)
            state = "FOLLOWING"
            self.latest_detection = det
            self.latest_detection_time = now
        else:
            twist = Twist()
            center_error = None
            area_ratio = None

            if self.latest_detection_time is None or (now - self.latest_detection_time).to_sec() > self.lost_timeout:
                twist.angular.z = self.search_angular_speed
                state = "SEARCHING"
            else:
                state = "HOLDING_LAST"

        self.cmd_pub.publish(twist)
        self.publish_status(
            state=state,
            detection=detections[0] if detections else None,
            twist=twist,
            center_error=center_error,
            area_ratio=area_ratio,
        )

        vis = self.draw(frame, detections, twist, state, center_error=center_error, area_ratio=area_ratio)
        if detections:
            cv2.putText(
                vis,
                f"source={detection_source}",
                (12, 95),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        out_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

        if self.debug_view:
            cv2.imshow("CleanBot Camera Follow", vis)
            cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("camera_follow_node")
    node = CameraFollowNode()
    node.run()