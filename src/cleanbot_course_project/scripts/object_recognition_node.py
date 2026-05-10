#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ObjectRecognitionNode:
    """
    CleanBot OpenCV object recognition node.

    Targets:
        1. blue_ball
        2. chair

    Strategy:
        1. Template matching first.
        2. If template matching fails, use HSV + morphology fallback.
        3. Publish annotated image and JSON detection results.
    """

    VALID_CLASSES = ["blue_ball", "chair"]

    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.template_dir = rospy.get_param(
            "~template_dir",
            os.path.expanduser("~/home_robot/datasets/object_templates")
        )

        self.template_min_score = float(rospy.get_param("~template_min_score", 0.55))
        self.min_area = int(rospy.get_param("~min_area", 300))
        self.debug_view = bool(rospy.get_param("~debug_view", False))
        self.enable_color_fallback = bool(rospy.get_param("~enable_color_fallback", True))

        self.template_scales = rospy.get_param(
            "~template_scales",
            [0.60, 0.75, 0.90, 1.00, 1.15, 1.30]
        )

        # Blue ball HSV fallback.
        self.blue_hsv_lower = np.array(
            rospy.get_param("~blue_hsv_lower", [88, 45, 30]),
            dtype=np.uint8
        )
        self.blue_hsv_upper = np.array(
            rospy.get_param("~blue_hsv_upper", [108, 255, 255]),
            dtype=np.uint8
        )

        # Deep-blue chair HSV fallback.
        self.chair_hsv_lower = np.array(
            rospy.get_param("~chair_hsv_lower", [100, 80, 20]),
            dtype=np.uint8
        )
        self.chair_hsv_upper = np.array(
            rospy.get_param("~chair_hsv_upper", [130, 255, 170]),
            dtype=np.uint8
        )

        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher(
            "/cleanbot/vision/annotated_image",
            Image,
            queue_size=1
        )

        self.det_pub = rospy.Publisher(
            "/cleanbot/vision/detections",
            String,
            queue_size=10
        )

        self.templates = self.load_templates()

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24
        )

        rospy.loginfo("Object recognition node started.")
        rospy.loginfo("Image topic: %s", self.image_topic)
        rospy.loginfo("Template dir: %s", self.template_dir)
        rospy.loginfo("Valid classes: %s", self.VALID_CLASSES)
        rospy.loginfo("Loaded template classes: %s", list(self.templates.keys()))
        rospy.loginfo("Template min score: %.3f", self.template_min_score)
        rospy.loginfo("Color fallback enabled: %s", self.enable_color_fallback)

    def preprocess_gray(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        return gray

    def load_templates(self):
        templates = {}

        if not os.path.isdir(self.template_dir):
            rospy.logwarn("Template dir does not exist: %s", self.template_dir)
            return templates

        for class_name in sorted(os.listdir(self.template_dir)):
            if class_name not in self.VALID_CLASSES:
                rospy.loginfo("Ignore template folder: %s", class_name)
                continue

            class_dir = os.path.join(self.template_dir, class_name)
            if not os.path.isdir(class_dir):
                continue

            templates[class_name] = []

            for filename in sorted(os.listdir(class_dir)):
                if not filename.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                    continue

                path = os.path.join(class_dir, filename)
                img = cv2.imread(path)

                if img is None:
                    rospy.logwarn("Failed to read template: %s", path)
                    continue

                if img.shape[0] < 12 or img.shape[1] < 12:
                    rospy.logwarn("Template too small, skipped: %s", path)
                    continue

                gray = self.preprocess_gray(img)
                h, w = gray.shape[:2]

                templates[class_name].append({
                    "class_name": class_name,
                    "path": path,
                    "image": img,
                    "gray": gray,
                    "size": (w, h)
                })

            rospy.loginfo(
                "Loaded %d templates for class [%s]",
                len(templates[class_name]),
                class_name
            )

        return {k: v for k, v in templates.items() if len(v) > 0}

    def detect_by_template_matching(self, frame):
        detections = []

        if not self.templates:
            return detections

        frame_gray = self.preprocess_gray(frame)
        fh, fw = frame_gray.shape[:2]

        for class_name in self.VALID_CLASSES:
            if class_name not in self.templates:
                continue

            best_det = None

            for tpl in self.templates[class_name]:
                tpl_gray = tpl["gray"]
                th0, tw0 = tpl_gray.shape[:2]

                for scale in self.template_scales:
                    tw = int(tw0 * float(scale))
                    th = int(th0 * float(scale))

                    if tw < 12 or th < 12:
                        continue
                    if tw >= fw or th >= fh:
                        continue

                    resized_tpl = cv2.resize(tpl_gray, (tw, th))

                    result = cv2.matchTemplate(
                        frame_gray,
                        resized_tpl,
                        cv2.TM_CCOEFF_NORMED
                    )

                    _, max_val, _, max_loc = cv2.minMaxLoc(result)

                    if max_val < self.template_min_score:
                        continue

                    x, y = max_loc

                    det = {
                        "label": class_name,
                        "bbox": [int(x), int(y), int(tw), int(th)],
                        "confidence": float(max_val),
                        "method": "template_matching",
                        "template_path": tpl["path"],
                        "scale": float(scale)
                    }

                    if best_det is None or det["confidence"] > best_det["confidence"]:
                        best_det = det

            if best_det is not None:
                detections.append(best_det)

        return detections

    def detect_blue_ball_by_color(self, frame):
        """
        HSV + morphology fallback for blue ball.
        Add circle/arc-shape constraints to avoid confusing light-blue trash bin.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.blue_hsv_lower, self.blue_hsv_upper)
        mask = cv2.medianBlur(mask, 5)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w < 12 or h < 12:
                continue

            # 1. 外接框要接近正方形
            aspect = w / float(h + 1e-6)

            # 2. 圆度：越接近 1 越像圆
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 1e-6:
                continue
            circularity = 4.0 * np.pi * area / (perimeter * perimeter)

            # 3. 与最小外接圆的填充程度
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            if radius < 5:
                continue
            circle_area = np.pi * radius * radius
            fill_ratio = area / float(circle_area + 1e-6)

            # 4. 凸包实心度，球通常更饱满，垃圾桶边缘更容易偏低
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = area / float(hull_area + 1e-6)

            # 5. 轮廓近似点数：圆弧目标通常点数较多，矩形/桶状会更少
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            approx_vertices = len(approx)

            # 6. 外接框填充度
            bbox_fill = area / float(w * h + 1e-6)

            # 强化“像球”的条件
            if (
                0.75 <= aspect <= 1.30 and
                circularity > 0.55 and
                fill_ratio > 0.60 and
                solidity > 0.90 and
                approx_vertices >= 8 and
                bbox_fill > 0.55
            ):
                confidence = (
                    0.30 * min(1.0, circularity) +
                    0.25 * min(1.0, fill_ratio) +
                    0.20 * min(1.0, solidity) +
                    0.15 * min(1.0, bbox_fill) +
                    0.10 * min(1.0, approx_vertices / 16.0)
                )

                detections.append({
                    "label": "blue_ball",
                    "bbox": [int(x), int(y), int(w), int(h)],
                    "confidence": float(min(1.0, confidence)),
                    "method": "color_morphology_circle_fallback"
                })

        detections = sorted(detections, key=lambda d: d["confidence"], reverse=True)
        return detections[:1]
        
    def detect_chair_by_color(self, frame):
        """
        HSV + morphology fallback for deep-blue chair.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.chair_hsv_lower, self.chair_hsv_upper)
        mask = cv2.medianBlur(mask, 5)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        detections = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < max(self.min_area, 600):
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if w < 20 or h < 20:
                continue

            aspect = w / float(h + 1e-6)
            bbox_fill = area / float(w * h + 1e-6)

            # Deep-blue chair usually appears as a relatively large but irregular region.
            if 0.40 <= aspect <= 2.4 and bbox_fill > 0.18:
                confidence = min(1.0, 0.40 + area / 6500.0 + 0.25 * bbox_fill)

                detections.append({
                    "label": "chair",
                    "bbox": [int(x), int(y), int(w), int(h)],
                    "confidence": float(confidence),
                    "method": "deep_blue_color_morphology_fallback"
                })

        detections = sorted(detections, key=lambda d: d["confidence"], reverse=True)
        return detections[:1]

    def bbox_iou(self, a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b

        ax2, ay2 = ax + aw, ay + ah
        bx2, by2 = bx + bw, by + bh

        ix1 = max(ax, bx)
        iy1 = max(ay, by)
        ix2 = min(ax2, bx2)
        iy2 = min(ay2, by2)

        iw = max(0, ix2 - ix1)
        ih = max(0, iy2 - iy1)

        inter = iw * ih
        union = aw * ah + bw * bh - inter + 1e-6

        return inter / union

    def nms(self, detections, iou_threshold=0.35):
        if not detections:
            return []

        detections = sorted(detections, key=lambda d: d["confidence"], reverse=True)
        keep = []

        for det in detections:
            should_keep = True

            for kept in keep:
                same_label = det["label"] == kept["label"]
                overlap = self.bbox_iou(det["bbox"], kept["bbox"]) > iou_threshold

                if same_label and overlap:
                    should_keep = False
                    break

            if should_keep:
                keep.append(det)

        return keep

    def has_label(self, detections, label):
        return any(det["label"] == label for det in detections)

    def draw_detections(self, frame, detections):
        out = frame.copy()

        if not detections:
            cv2.rectangle(out, (0, 0), (out.shape[1], 42), (0, 0, 0), -1)
            cv2.putText(
                out,
                "No target detected",
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
                cv2.LINE_AA
            )
            return out

        for det in detections:
            x, y, w, h = det["bbox"]
            label = det["label"]
            conf = det["confidence"]
            method = det["method"]

            if label == "blue_ball":
                color = (255, 120, 0)
            else:
                color = (0, 255, 0)

            cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)

            text = f"{label}: {conf:.2f} [{method}]"
            cv2.putText(
                out,
                text,
                (x, max(24, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA
            )

        return out

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Image conversion failed: %s", str(e))
            return

        detections = []

        template_detections = self.detect_by_template_matching(frame)
        detections.extend(template_detections)

        if self.enable_color_fallback:
            if not self.has_label(template_detections, "blue_ball"):
                detections.extend(self.detect_blue_ball_by_color(frame))

            if not self.has_label(template_detections, "chair"):
                detections.extend(self.detect_chair_by_color(frame))

        detections = self.nms(detections, iou_threshold=0.35)
        detections = sorted(detections, key=lambda d: d["confidence"], reverse=True)

        annotated = self.draw_detections(frame, detections)

        result = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "num_detections": len(detections),
            "detections": detections
        }

        self.det_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
        except Exception as e:
            rospy.logerr("Failed to publish annotated image: %s", str(e))

        if self.debug_view:
            cv2.imshow("CleanBot Object Recognition", annotated)
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("object_recognition_node")
    node = ObjectRecognitionNode()
    rospy.spin()
