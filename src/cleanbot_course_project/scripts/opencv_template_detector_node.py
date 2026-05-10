#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import cv2
import rospy
import numpy as np
from pathlib import Path
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class OpenCVTemplateDetector:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.template_dir = self.resolve_template_dir(Path(rospy.get_param(
            "~template_dir",
            str(Path.home() / "home_robot/datasets/object_templates")
        )))

        self.min_score = float(rospy.get_param("~min_score", 0.45))
        self.ratio_test = float(rospy.get_param("~ratio_test", 0.75))
        self.min_good_matches = int(rospy.get_param("~min_good_matches", 12))
        self.min_inliers = int(rospy.get_param("~min_inliers", 8))
        self.min_inlier_ratio = float(rospy.get_param("~min_inlier_ratio", 0.55))
        self.max_reproj_error = float(rospy.get_param("~max_reproj_error", 6.0))
        self.debug_view = bool(rospy.get_param("~debug_view", False))

        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=1200)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.templates = self.load_templates()

        self.result_pub = rospy.Publisher(
            "/cleanbot/vision/template_detections",
            String,
            queue_size=10
        )

        self.image_pub = rospy.Publisher(
            "/cleanbot/vision/template_detection_image",
            Image,
            queue_size=1
        )

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("OpenCV template detector started.")
        rospy.loginfo("Image topic: %s", self.image_topic)
        rospy.loginfo("Template dir: %s", str(self.template_dir))
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
                image_paths.extend(list(class_dir.glob(suffix)))

            for path in sorted(image_paths):
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

                templates[class_name].append({
                    "class_name": class_name,
                    "path": str(path),
                    "image": img,
                    "gray": gray,
                    "kp": kp,
                    "des": des,
                    "size": (img.shape[1], img.shape[0])
                })

            rospy.loginfo("Loaded %d templates for class [%s]",
                          len(templates[class_name]), class_name)

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
                    axis=1
                )
            )
        )

        if reproj_error > self.max_reproj_error:
            return None

        w, h = template["size"]
        corners = np.float32([
            [0, 0],
            [w, 0],
            [w, h],
            [0, h]
        ]).reshape(-1, 1, 2)

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
            "template_score": float(min(1.0, len(good) / 40.0)),
            "match_count": int(len(good)),
            "inliers": int(inlier_count),
            "inlier_ratio": float(inlier_ratio),
            "reproj_error": float(reproj_error),
            "template_path": template["path"],
            "method": "orb_template_homography"
        }

    def match_templates(self, frame):
        frame_gray = self.preprocess(frame)
        frame_kp, frame_des = self.orb.detectAndCompute(frame_gray, None)

        if frame_des is None or len(frame_kp) < 10:
            return []

        detections = []

        for class_name, template_list in self.templates.items():
            best_det = None

            for tpl in template_list:
                det = self.match_one_template(tpl, frame_kp, frame_des, frame.shape)
                if det is None or det["score"] < self.min_score:
                    continue

                det["label"] = class_name
                det["confidence"] = det["score"]

                if best_det is None or det["score"] > best_det["score"]:
                    best_det = det

            if best_det is not None:
                detections.append(best_det)

        detections = self.nms(detections, iou_threshold=0.35)
        detections = sorted(detections, key=lambda d: d["score"], reverse=True)
        return detections

    def nms(self, detections, iou_threshold=0.35):
        if not detections:
            return []

        boxes = np.array([d["bbox"] for d in detections], dtype=np.float32)
        scores = np.array([d["score"] for d in detections], dtype=np.float32)

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 0] + boxes[:, 2]
        y2 = boxes[:, 1] + boxes[:, 3]

        areas = boxes[:, 2] * boxes[:, 3]
        order = scores.argsort()[::-1]

        keep = []

        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)

            inter = w * h
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)

            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]

        return [detections[i] for i in keep]

    def draw(self, frame, detections):
        vis = frame.copy()

        if len(detections) == 0:
            text = "No target detected"
            color = (0, 0, 255)
        else:
            top = detections[0]
            text = f"{top['label']} score={top['score']:.2f}"
            color = (0, 255, 0)

        cv2.rectangle(vis, (0, 0), (vis.shape[1], 45), (0, 0, 0), -1)
        cv2.putText(
            vis,
            text,
            (15, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            color,
            2,
            cv2.LINE_AA
        )

        for det in detections:
            x, y, w, h = det["bbox"]
            label = det["label"]
            score = det["score"]

            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f"{label} {score:.2f}",
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )

        return vis

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("Image conversion failed: %s", str(e))
            return

        detections = self.match_templates(frame)
        vis = self.draw(frame, detections)

        result_msg = {
            "stamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "num_detections": len(detections),
            "detections": detections
        }

        self.result_pub.publish(String(data=json.dumps(result_msg, ensure_ascii=False)))

        out_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

        if self.debug_view:
            cv2.imshow("OpenCV Template Detection", vis)
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("opencv_template_detector_node")
    node = OpenCVTemplateDetector()
    rospy.spin()
