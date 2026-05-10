#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs import point_cloud2 as pcl2
from std_msgs.msg import Header, String


class RGBDPointCloudNode:
    def __init__(self):
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/depth/camera_info")
        self.frame_id = rospy.get_param("~frame_id", "camera_rgb_optical_frame")

        self.output_topic = rospy.get_param("~output_topic", "/pointcloud_output")
        self.target_cloud_topic = rospy.get_param("~target_cloud_topic", "/pointcloud_target_cloud")
        self.target_point_topic = rospy.get_param("~target_point_topic", "/pointcloud_target_point")
        self.target_info_topic = rospy.get_param("~target_info_topic", "/pointcloud_target_info")

        self.step = max(1, int(rospy.get_param("~step", 2)))
        self.min_depth = float(rospy.get_param("~min_depth", 0.1))
        self.max_depth = float(rospy.get_param("~max_depth", 8.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.min_target_points = int(rospy.get_param("~min_target_points", 80))

        self.target_hsv_lower = np.array(
            rospy.get_param("~target_hsv_lower", [90, 25, 20]),
            dtype=np.uint8,
        )
        self.target_hsv_upper = np.array(
            rospy.get_param("~target_hsv_upper", [140, 255, 220]),
            dtype=np.uint8,
        )

        self.save_pcd = bool(rospy.get_param("~save_pcd", True))
        self.pcd_output_path = Path(
            rospy.get_param("~pcd_output_path", str(Path.home() / "home_robot/output_cloud.pcd"))
        )
        self.save_interval = float(rospy.get_param("~save_interval", 1.0))
        self.last_save_time = rospy.Time(0)
        self.last_saved_path = None

        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.info = None

        self.cloud_pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.target_cloud_pub = rospy.Publisher(self.target_cloud_topic, PointCloud2, queue_size=1)
        self.target_point_pub = rospy.Publisher(self.target_point_topic, PointStamped, queue_size=1)
        self.target_info_pub = rospy.Publisher(self.target_info_topic, String, queue_size=10)

        self.sub_rgb = rospy.Subscriber(self.rgb_topic, Image, self.cb_rgb, queue_size=1, buff_size=2**24)
        self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1, buff_size=2**24)
        self.sub_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        self.last_target_distance = None

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("RGBD point cloud node started.")
        rospy.loginfo("RGB topic: %s", self.rgb_topic)
        rospy.loginfo("Depth topic: %s", self.depth_topic)
        rospy.loginfo("Camera info topic: %s", self.camera_info_topic)
        rospy.loginfo("Output topic: %s", self.output_topic)
        rospy.loginfo("Target cloud topic: %s", self.target_cloud_topic)
        rospy.loginfo("Step: %d", self.step)
        rospy.loginfo("PCD auto-save: %s", self.save_pcd)
        rospy.loginfo("PCD base path: %s", str(self.pcd_output_path))

    def on_shutdown(self):
        pass

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

    @staticmethod
    def _pack_rgb_float(bgr):
        b = bgr[:, 0].astype(np.uint32)
        g = bgr[:, 1].astype(np.uint32)
        r = bgr[:, 2].astype(np.uint32)
        rgb_uint32 = (r << 16) | (g << 8) | b
        return rgb_uint32.view(np.float32)

    @staticmethod
    def _cloud_fields():
        return [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]

    def _publish_cloud(self, pub, points, frame_id):
        if not points:
            return None
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        cloud = pcl2.create_cloud(header, self._cloud_fields(), points)
        pub.publish(cloud)
        return cloud

    def _write_pcd_ascii(self, points):
        if not self.save_pcd or not points:
            return
        if (rospy.Time.now() - self.last_save_time).to_sec() < self.save_interval:
            return

        self.pcd_output_path.parent.mkdir(parents=True, exist_ok=True)
        timestamp = rospy.Time.now().to_sec()
        suffix = self.pcd_output_path.suffix or ".pcd"
        save_path = self.pcd_output_path.with_name(f"{self.pcd_output_path.stem}_{timestamp:.3f}{suffix}")

        with save_path.open("w", encoding="utf-8") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            for x, y, z, rgb in points:
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {rgb:.8e}\n")
        self.last_save_time = rospy.Time.now()
        self.last_saved_path = str(save_path)
        rospy.loginfo("Saved PCD: %s", str(save_path))

    def _build_points(self):
        if self.rgb is None or self.depth is None or self.info is None:
            return None, None, None

        rgb = self.rgb
        depth = self._depth_to_meters(self.depth)
        h_d, w_d = depth.shape[:2]
        h_rgb, w_rgb = rgb.shape[:2]

        fx = float(self.info.K[0])
        fy = float(self.info.K[4])
        cx = float(self.info.K[2])
        cy = float(self.info.K[5])

        u = np.arange(0, w_d, self.step, dtype=np.int32)
        v = np.arange(0, h_d, self.step, dtype=np.int32)
        uu, vv = np.meshgrid(u, v)

        depth_sample = depth[vv, uu]
        valid = np.isfinite(depth_sample)
        valid &= depth_sample >= self.min_depth
        valid &= depth_sample <= self.max_depth

        if not np.any(valid):
            return [], [], {"count": 0}

        uu_valid = uu[valid].astype(np.float32)
        vv_valid = vv[valid].astype(np.float32)
        z = depth_sample[valid].astype(np.float32)
        x = (uu_valid - cx) * z / fx
        y = (cy - vv_valid) * z / fy

        if w_rgb != w_d or h_rgb != h_d:
            uu_rgb = np.clip((uu_valid * (w_rgb / float(w_d))).astype(np.int32), 0, w_rgb - 1)
            vv_rgb = np.clip((vv_valid * (h_rgb / float(h_d))).astype(np.int32), 0, h_rgb - 1)
        else:
            uu_rgb = uu_valid.astype(np.int32)
            vv_rgb = vv_valid.astype(np.int32)

        colors = rgb[vv_rgb, uu_rgb]
        rgb_float = self._pack_rgb_float(colors)

        points = list(zip(x.tolist(), y.tolist(), z.tolist(), rgb_float.tolist()))

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.target_hsv_lower, self.target_hsv_upper)
        mask = cv2.medianBlur(mask, 5)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        color_selected = mask[vv_rgb, uu_rgb] > 0
        target_valid = valid.copy()
        target_valid[valid] &= color_selected

        if np.any(target_valid):
            tx = x[color_selected]
            ty = y[color_selected]
            tz = z[color_selected]
            target_colors = colors[color_selected]
            target_rgb = self._pack_rgb_float(target_colors)
            target_points = list(zip(tx.tolist(), ty.tolist(), tz.tolist(), target_rgb.tolist()))
        else:
            target_points = []

        centroid = None
        if len(target_points) >= self.min_target_points:
            tx = np.array([p[0] for p in target_points], dtype=np.float32)
            ty = np.array([p[1] for p in target_points], dtype=np.float32)
            tz = np.array([p[2] for p in target_points], dtype=np.float32)
            centroid = np.array([float(np.mean(tx)), float(np.mean(ty)), float(np.mean(tz))], dtype=np.float32)

        return points, target_points, {
            "count": len(points),
            "target_count": len(target_points),
            "centroid": centroid,
            "frame_id": self.info.header.frame_id or self.frame_id,
        }

    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            points, target_points, meta = self._build_points()
            if points is None:
                rate.sleep()
                continue

            frame_id = meta["frame_id"]
            cloud = self._publish_cloud(self.cloud_pub, points, frame_id)
            target_cloud = self._publish_cloud(self.target_cloud_pub, target_points, frame_id)

            if cloud is not None:
                self._write_pcd_ascii(points)

            info = {
                "frame_id": frame_id,
                "point_count": meta["count"],
                "target_point_count": meta["target_count"],
            }

            centroid = meta.get("centroid")
            if centroid is not None:
                distance = float(np.linalg.norm(centroid))
                info["target_centroid"] = {
                    "x": float(centroid[0]),
                    "y": float(centroid[1]),
                    "z": float(centroid[2]),
                    "distance": distance,
                }
                if self.last_target_distance is None or abs(distance - self.last_target_distance) > 0.1:
                    rospy.loginfo(
                        "Target centroid: x=%.2f y=%.2f z=%.2f distance=%.2f m (points=%d)",
                        centroid[0],
                        centroid[1],
                        centroid[2],
                        distance,
                        meta["target_count"],
                    )
                    self.last_target_distance = distance

                pt = PointStamped()
                pt.header.stamp = rospy.Time.now()
                pt.header.frame_id = frame_id
                pt.point.x = float(centroid[0])
                pt.point.y = float(centroid[1])
                pt.point.z = float(centroid[2])
                self.target_point_pub.publish(pt)

            rospy.loginfo_throttle(
                3.0,
                "Point cloud stats: total=%d target=%d save=%s",
                meta["count"],
                meta["target_count"],
                "on" if self.save_pcd else "off",
            )

            self.target_info_pub.publish(String(data=json.dumps(info, ensure_ascii=False)))

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("get_pointcloud")
    RGBDPointCloudNode().run()
