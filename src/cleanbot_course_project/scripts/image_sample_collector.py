#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from datetime import datetime


class ImageSampleCollector:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.output_dir = rospy.get_param("~output_dir", os.path.expanduser("~/home_robot/datasets/target_samples"))
        self.label = rospy.get_param("~label", "target")
        self.auto_save = rospy.get_param("~auto_save", False)
        self.save_interval = float(rospy.get_param("~save_interval", 1.0))
        self.resize_width = int(rospy.get_param("~resize_width", 0))
        self.max_samples = int(rospy.get_param("~max_samples", 0))
        self.show_preview = bool(rospy.get_param("~show_preview", False))

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_stamp = None
        self.sample_count = 0
        self.last_save_time = rospy.Time.now()

        self.label_dir = os.path.join(self.output_dir, self.label)
        os.makedirs(self.label_dir, exist_ok=True)

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )

        rospy.loginfo("Image sample collector started.")
        rospy.loginfo("Image topic: %s", self.image_topic)
        rospy.loginfo("Output dir : %s", self.label_dir)
        rospy.loginfo("Label      : %s", self.label)
        rospy.loginfo("Keys: [s] save one image, [a] toggle auto-save, [q] quit")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.resize_width > 0:
                h, w = frame.shape[:2]
                scale = self.resize_width / float(w)
                frame = cv2.resize(frame, (self.resize_width, int(h * scale)))
            self.latest_frame = frame
            self.latest_stamp = msg.header.stamp
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))

    def save_frame(self):
        if self.latest_frame is None:
            rospy.logwarn("No image received yet.")
            return

        if self.max_samples > 0 and self.sample_count >= self.max_samples:
            rospy.logwarn("Reached max_samples=%d", self.max_samples)
            return

        now_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{self.label}_{now_str}_{self.sample_count:06d}.jpg"
        save_path = os.path.join(self.label_dir, filename)

        ok = cv2.imwrite(save_path, self.latest_frame)
        if ok:
            self.sample_count += 1
            rospy.loginfo("Saved image: %s", save_path)
        else:
            rospy.logerr("Failed to save image: %s", save_path)

    def draw_overlay(self, frame):
        view = frame.copy()
        text_lines = [
            f"Label: {self.label}",
            f"Saved: {self.sample_count}",
            f"Auto-save: {self.auto_save}",
            "s: save | a: auto | q: quit"
        ]

        y = 25
        for line in text_lines:
            cv2.putText(
                view,
                line,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA
            )
            y += 28
        return view

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.latest_frame is not None and self.show_preview:
                frame_show = self.draw_overlay(self.latest_frame)
                cv2.imshow("CleanBot Image Sample Collector", frame_show)

                key = cv2.waitKey(1) & 0xFF

                if key == ord("s"):
                    self.save_frame()

                elif key == ord("a"):
                    self.auto_save = not self.auto_save
                    rospy.loginfo("Auto-save set to: %s", self.auto_save)

                elif key == ord("q"):
                    rospy.loginfo("Quit image sample collector.")
                    break

            if self.auto_save:
                now = rospy.Time.now()
                if (now - self.last_save_time).to_sec() >= self.save_interval:
                    self.save_frame()
                    self.last_save_time = now

            if not self.show_preview:
                rate.sleep()
                continue

            rate.sleep()

        if self.show_preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("image_sample_collector", anonymous=False)
    collector = ImageSampleCollector()
    collector.run()
