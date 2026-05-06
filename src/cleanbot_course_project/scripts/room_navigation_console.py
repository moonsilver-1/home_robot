#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

import sys
import termios
import tty


class RoomNavigationConsole:
    def __init__(self):
        self.command_topic = rospy.get_param("~command_topic", "/cleanbot/room_navigation/command")
        self.publisher = rospy.Publisher(self.command_topic, String, queue_size=10)
        self.aliases = {
            "l": "living_room",
            "living_room": "living_room",
            "客厅": "living_room",
            "k": "kitchen",
            "kitchen": "kitchen",
            "厨房": "kitchen",
            "b": "bedroom",
            "bedroom": "bedroom",
            "卧室": "bedroom",
        }
        self.single_key_aliases = {
            "l": "living_room",
            "k": "kitchen",
            "b": "bedroom",
            "q": "QUIT",
        }

    def resolve_command(self, text):
        value = str(text).strip()
        if not value:
            return None
        key = value.lower()
        return self.aliases.get(key, value)

    def run(self):
        rospy.loginfo("Room command console started.")
        rospy.loginfo("Press l/k/b to go to living_room/kitchen/bedroom, q to quit.")
        rospy.loginfo("Press Ctrl+C to exit.")

        if sys.stdin.isatty():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while not rospy.is_shutdown():
                    raw = sys.stdin.read(1)
                    if not raw:
                        continue

                    key = raw.lower()
                    command = self.single_key_aliases.get(key)
                    if command == "QUIT":
                        break
                    if command is None:
                        continue

                    self.publisher.publish(String(data=command))
                    rospy.loginfo("Published room command: %s -> %s", raw, command)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        else:
            while not rospy.is_shutdown():
                try:
                    raw = input("room> ").strip()
                except EOFError:
                    break
                except KeyboardInterrupt:
                    break

                command = self.resolve_command(raw)
                if command is None:
                    continue

                if command.lower() in {"q", "quit", "exit"}:
                    break

                self.publisher.publish(String(data=command))
                rospy.loginfo("Published room command: %s -> %s", raw, command)


if __name__ == "__main__":
    rospy.init_node("room_navigation_console")
    RoomNavigationConsole().run()
