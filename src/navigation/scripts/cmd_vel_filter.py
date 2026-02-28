#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel deadband filter:
- If |angular.z| < angular_deadband -> set angular.z = 0
- Optional linear deadband for linear components
"""

import rospy
from geometry_msgs.msg import Twist


class CmdVelFilter:
    def __init__(self):
        self.angular_deadband = rospy.get_param("~angular_deadband", 0.1)
        self.linear_deadband = rospy.get_param("~linear_deadband", 0.0)
        input_topic = rospy.get_param("~input_topic", "/cmd_vel_raw")
        output_topic = rospy.get_param("~output_topic", "/cmd_vel")

        self.pub = rospy.Publisher(output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(input_topic, Twist, self.cb, queue_size=10)

    def cb(self, msg: Twist):
        out = Twist()
        out.linear.x = 0.0 if abs(msg.linear.x) < self.linear_deadband else msg.linear.x
        out.linear.y = 0.0 if abs(msg.linear.y) < self.linear_deadband else msg.linear.y
        out.linear.z = 0.0 if abs(msg.linear.z) < self.linear_deadband else msg.linear.z

        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = 0.0 if abs(msg.angular.z) < self.angular_deadband else msg.angular.z

        self.pub.publish(out)


def main():
    rospy.init_node("cmd_vel_filter")
    CmdVelFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
