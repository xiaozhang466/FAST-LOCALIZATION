#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel smooth filter:
- Low-pass filter (EMA) to smooth rapid velocity changes
- Rate limiter to prevent sudden acceleration/deceleration
- Small deadband to suppress near-zero noise
- Watchdog: publish zero velocity if no input for timeout period
"""

import rospy
from geometry_msgs.msg import Twist


class CmdVelFilter:
    def __init__(self):
        # Deadband: suppress very small values
        self.angular_deadband = rospy.get_param("~angular_deadband", 0.03)
        self.linear_deadband = rospy.get_param("~linear_deadband", 0.0)

        # Low-pass filter smoothing factor (0~1, smaller = smoother but more lag)
        self.alpha_linear = rospy.get_param("~alpha_linear", 0.3)
        self.alpha_angular = rospy.get_param("~alpha_angular", 0.25)

        # Rate limiter: max change per cycle (at 10Hz control freq)
        self.max_linear_acc = rospy.get_param("~max_linear_acc", 0.15)   # m/s per cycle
        self.max_angular_acc = rospy.get_param("~max_angular_acc", 0.15)  # rad/s per cycle

        # Watchdog timeout
        self.timeout = rospy.get_param("~timeout", 0.5)

        input_topic = rospy.get_param("~input_topic", "/cmd_vel_raw")
        output_topic = rospy.get_param("~output_topic", "/cmd_vel")

        self.pub = rospy.Publisher(output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(input_topic, Twist, self.cb, queue_size=10)

        # State
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0
        self.last_msg_time = rospy.Time.now()
        self.stopped = False
        self.timer = rospy.Timer(rospy.Duration(0.1), self.watchdog)

    def _clamp(self, value, limit):
        """Clamp value to [-limit, limit]."""
        return max(-limit, min(limit, value))

    def _apply_deadband(self, value, deadband):
        """Set value to 0 if within deadband."""
        return 0.0 if abs(value) < deadband else value

    def _rate_limit(self, target, prev, max_change):
        """Limit the rate of change between consecutive outputs."""
        delta = self._clamp(target - prev, max_change)
        return prev + delta

    def cb(self, msg: Twist):
        self.last_msg_time = rospy.Time.now()
        self.stopped = False

        # 1. Apply deadband
        linear_x = self._apply_deadband(msg.linear.x, self.linear_deadband)
        angular_z = self._apply_deadband(msg.angular.z, self.angular_deadband)

        # 2. Low-pass filter (EMA): smoothed = alpha * new + (1 - alpha) * prev
        linear_x = self.alpha_linear * linear_x + (1.0 - self.alpha_linear) * self.prev_linear_x
        angular_z = self.alpha_angular * angular_z + (1.0 - self.alpha_angular) * self.prev_angular_z

        # 3. Rate limiter: prevent sudden jumps
        linear_x = self._rate_limit(linear_x, self.prev_linear_x, self.max_linear_acc)
        angular_z = self._rate_limit(angular_z, self.prev_angular_z, self.max_angular_acc)

        # Update state
        self.prev_linear_x = linear_x
        self.prev_angular_z = angular_z

        # Publish
        out = Twist()
        out.linear.x = linear_x
        out.angular.z = angular_z
        self.pub.publish(out)

    def watchdog(self, event):
        """Publish zero velocity if no input received within timeout."""
        if self.stopped:
            return
        dt = (rospy.Time.now() - self.last_msg_time).to_sec()
        if dt > self.timeout:
            self.pub.publish(Twist())
            self.prev_linear_x = 0.0
            self.prev_angular_z = 0.0
            self.stopped = True
            rospy.logwarn_throttle(5.0, "cmd_vel_filter: no input for %.1fs, sending zero velocity" % dt)


def main():
    rospy.init_node("cmd_vel_filter")
    CmdVelFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
