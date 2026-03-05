#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel delay compensator:
- Delay feedforward compensation: u_comp = u + tau * du/dt
"""

import rospy
from geometry_msgs.msg import Twist


class CmdVelFilter:
    def __init__(self):
        # Delay feedforward compensation: u_comp = u + tau * du/dt
        self.delay_comp_enable = rospy.get_param("~delay_comp_enable", True)
        self.delay_comp_tau_linear = rospy.get_param("~delay_comp_tau_linear", 0.14)
        self.delay_comp_tau_angular = rospy.get_param("~delay_comp_tau_angular", 0.14)
        self.delay_comp_max_dt = rospy.get_param("~delay_comp_max_dt", 0.2)
        self.delay_comp_max_linear_boost = rospy.get_param("~delay_comp_max_linear_boost", 0.20)
        self.delay_comp_max_angular_boost = rospy.get_param("~delay_comp_max_angular_boost", 0.50)

        input_topic = rospy.get_param("~input_topic", "/cmd_vel_raw")
        output_topic = rospy.get_param("~output_topic", "/cmd_vel")

        self.pub = rospy.Publisher(output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(
            input_topic, Twist, self.cb, queue_size=1, tcp_nodelay=True
        )

        # State
        self.prev_target_linear_x = 0.0
        self.prev_target_angular_z = 0.0
        self.last_input_time = None

        rospy.loginfo(
            "cmd_vel_filter started: delay_comp_enable=%s tau_linear=%.3f tau_angular=%.3f",
            str(self.delay_comp_enable),
            self.delay_comp_tau_linear,
            self.delay_comp_tau_angular,
        )

    def _delay_compensate(self, target, prev_target, dt, tau, max_boost):
        """Feedforward compensation to counteract fixed actuation delay."""
        if dt <= 1e-4 or dt > self.delay_comp_max_dt:
            return target
        derivative = (target - prev_target) / dt
        boost = max(-max_boost, min(max_boost, tau * derivative))
        return target + boost

    def cb(self, msg: Twist):
        now = rospy.Time.now()
        dt = None
        if self.last_input_time is not None:
            dt = (now - self.last_input_time).to_sec()

        target_linear_x = msg.linear.x
        target_angular_z = msg.angular.z

        # Delay compensation using command derivative
        if self.delay_comp_enable and dt is not None:
            target_linear_x = self._delay_compensate(
                target_linear_x,
                self.prev_target_linear_x,
                dt,
                self.delay_comp_tau_linear,
                self.delay_comp_max_linear_boost,
            )
            target_angular_z = self._delay_compensate(
                target_angular_z,
                self.prev_target_angular_z,
                dt,
                self.delay_comp_tau_angular,
                self.delay_comp_max_angular_boost,
            )

        out = Twist()
        out.linear.x = target_linear_x
        out.linear.y = msg.linear.y
        out.linear.z = msg.linear.z
        out.angular.x = msg.angular.x
        out.angular.y = msg.angular.y
        out.angular.z = target_angular_z
        self.pub.publish(out)

        # Update state with original command for next derivative estimate
        self.prev_target_linear_x = msg.linear.x
        self.prev_target_angular_z = msg.angular.z
        self.last_input_time = now


def main():
    rospy.init_node("cmd_vel_filter")
    CmdVelFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
