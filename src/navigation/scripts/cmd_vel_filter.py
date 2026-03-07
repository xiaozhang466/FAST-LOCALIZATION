#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel filter:
- Delay feedforward compensation
- Angular smoothing (low-pass + slew-rate limit)
- Angular feedback damping (predictive braking by measured yaw rate)
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CmdVelFilter:
    def __init__(self):
        # Delay feedforward compensation: u_comp = u + tau * du/dt
        self.delay_comp_enable = rospy.get_param("~delay_comp_enable", True)
        self.delay_comp_tau_linear = rospy.get_param("~delay_comp_tau_linear", 0.14)
        self.delay_comp_tau_angular = rospy.get_param("~delay_comp_tau_angular", 0.14)
        self.delay_comp_max_dt = rospy.get_param("~delay_comp_max_dt", 0.2)
        self.delay_comp_max_linear_boost = rospy.get_param("~delay_comp_max_linear_boost", 0.20)
        self.delay_comp_max_angular_boost = rospy.get_param("~delay_comp_max_angular_boost", 0.50)
        # Angular smoothing: low-pass + slew-rate limit
        self.angular_lpf_enable = rospy.get_param("~angular_lpf_enable", True)
        self.angular_lpf_alpha = rospy.get_param("~angular_lpf_alpha", 0.35)
        self.angular_slew_rate = rospy.get_param("~angular_slew_rate", 1.0)
        # Angular predictive braking via measured yaw-rate feedback
        self.angular_damping_enable = rospy.get_param("~angular_damping_enable", True)
        self.angular_damping_topic = rospy.get_param("~angular_damping_topic", "/ranger_odom")
        self.angular_damping_gain = rospy.get_param("~angular_damping_gain", 0.35)
        self.angular_damping_max = rospy.get_param("~angular_damping_max", 0.12)
        self.angular_damping_timeout = rospy.get_param("~angular_damping_timeout", 0.2)

        input_topic = rospy.get_param("~input_topic", "/cmd_vel_raw")
        output_topic = rospy.get_param("~output_topic", "/cmd_vel")

        self.pub = rospy.Publisher(output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(
            input_topic, Twist, self.cb, queue_size=1, tcp_nodelay=True
        )
        self.odom_sub = rospy.Subscriber(
            self.angular_damping_topic, Odometry, self.odom_cb, queue_size=10, tcp_nodelay=True
        )

        # State
        self.prev_target_linear_x = 0.0
        self.prev_target_angular_z = 0.0
        self.last_input_time = None
        self.prev_output_angular_z = 0.0
        self.last_output_time = None
        self.measured_angular_z = 0.0
        self.last_odom_time = None

        rospy.loginfo(
            "cmd_vel_filter started: delay_comp_enable=%s tau_linear=%.3f tau_angular=%.3f "
            "angular_lpf_enable=%s alpha=%.2f angular_slew_rate=%.2f "
            "angular_damping_enable=%s gain=%.2f max=%.2f topic=%s",
            str(self.delay_comp_enable),
            self.delay_comp_tau_linear,
            self.delay_comp_tau_angular,
            str(self.angular_lpf_enable),
            self.angular_lpf_alpha,
            self.angular_slew_rate,
            str(self.angular_damping_enable),
            self.angular_damping_gain,
            self.angular_damping_max,
            self.angular_damping_topic,
        )

    def odom_cb(self, msg: Odometry):
        self.measured_angular_z = msg.twist.twist.angular.z
        self.last_odom_time = rospy.Time.now()

    def _delay_compensate(self, target, prev_target, dt, tau, max_boost):
        """Feedforward compensation to counteract fixed actuation delay."""
        if dt <= 1e-4 or dt > self.delay_comp_max_dt:
            return target
        derivative = (target - prev_target) / dt
        boost = max(-max_boost, min(max_boost, tau * derivative))
        return target + boost

    def cb(self, msg: Twist):
        now = rospy.Time.now()
        dt_in = None
        if self.last_input_time is not None:
            dt_in = (now - self.last_input_time).to_sec()

        target_linear_x = msg.linear.x
        target_angular_z = msg.angular.z

        # Delay compensation using command derivative
        if self.delay_comp_enable and dt_in is not None:
            target_linear_x = self._delay_compensate(
                target_linear_x,
                self.prev_target_linear_x,
                dt_in,
                self.delay_comp_tau_linear,
                self.delay_comp_max_linear_boost,
            )
            target_angular_z = self._delay_compensate(
                target_angular_z,
                self.prev_target_angular_z,
                dt_in,
                self.delay_comp_tau_angular,
                self.delay_comp_max_angular_boost,
            )

        # Predictive braking using measured yaw rate to reduce turn overshoot.
        if self.angular_damping_enable and self.last_odom_time is not None:
            odom_age = (now - self.last_odom_time).to_sec()
            if 0.0 <= odom_age <= self.angular_damping_timeout:
                correction = self.angular_damping_gain * self.measured_angular_z
                max_corr = max(0.0, self.angular_damping_max)
                correction = max(-max_corr, min(max_corr, correction))
                target_angular_z -= correction

        # Low-pass filter on angular command to reduce oscillatory wheel steering.
        if self.angular_lpf_enable and self.last_output_time is not None:
            alpha = max(0.0, min(1.0, self.angular_lpf_alpha))
            target_angular_z = alpha * target_angular_z + (1.0 - alpha) * self.prev_output_angular_z

        # Slew-rate limit on angular command: limit |dw/dt|.
        if self.last_output_time is not None:
            dt_out = (now - self.last_output_time).to_sec()
            if dt_out > 1e-4 and self.angular_slew_rate > 0.0:
                max_delta = self.angular_slew_rate * dt_out
                delta = target_angular_z - self.prev_output_angular_z
                if delta > max_delta:
                    target_angular_z = self.prev_output_angular_z + max_delta
                elif delta < -max_delta:
                    target_angular_z = self.prev_output_angular_z - max_delta

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
        self.prev_output_angular_z = target_angular_z
        self.last_output_time = now


def main():
    rospy.init_node("cmd_vel_filter")
    CmdVelFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
