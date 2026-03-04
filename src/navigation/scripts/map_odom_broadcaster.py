#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Publish map->odom from:
  - global odom (map-like frame -> base frame), e.g. FAST-LOCALIZATION /Odometry
  - local odom (odom frame -> base frame), e.g. /ranger_odom

Formula:
  T_map_odom = T_map_base * inv(T_odom_base)
"""

import threading

import numpy as np
import rospy
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def odom_msg_to_matrix(msg: Odometry) -> np.ndarray:
    pose = msg.pose.pose
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    t = [pose.position.x, pose.position.y, pose.position.z]

    mat = tft.quaternion_matrix(q)
    mat[0, 3] = t[0]
    mat[1, 3] = t[1]
    mat[2, 3] = t[2]
    return mat


class MapOdomBroadcaster:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.global_odom_topic = rospy.get_param("~global_odom_topic", "/Odometry")
        self.local_odom_topic = rospy.get_param("~local_odom_topic", "/ranger_odom")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.msg_timeout = float(rospy.get_param("~msg_timeout", 0.5))

        self._lock = threading.Lock()
        self._global_odom = None
        self._local_odom = None

        self._tf_broadcaster = tf2_ros.TransformBroadcaster()

        self._global_sub = rospy.Subscriber(
            self.global_odom_topic, Odometry, self._global_odom_cb, queue_size=10
        )
        self._local_sub = rospy.Subscriber(
            self.local_odom_topic, Odometry, self._local_odom_cb, queue_size=50
        )

        period = max(1.0 / self.publish_rate, 0.001)
        self._timer = rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo(
            "map_odom_broadcaster started: map_frame=%s odom_frame=%s global_odom=%s local_odom=%s",
            self.map_frame,
            self.odom_frame,
            self.global_odom_topic,
            self.local_odom_topic,
        )

    def _global_odom_cb(self, msg: Odometry):
        with self._lock:
            self._global_odom = msg

    def _local_odom_cb(self, msg: Odometry):
        with self._lock:
            self._local_odom = msg

    def _on_timer(self, _event):
        with self._lock:
            global_odom = self._global_odom
            local_odom = self._local_odom

        if global_odom is None or local_odom is None:
            return

        now = rospy.Time.now()
        g_age = (now - global_odom.header.stamp).to_sec() if global_odom.header.stamp else 0.0
        l_age = (now - local_odom.header.stamp).to_sec() if local_odom.header.stamp else 0.0
        if g_age > self.msg_timeout or l_age > self.msg_timeout:
            rospy.logwarn_throttle(
                2.0,
                "map_odom_broadcaster stale odom: global_age=%.3fs local_age=%.3fs",
                g_age,
                l_age,
            )
            return

        t_map_base = odom_msg_to_matrix(global_odom)
        t_odom_base = odom_msg_to_matrix(local_odom)

        try:
            t_map_odom = np.matmul(t_map_base, np.linalg.inv(t_odom_base))
        except np.linalg.LinAlgError:
            rospy.logwarn_throttle(2.0, "map_odom_broadcaster failed to invert local odom matrix")
            return

        trans = tft.translation_from_matrix(t_map_odom)
        quat = tft.quaternion_from_matrix(t_map_odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = float(trans[0])
        tf_msg.transform.translation.y = float(trans[1])
        tf_msg.transform.translation.z = float(trans[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self._tf_broadcaster.sendTransform(tf_msg)


def main():
    rospy.init_node("map_odom_broadcaster")
    MapOdomBroadcaster()
    rospy.spin()


if __name__ == "__main__":
    main()

