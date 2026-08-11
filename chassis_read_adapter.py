#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read-only adapter for the new Jaten/Kuavo chassis ROS interface.

This file only reads ROS topics. It does not publish motion commands or call
control services.
"""

import math
from typing import Dict, Optional

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ChassisReadAdapter:
    """Read-only replacement for old AMR query methods."""

    POSE_TOPIC = "/move_base/amcl_pose"
    NAV_SOURCE_TOPIC = "/twist_mux_new/out/nav_source_used"

    def __init__(self, wait_timeout: float = 2.0):
        self.wait_timeout = wait_timeout

    def get_current_pose(self, timeout: Optional[float] = None) -> Optional[Dict[str, float]]:
        """
        Replacement for old robot_pose_speed()['pose'].

        Returns:
            {"x": float, "y": float, "theta": float}, or None on timeout/error.
        """
        timeout = self.wait_timeout if timeout is None else timeout
        try:
            msg = rospy.wait_for_message(self.POSE_TOPIC, PoseWithCovarianceStamped, timeout=timeout)
        except Exception as exc:
            rospy.logwarn("Failed to read %s: %s", self.POSE_TOPIC, exc)
            return None

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        theta = quat_to_yaw(ori.x, ori.y, ori.z, ori.w)

        return {
            "x": pos.x,
            "y": pos.y,
            "theta": theta,
        }

    def get_nav_source_used(self, timeout: Optional[float] = None) -> Optional[str]:
        """Read currently selected twist_mux navigation source."""
        timeout = self.wait_timeout if timeout is None else timeout
        try:
            msg = rospy.wait_for_message(self.NAV_SOURCE_TOPIC, String, timeout=timeout)
            return msg.data
        except Exception as exc:
            rospy.logwarn("Failed to read %s: %s", self.NAV_SOURCE_TOPIC, exc)
            return None

    def check_motion_interface_ready(self, timeout: Optional[float] = None) -> Dict[str, object]:
        """仅检查位姿读取接口；不检查急停、驱动使能或底盘故障。"""
        pose = self.get_current_pose(timeout=timeout)
        nav_source = self.get_nav_source_used(timeout=timeout)
        reasons = []
        if pose is None:
            reasons.append("pose_unavailable")

        return {
            "ready": not reasons,
            "reasons": reasons,
            "pose": pose,
            "nav_source_used": nav_source,
        }
