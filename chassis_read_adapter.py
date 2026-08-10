#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read-only adapter for the new Jaten/Kuavo chassis ROS interface.

This file only reads ROS topics. It does not publish motion commands or call
control services.
"""

import math
from typing import Any, Dict, Optional

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

try:
    from jaten_msgs.msg import ErrorState, RobotChassisState
except ImportError:  # Allows importing on machines without the robot workspace.
    ErrorState = None
    RobotChassisState = None


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ChassisReadAdapter:
    """Read-only replacement for old AMR query methods."""

    POSE_TOPIC = "/move_base/amcl_pose"
    ROBOT_STATE_TOPIC = "/jaten_behavior_manager/robot_state"
    DRIVER_ERROR_TOPIC = "/jaten_behavior_manager/out/state/chassis_driver_error_state"
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

    def get_robot_state(self, timeout: Optional[float] = None) -> Optional[Any]:
        """Read raw jaten_msgs/RobotChassisState."""
        if RobotChassisState is None:
            rospy.logerr("jaten_msgs/RobotChassisState is not available in this environment")
            return None

        timeout = self.wait_timeout if timeout is None else timeout
        try:
            return rospy.wait_for_message(self.ROBOT_STATE_TOPIC, RobotChassisState, timeout=timeout)
        except Exception as exc:
            rospy.logwarn("Failed to read %s: %s", self.ROBOT_STATE_TOPIC, exc)
            return None

    def get_robot_state_dict(self, timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Read chassis state and convert known fields to a plain dict."""
        msg = self.get_robot_state(timeout=timeout)
        if msg is None:
            return None

        fields = [
            "manual_mode",
            "is_moving",
            "mapping",
            "soft_estop",
            "dispatch_mode",
            "driver_error",
            "driver_enable",
            "estop",
            "bumper",
            "start_charge",
            "battery_level",
            "upper_limit_detection",
            "lower_limit_detection",
            "detect_zero",
            "led",
            "music_id",
        ]
        return {name: getattr(msg, name) for name in fields if hasattr(msg, name)}

    def get_driver_error_state(self, timeout: Optional[float] = None) -> Optional[Any]:
        """Read raw jaten_msgs/ErrorState for chassis driver errors."""
        if ErrorState is None:
            rospy.logerr("jaten_msgs/ErrorState is not available in this environment")
            return None

        timeout = self.wait_timeout if timeout is None else timeout
        try:
            return rospy.wait_for_message(self.DRIVER_ERROR_TOPIC, ErrorState, timeout=timeout)
        except Exception as exc:
            rospy.logwarn("Failed to read %s: %s", self.DRIVER_ERROR_TOPIC, exc)
            return None

    def get_nav_source_used(self, timeout: Optional[float] = None) -> Optional[str]:
        """Read currently selected twist_mux navigation source."""
        timeout = self.wait_timeout if timeout is None else timeout
        try:
            msg = rospy.wait_for_message(self.NAV_SOURCE_TOPIC, String, timeout=timeout)
            return msg.data
        except Exception as exc:
            rospy.logwarn("Failed to read %s: %s", self.NAV_SOURCE_TOPIC, exc)
            return None

    def check_chassis_ready(self, timeout: Optional[float] = None) -> Dict[str, Any]:
        """
        Read readiness-related state.

        This mirrors the old robot_mode()/connection check idea, but uses the
        actual Jaten chassis status topic.
        """
        state = self.get_robot_state_dict(timeout=timeout)
        driver_error = self.get_driver_error_state(timeout=timeout)
        nav_source = self.get_nav_source_used(timeout=timeout)

        ready = False
        reasons = []

        if state is None:
            reasons.append("robot_state_unavailable")
        else:
            if not state.get("driver_enable", False):
                reasons.append("driver_not_enabled")
            if state.get("driver_error", False):
                reasons.append("driver_error")
            if state.get("estop", False):
                reasons.append("estop_pressed")
            if state.get("soft_estop", False):
                reasons.append("soft_estop")
            if state.get("bumper", False):
                reasons.append("bumper_triggered")
            if not reasons:
                ready = True

        if driver_error is not None and getattr(driver_error, "status", 0) != 0:
            ready = False
            reasons.append("driver_error_state_nonzero")

        return {
            "ready": ready,
            "reasons": reasons,
            "robot_state": state,
            "driver_error_state": driver_error,
            "nav_source_used": nav_source,
        }

