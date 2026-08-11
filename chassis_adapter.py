#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo/Jaten 新底盘统一适配器。

本文件只负责底盘，组合两个已经确认的接口：

1. ROS `/move_base/amcl_pose`：读取地图坐标系下的 x、y、theta。
2. 底盘 HTTP `/command?cmd=...`：切换模式、发送 RobotMotion、按站点名导航。

跨区域导航只保留一种方式：预先在底盘路网中配置站点，再调用
`navigate_to_station()`。旧程序的 task_id 和任意 x/y/theta 导航不再保留。

相对前进、后退和横移仍使用“距离”作为调用参数。适配器内部将目标距离转换为
地图坐标目标，再用短时间 RobotMotion 速度脉冲和 AMCL 位姿进行闭环校正。

使用前由主程序完成 `rospy.init_node(...)`。本文件不会初始化机械臂、夹爪、
升降台、相机或视觉功能。
"""

import json
import math
import time
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """将 ROS 四元数转换为偏航角 theta。"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ChassisReadAdapter:
    """只读访问新底盘 ROS 标准消息，不发送任何运动指令。"""

    POSE_TOPIC = "/move_base/amcl_pose"
    NAV_SOURCE_TOPIC = "/twist_mux_new/out/nav_source_used"

    def __init__(self, wait_timeout: float = 2.0):
        self.wait_timeout = wait_timeout

    def get_current_pose(
            self, timeout: Optional[float] = None) -> Optional[Dict[str, float]]:
        """替代旧 `robot_pose_speed()['pose']`，返回 x、y、theta。"""
        timeout = self.wait_timeout if timeout is None else timeout
        try:
            msg = rospy.wait_for_message(
                self.POSE_TOPIC,
                PoseWithCovarianceStamped,
                timeout=timeout,
            )
        except Exception as exc:
            rospy.logwarn("读取 %s 失败: %s", self.POSE_TOPIC, exc)
            return None

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return {
            "x": position.x,
            "y": position.y,
            "theta": quat_to_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        }

    def get_nav_source_used(self, timeout: Optional[float] = None) -> Optional[str]:
        """读取 twist_mux_new 当前选中的导航速度来源。"""
        timeout = self.wait_timeout if timeout is None else timeout
        try:
            msg = rospy.wait_for_message(
                self.NAV_SOURCE_TOPIC,
                String,
                timeout=timeout,
            )
            return msg.data
        except Exception as exc:
            rospy.logwarn("读取 %s 失败: %s", self.NAV_SOURCE_TOPIC, exc)
            return None

    def check_motion_interface_ready(
            self, timeout: Optional[float] = None) -> Dict[str, object]:
        """仅检查位姿读取；不检查急停、驱动使能或底盘故障。"""
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


class ChassisHttpClient:
    """调用底盘 jaten-api，不依赖 ROS 厂商自定义消息包。"""

    def __init__(
            self, host: str = "192.168.26.22", port: int = 8888,
            token: Optional[str] = None, timeout: float = 3.0):
        self.base_url = "http://{}:{}".format(host, port).rstrip("/")
        self.token = token
        self.timeout = timeout

    def send_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """通过 `/command?cmd=...` 发送一条 NavigationApi JSON 命令。"""
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["Authorization"] = self.token

        command_url = self.base_url + "/command?" + urlencode({
            "cmd": json.dumps(payload, separators=(",", ":")),
        })
        request = Request(
            command_url,
            data=b"",
            headers=headers,
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                raw = response.read().decode("utf-8", errors="replace")
                if not raw:
                    return {"http_status": response.status}
                try:
                    result = json.loads(raw)
                except json.JSONDecodeError:
                    result = {"raw": raw}
                if isinstance(result, dict):
                    result.setdefault("http_status", response.status)
                return result
        except HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(
                "Chassis command HTTP {}: {}".format(exc.code, body)
            ) from exc
        except URLError as exc:
            raise RuntimeError(
                "Chassis command connection failed: {}".format(exc.reason)
            ) from exc

    def robot_motion(self, vx: float, vy: float, vw: float) -> Dict[str, Any]:
        """发送一次 RobotMotion 速度请求。"""
        return self.send_command({
            "id": "0",
            "method": "RobotMotion",
            "params": {
                "vx": float(vx),
                "vy": float(vy),
                "vw": float(vw),
            },
        })

    @staticmethod
    def make_change_mode_payload(
            mode: str, request_id: str = "1") -> Dict[str, Any]:
        """构造与底盘网页相同的手动/自动模式切换命令。"""
        normalized_mode = mode.strip().upper()
        if normalized_mode not in ("MANUAL", "AUTO"):
            raise ValueError("mode must be MANUAL or AUTO")
        return {
            "method": "ChangeMode",
            "id": str(request_id),
            "params": {"mode": normalized_mode},
        }

    def change_mode(
            self, mode: str, request_id: str = "1") -> Dict[str, Any]:
        """切换底盘模式；MANUAL 用于 RobotMotion，AUTO 用于导航。"""
        return self.send_command(self.make_change_mode_payload(mode, request_id))

    @staticmethod
    def _command_rejected(result: Dict[str, Any]) -> bool:
        return bool(result.get("error") or result.get("success") is False)

    @staticmethod
    def _requires_manual_mode(result: Dict[str, Any]) -> bool:
        error = result.get("error")
        if not isinstance(error, dict) or error.get("code") != 8211:
            return False
        message = str(error.get("message", "")).lower()
        return "auto mode" in message or "manual mode" in message

    def ensure_manual_mode(
            self, request_id: str = "1", verify_retries: int = 5,
            verify_interval: float = 0.2) -> Dict[str, Any]:
        """用零速度探测 RobotMotion；必要时切到 MANUAL 并再次确认。"""
        probe_result = self.robot_motion(0.0, 0.0, 0.0)
        if not self._command_rejected(probe_result):
            return {"changed": False, "probe_result": probe_result}

        if not self._requires_manual_mode(probe_result):
            raise RuntimeError(
                "RobotMotion readiness probe was rejected: {}".format(probe_result)
            )

        change_result = self.change_mode("MANUAL", request_id)
        if self._command_rejected(change_result):
            raise RuntimeError(
                "ChangeMode MANUAL was rejected: {}".format(change_result)
            )

        last_verify_result = None
        for _ in range(max(1, verify_retries)):
            time.sleep(max(0.0, verify_interval))
            last_verify_result = self.robot_motion(0.0, 0.0, 0.0)
            if not self._command_rejected(last_verify_result):
                return {
                    "changed": True,
                    "probe_result": probe_result,
                    "change_result": change_result,
                    "verify_result": last_verify_result,
                }
        raise RuntimeError(
            "MANUAL mode could not be verified: {}".format(last_verify_result)
        )

    @staticmethod
    def make_dispatch_goal_node_name_payload(
            node_name: str, request_id: str = "1") -> Dict[str, Any]:
        """构造与网页右键站点导航相同的命令，不发送请求。"""
        node_name = node_name.strip()
        if not node_name:
            raise ValueError("node_name must not be empty")
        return {
            "method": "DispatchGoalNodeName",
            "id": str(request_id),
            "params": {"name": [node_name]},
        }

    def dispatch_goal_node_name(
            self, node_name: str, request_id: str = "1") -> Dict[str, Any]:
        """按当前路网中的站点名称下发导航目标。"""
        return self.send_command(
            self.make_dispatch_goal_node_name_payload(node_name, request_id)
        )

    def stop(self, repeat: int = 3) -> None:
        """重复发送零速度，尽量确保手动运动停止。"""
        last_error = None
        for _ in range(max(1, repeat)):
            try:
                self.robot_motion(0.0, 0.0, 0.0)
                last_error = None
            except RuntimeError as exc:
                last_error = exc
        if last_error is not None:
            raise last_error


def normalize_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


class ChassisAdapter:
    """为新底盘提供与旧主程序流程接近的高层控制接口。"""

    def __init__(
            self,
            host: str = "192.168.26.22",
            port: int = 8888,
            token: Optional[str] = None,
            http_timeout: float = 3.0,
            pose_timeout: float = 2.0,
            reader: Optional[ChassisReadAdapter] = None,
            http_client: Optional[ChassisHttpClient] = None):
        self.reader = reader or ChassisReadAdapter(wait_timeout=pose_timeout)
        self.http = http_client or ChassisHttpClient(
            host=host,
            port=port,
            token=token,
            timeout=http_timeout,
        )

    # ==================== 只读状态 ====================

    def get_current_pose(self) -> Optional[Dict[str, float]]:
        """替代旧 `robot_pose_speed()['pose']`，返回 x、y、theta。"""
        return self.reader.get_current_pose()

    def get_nav_source_used(self) -> Optional[str]:
        """返回 twist_mux_new 当前选中的导航速度来源。"""
        return self.reader.get_nav_source_used()

    # ==================== 模式与基础 HTTP 命令 ====================

    @staticmethod
    def _command_rejected(result: Dict[str, Any]) -> bool:
        return bool(result.get("error") or result.get("success") is False)

    def ensure_manual_mode(self) -> Dict[str, Any]:
        """确保 RobotMotion 可用；必要时切换到 MANUAL 并用零速度验证。"""
        return self.http.ensure_manual_mode()

    def ensure_auto_mode(
            self, settle_time: float = 0.5,
            request_id: str = "1") -> Dict[str, Any]:
        """切换到站点导航需要的 AUTO 模式。

        当前上位机没有可直接反序列化的 jaten_msgs 状态消息，因此这里只能确认
        ChangeMode 命令未被 HTTP 接口拒绝，不能像 MANUAL 一样用 RobotMotion
        零速度请求进行反向验证。
        """
        result = self.http.change_mode("AUTO", request_id=request_id)
        if self._command_rejected(result):
            raise RuntimeError("ChangeMode AUTO was rejected: {}".format(result))
        if settle_time > 0.0:
            rospy.sleep(settle_time)
        return result

    def stop(self) -> None:
        """重复发送手动模式零速度；不能替代自动导航任务的取消接口。"""
        self.http.stop()

    def _send_velocity_pulse(
            self, vx: float = 0.0, vy: float = 0.0, vw: float = 0.0,
            duration: float = 0.0, publish_interval: float = 0.1) -> None:
        """在指定时间内重复发送 RobotMotion，并保证结束时发送零速度。"""
        if duration <= 0.0:
            return

        end_time = time.monotonic() + duration
        try:
            while time.monotonic() < end_time and not rospy.is_shutdown():
                result = self.http.robot_motion(vx=vx, vy=vy, vw=vw)
                if self._command_rejected(result):
                    raise RuntimeError("RobotMotion was rejected: {}".format(result))
                rospy.sleep(publish_interval)
        finally:
            self.http.stop()

    # ==================== 相对移动 ====================

    def move_relative(
            self,
            forward: float = 0.0,
            lateral: float = 0.0,
            tolerance: float = 0.01,
            max_iterations: int = 12,
            linear_speed: float = 0.03,
            max_step_distance: float = 0.05,
            settle_time: float = 0.5,
            final_tolerance_multiplier: float = 2.0) -> bool:
        """在机器人起始本体坐标系中闭环移动指定距离。

        Args:
            forward: 正值前进，负值后退，单位米。
            lateral: 正值左移，负值右移，单位米。横移尚需现场小距离验证。
            tolerance: 地图平面最终误差，默认 0.01 米。
            max_iterations: 最大 AMCL 闭环校正次数。
            linear_speed: RobotMotion 线速度绝对值，单位米/秒。
            max_step_distance: 单次速度脉冲对应的最大预计距离。
            settle_time: 每次脉冲后等待底盘和 AMCL 稳定的时间。
            final_tolerance_multiplier: 迭代耗尽后的放宽验收倍数，保持旧逻辑。
        """
        if tolerance <= 0.0:
            raise ValueError("tolerance must be greater than zero")
        if max_iterations <= 0:
            raise ValueError("max_iterations must be greater than zero")
        if linear_speed <= 0.0:
            raise ValueError("linear_speed must be greater than zero")
        if max_step_distance <= 0.0:
            raise ValueError("max_step_distance must be greater than zero")

        start_pose = self.get_current_pose()
        if start_pose is None:
            rospy.logerr("无法读取起始位姿，拒绝执行相对移动")
            return False

        start_theta = start_pose["theta"]
        cos_theta = math.cos(start_theta)
        sin_theta = math.sin(start_theta)
        target_x = (
            start_pose["x"] + forward * cos_theta - lateral * sin_theta
        )
        target_y = (
            start_pose["y"] + forward * sin_theta + lateral * cos_theta
        )

        try:
            self.ensure_manual_mode()
        except RuntimeError as exc:
            rospy.logerr("切换手动模式失败，拒绝移动: %s", exc)
            return False

        rospy.loginfo(
            "相对移动目标: forward=%.3fm lateral=%.3fm, map=(%.3f, %.3f)",
            forward, lateral, target_x, target_y,
        )

        for iteration in range(max_iterations):
            if rospy.is_shutdown():
                return False

            pose = self.get_current_pose()
            if pose is None:
                rospy.logwarn("第 %d 次迭代无法读取位姿", iteration + 1)
                rospy.sleep(settle_time)
                continue

            dx = target_x - pose["x"]
            dy = target_y - pose["y"]
            distance = math.hypot(dx, dy)
            if distance <= tolerance:
                rospy.loginfo("相对移动完成，最终位置误差 %.3fm", distance)
                return True

            current_cos = math.cos(pose["theta"])
            current_sin = math.sin(pose["theta"])
            error_forward = dx * current_cos + dy * current_sin
            error_lateral = -dx * current_sin + dy * current_cos

            rospy.loginfo(
                "相对移动迭代 %d/%d: error_forward=%.3fm, "
                "error_lateral=%.3fm, distance=%.3fm",
                iteration + 1, max_iterations,
                error_forward, error_lateral, distance,
            )

            # 与旧程序相同，先修正前后方向，再修正左右方向。每次只执行一个
            # 方向，下一次迭代重新读取 AMCL，避免连续开环动作累积误差。
            if abs(error_forward) > tolerance:
                pulse_distance = min(abs(error_forward), max_step_distance)
                velocity = linear_speed if error_forward > 0.0 else -linear_speed
                try:
                    self._send_velocity_pulse(
                        vx=velocity,
                        duration=pulse_distance / linear_speed,
                    )
                except RuntimeError as exc:
                    rospy.logerr("前后移动命令失败: %s", exc)
                    return False
            elif abs(error_lateral) > tolerance:
                pulse_distance = min(abs(error_lateral), max_step_distance)
                velocity = linear_speed if error_lateral > 0.0 else -linear_speed
                try:
                    self._send_velocity_pulse(
                        vy=velocity,
                        duration=pulse_distance / linear_speed,
                    )
                except RuntimeError as exc:
                    rospy.logerr("横移命令失败: %s", exc)
                    return False

            rospy.sleep(settle_time)

        final_pose = self.get_current_pose()
        if final_pose is None:
            return False
        final_error = math.hypot(
            target_x - final_pose["x"],
            target_y - final_pose["y"],
        )
        accepted = final_error <= tolerance * final_tolerance_multiplier
        if accepted:
            rospy.logwarn("相对移动以放宽容差完成，最终误差 %.3fm", final_error)
        else:
            rospy.logerr("相对移动未达到目标，最终误差 %.3fm", final_error)
        return accepted

    def fine_move_forward_lateral(
            self, forward_distance: float = 0.0,
            lateral_distance: float = 0.0,
            tolerance: float = 0.01,
            max_iterations: int = 12) -> bool:
        """兼容旧程序函数名，内部调用 `move_relative()`。"""
        return self.move_relative(
            forward=forward_distance,
            lateral=lateral_distance,
            tolerance=tolerance,
            max_iterations=max_iterations,
        )

    def rotate_relative(
            self,
            angle: float,
            tolerance: float = 0.03,
            max_iterations: int = 12,
            angular_speed: float = 0.05,
            max_step_angle: float = 0.15,
            settle_time: float = 0.5) -> bool:
        """根据 AMCL theta 闭环旋转；正值左转，负值右转。"""
        if tolerance <= 0.0:
            raise ValueError("tolerance must be greater than zero")
        if angular_speed <= 0.0:
            raise ValueError("angular_speed must be greater than zero")
        if max_step_angle <= 0.0:
            raise ValueError("max_step_angle must be greater than zero")

        start_pose = self.get_current_pose()
        if start_pose is None:
            rospy.logerr("无法读取起始位姿，拒绝旋转")
            return False
        target_theta = normalize_angle(start_pose["theta"] + angle)

        try:
            self.ensure_manual_mode()
        except RuntimeError as exc:
            rospy.logerr("切换手动模式失败，拒绝旋转: %s", exc)
            return False

        for iteration in range(max_iterations):
            pose = self.get_current_pose()
            if pose is None:
                rospy.sleep(settle_time)
                continue

            error = normalize_angle(target_theta - pose["theta"])
            if abs(error) <= tolerance:
                rospy.loginfo("旋转完成，最终角度误差 %.3frad", abs(error))
                return True

            pulse_angle = min(abs(error), max_step_angle)
            velocity = angular_speed if error > 0.0 else -angular_speed
            rospy.loginfo(
                "旋转迭代 %d/%d: angle_error=%.3frad",
                iteration + 1, max_iterations, error,
            )
            try:
                self._send_velocity_pulse(
                    vw=velocity,
                    duration=pulse_angle / angular_speed,
                )
            except RuntimeError as exc:
                rospy.logerr("旋转命令失败: %s", exc)
                return False
            rospy.sleep(settle_time)

        final_pose = self.get_current_pose()
        if final_pose is None:
            return False
        final_error = abs(normalize_angle(target_theta - final_pose["theta"]))
        rospy.logerr("旋转未达到目标，最终角度误差 %.3frad", final_error)
        return final_error <= tolerance

    # ==================== 路网站点导航 ====================

    def _wait_navigation_started(
            self,
            initial_pose: Dict[str, float],
            timeout: float,
            check_interval: float,
            position_threshold: float,
            angle_threshold: float) -> bool:
        """检测站点任务下发后底盘是否发生了可观测运动。"""
        start_time = time.monotonic()
        while time.monotonic() - start_time < timeout and not rospy.is_shutdown():
            pose = self.get_current_pose()
            if pose is not None:
                moved = math.hypot(
                    pose["x"] - initial_pose["x"],
                    pose["y"] - initial_pose["y"],
                )
                rotated = abs(normalize_angle(
                    pose["theta"] - initial_pose["theta"]
                ))
                if moved > position_threshold or rotated > angle_threshold:
                    rospy.loginfo(
                        "检测到导航开始: 位移 %.3fm，转角 %.3frad",
                        moved, rotated,
                    )
                    return True
            rospy.sleep(check_interval)
        return False

    def _wait_navigation_stable(
            self,
            timeout: float,
            stable_time: float,
            check_interval: float,
            position_threshold: float,
            angle_threshold: float) -> bool:
        """位置和朝向连续稳定指定时间后，认为站点导航结束。"""
        pose = self.get_current_pose()
        if pose is None:
            return False

        anchor_pose = pose
        last_change_time = time.monotonic()
        start_time = time.monotonic()

        while time.monotonic() - start_time < timeout and not rospy.is_shutdown():
            rospy.sleep(check_interval)
            pose = self.get_current_pose()
            if pose is None:
                continue

            moved = math.hypot(
                pose["x"] - anchor_pose["x"],
                pose["y"] - anchor_pose["y"],
            )
            rotated = abs(normalize_angle(
                pose["theta"] - anchor_pose["theta"]
            ))
            if moved > position_threshold or rotated > angle_threshold:
                anchor_pose = pose
                last_change_time = time.monotonic()

            stable_duration = time.monotonic() - last_change_time
            rospy.loginfo(
                "导航监控: x=%.3f y=%.3f theta=%.3f，稳定 %.1fs",
                pose["x"], pose["y"], pose["theta"], stable_duration,
            )
            if stable_duration >= stable_time:
                return True
        return False

    def navigate_to_station(
            self,
            station_name: str,
            timeout: float = 120.0,
            initial_check_time: float = 15.0,
            stable_time: float = 5.0,
            check_interval: float = 0.5,
            position_threshold: float = 0.01,
            angle_threshold: float = 0.03,
            max_retries: int = 3,
            extra_forward: float = 0.0,
            request_id: str = "1") -> bool:
        """切到 AUTO，按路网站点名导航，并通过 AMCL 变化等待任务结束。

        HTTP 成功只说明命令被接收，所以这里沿用旧程序的两阶段监控：先确认机器人
        发生运动，再等待位置和朝向连续稳定。由于当前没有可读的厂商导航结果消息，
        “稳定”仍可能是到站、受阻或导航失败，正式运行时需保留超时和人工急停。
        """
        station_name = station_name.strip()
        if not station_name:
            raise ValueError("station_name must not be empty")
        if max_retries <= 0:
            raise ValueError("max_retries must be greater than zero")

        for attempt in range(1, max_retries + 1):
            initial_pose = self.get_current_pose()
            if initial_pose is None:
                rospy.logerr("无法读取导航前位姿（尝试 %d/%d）", attempt, max_retries)
                continue

            try:
                self.ensure_auto_mode(request_id=request_id)
                result = self.http.dispatch_goal_node_name(
                    station_name,
                    request_id=request_id,
                )
            except RuntimeError as exc:
                rospy.logerr("站点导航命令异常（尝试 %d/%d）: %s", attempt, max_retries, exc)
                continue

            if self._command_rejected(result):
                rospy.logerr(
                    "站点导航被拒绝（尝试 %d/%d）: %s",
                    attempt, max_retries, result,
                )
                continue

            rospy.loginfo(
                "已下发站点导航 %s（尝试 %d/%d）",
                station_name, attempt, max_retries,
            )
            if not self._wait_navigation_started(
                    initial_pose=initial_pose,
                    timeout=initial_check_time,
                    check_interval=check_interval,
                    position_threshold=position_threshold,
                    angle_threshold=angle_threshold):
                rospy.logwarn("未检测到导航启动（尝试 %d/%d）", attempt, max_retries)
                continue

            if not self._wait_navigation_stable(
                    timeout=timeout,
                    stable_time=stable_time,
                    check_interval=check_interval,
                    position_threshold=position_threshold,
                    angle_threshold=angle_threshold):
                rospy.logwarn("导航未在超时时间内稳定（尝试 %d/%d）", attempt, max_retries)
                continue

            rospy.loginfo("站点 %s 导航结束", station_name)
            if abs(extra_forward) > 0.001:
                return self.move_relative(forward=extra_forward)
            return True

        rospy.logerr("站点 %s 导航失败", station_name)
        return False
