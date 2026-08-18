#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo 5-W v63 躯干高度控制适配器。

运行真实动作前，必须先在下位机启动本体控制程序：

    cd ~/kuavo-ros-opensource
    sudo su
    source devel/setup.bash
    roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=h12

本适配器通常在上位机运行。正式控制链路由本地下载的 v63 源码确认：

    /cmd_lb_torso_pose (geometry_msgs/Twist)
        -> MobileManipulatorReferenceManager
        -> 躯干 4 自由度 Ruckig 轨迹规划 [x, z, yaw, pitch]
        -> 轮臂 MPC / WBC
        -> 腿部和腰部关节

``linear.z`` 是躯干目标 Z 位置，单位为米，不是速度或增量。程序先调用只读服务
``/mobile_manipulator_get_torso_initial_pose`` 获取本机初始躯干位姿，再使用
``初始 Z + offset`` 形成目标。为保持与 v63 手柄源码一致，默认只允许 offset 位于
0.0~0.32 m；不得把旧升降台的 0.21/0.45 m 绝对高度直接传入本接口。

安全说明：

1. 默认 ``execute=False``，只诊断和打印目标，不发布运动指令；
2. 真实发布前检查 ROBOT_VERSION、taskFile、命令订阅者、初始位姿服务、MPC
   observation、传感器和 Ruckig 时间反馈；
3. ``/lb_torso_pose_reach_time`` 返回的是规划器预计运动时间，不是实测到位信号；
4. 当前公开接口没有直接返回实时躯干 Z 的专用服务，因此首次测试必须现场观察，
   使用很小的正偏移，急停可触及，并确认腿部、腰部和机械臂周围有足够空间；
5. 不要同时使用 H12 躯干模式、网页或其他程序发布 ``/cmd_lb_torso_pose``；
6. ``--execute`` 不能跨机器启动下位机 launch。若本体尚未进入 stance，本适配器
   会沿用机械臂适配器的策略调用本体初始化服务，然后等待高度控制链路上线。
"""

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional

import rosgraph
import rospy
from geometry_msgs.msg import Twist
from kuavo_msgs.srv import changeTorsoCtrlMode, getLbTorsoInitialPose
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


TORSO_COMMAND_TOPIC = "/cmd_lb_torso_pose"
TORSO_REACH_TIME_TOPIC = "/lb_torso_pose_reach_time"
INITIAL_POSE_SERVICE = "/mobile_manipulator_get_torso_initial_pose"
GET_MPC_MODE_SERVICE = "/mobile_manipulator_get_mpc_control_mode"
MPC_CONTROL_SERVICE = "/mobile_manipulator_mpc_control"
MPC_OBSERVATION_TOPIC = "/mobile_manipulator_mpc_observation"
SENSORS_TOPIC = "/sensors_data_raw"
REAL_LAUNCH_STATUS_SERVICE = "/humanoid_controller/real_launch_status"
REAL_INITIAL_START_SERVICE = "/humanoid_controller/real_initial_start"


@dataclass(frozen=True)
class TorsoPose:
    """躯干相对底盘的初始/目标位姿，平移单位 m，角度单位 rad。"""

    x: float
    y: float
    z: float
    yaw: float
    pitch: float
    roll: float

    def validate(self) -> None:
        values = (self.x, self.y, self.z, self.yaw, self.pitch, self.roll)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("torso pose contains a non-finite value")

    def as_dict(self) -> Dict[str, float]:
        return {
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "yaw": float(self.yaw),
            "pitch": float(self.pitch),
            "roll": float(self.roll),
        }


class HeightAdapter:
    """以初始躯干位姿为基准控制 Kuavo 5-W v63 高度。"""

    def __init__(
        self,
        execute: bool = False,
        command_topic: str = TORSO_COMMAND_TOPIC,
        reach_time_topic: str = TORSO_REACH_TIME_TOPIC,
        initial_pose_service: str = INITIAL_POSE_SERVICE,
        get_mpc_mode_service: str = GET_MPC_MODE_SERVICE,
        expected_robot_version: int = 63,
        min_offset: float = 0.0,
        max_offset: float = 0.32,
        service_timeout: float = 3.0,
        control_path_timeout: float = 5.0,
        stance_initialization_timeout: float = 30.0,
        reach_feedback_timeout: float = 2.0,
        reach_time_margin: float = 0.3,
    ):
        self.execute = bool(execute)
        self.command_topic = command_topic
        self.reach_time_topic = reach_time_topic
        self.initial_pose_service = initial_pose_service
        self.get_mpc_mode_service = get_mpc_mode_service
        self.expected_robot_version = int(expected_robot_version)
        self.min_offset = float(min_offset)
        self.max_offset = float(max_offset)
        self.service_timeout = max(0.0, float(service_timeout))
        self.control_path_timeout = max(0.0, float(control_path_timeout))
        self.stance_initialization_timeout = max(0.0, float(stance_initialization_timeout))
        self.reach_feedback_timeout = max(0.0, float(reach_feedback_timeout))
        self.reach_time_margin = max(0.0, float(reach_time_margin))

        if not math.isfinite(self.min_offset) or not math.isfinite(self.max_offset):
            raise ValueError("height offset bounds must be finite")
        if self.min_offset > self.max_offset:
            raise ValueError("min_offset must not exceed max_offset")

        self._initial_pose: Optional[TorsoPose] = None
        self._stance_initialized = False
        self._stance_lock = threading.Lock()
        self._reach_condition = threading.Condition()
        self._reach_generation = 0
        self._latest_reach_time: Optional[float] = None

        self._publisher = rospy.Publisher(self.command_topic, Twist, queue_size=10)
        self._reach_subscriber = rospy.Subscriber(
            self.reach_time_topic,
            Float32,
            self._reach_time_callback,
            queue_size=10,
        )

    def readiness(self) -> dict:
        """只读检查高度控制链路，不调用模式服务或发布运动指令。"""
        try:
            master = rosgraph.Master(rospy.get_name())
            publishers, subscribers, services = master.getSystemState()
            topic_types = dict(master.getTopicTypes())
        except Exception as exc:
            return {
                "ready": False,
                "reasons": ["ros_master_unavailable"],
                "error": repr(exc),
                "execute": self.execute,
            }

        publisher_map = dict(publishers)
        subscriber_map = dict(subscribers)
        service_map = dict(services)

        command_subscribers = subscriber_map.get(self.command_topic, [])
        command_publishers = publisher_map.get(self.command_topic, [])
        reach_publishers = publisher_map.get(self.reach_time_topic, [])
        observation_publishers = publisher_map.get(MPC_OBSERVATION_TOPIC, [])
        sensor_publishers = publisher_map.get(SENSORS_TOPIC, [])
        initial_pose_nodes = service_map.get(self.initial_pose_service, [])
        get_mode_nodes = service_map.get(self.get_mpc_mode_service, [])
        mpc_control_nodes = service_map.get(MPC_CONTROL_SERVICE, [])

        robot_version = rospy.get_param("/robot_version", None)
        task_file = rospy.get_param("/taskFile", None)
        local_env_version = os.environ.get("ROBOT_VERSION")
        version_matches = str(robot_version) == str(self.expected_robot_version)
        task_matches = bool(task_file) and "kuavo_s{}".format(
            self.expected_robot_version
        ) in str(task_file)

        reasons = []
        if not command_subscribers:
            reasons.append("torso_command_has_no_subscriber")
        if topic_types.get(self.command_topic) != "geometry_msgs/Twist":
            reasons.append("torso_command_type_mismatch")
        if command_subscribers and self._publisher.get_num_connections() <= 0:
            reasons.append("torso_publisher_not_connected")
        if not reach_publishers:
            reasons.append("torso_reach_time_has_no_publisher")
        if not initial_pose_nodes:
            reasons.append("initial_torso_pose_service_unavailable")
        if not get_mode_nodes:
            reasons.append("mpc_mode_query_service_unavailable")
        if not mpc_control_nodes:
            reasons.append("mpc_control_service_unavailable")
        if not observation_publishers:
            reasons.append("mpc_observation_has_no_publisher")
        if not sensor_publishers:
            reasons.append("sensors_data_raw_has_no_publisher")
        if robot_version is None:
            reasons.append("robot_version_rosparam_missing")
        elif not version_matches:
            reasons.append("robot_version_mismatch")
        if task_file is None:
            reasons.append("task_file_rosparam_missing")
        elif not task_matches:
            reasons.append("task_file_version_mismatch")

        other_command_publishers = [
            node for node in command_publishers if node != rospy.get_name()
        ]
        return {
            "ready": not reasons,
            "reasons": reasons,
            "execute": self.execute,
            "expected_robot_version": self.expected_robot_version,
            "robot_version_rosparam": robot_version,
            "robot_version_environment": local_env_version,
            "version_matches": version_matches,
            "task_file": task_file,
            "task_file_matches": task_matches,
            "command_topic": self.command_topic,
            "command_topic_type": topic_types.get(self.command_topic),
            "command_subscribers": command_subscribers,
            "other_command_publishers": other_command_publishers,
            "publisher_connections": self._publisher.get_num_connections(),
            "reach_time_topic": self.reach_time_topic,
            "reach_time_publishers": reach_publishers,
            "initial_pose_service": self.initial_pose_service,
            "initial_pose_service_nodes": initial_pose_nodes,
            "get_mpc_mode_service": self.get_mpc_mode_service,
            "get_mpc_mode_service_nodes": get_mode_nodes,
            "mpc_control_service_nodes": mpc_control_nodes,
            "mpc_observation_publishers": observation_publishers,
            "sensor_publishers": sensor_publishers,
            "allowed_offset_m": [self.min_offset, self.max_offset],
            "feedback_note": "reach_time is planned duration, not measured height",
        }

    def startup_diagnostics(self, query_services: bool = True) -> dict:
        """汇总本体状态、控制链路、初始位姿和当前 MPC 模式。"""
        control_path = self.readiness()
        try:
            master = rosgraph.Master(rospy.get_name())
            _, _, services = master.getSystemState()
            service_map = dict(services)
        except Exception as exc:
            return {
                "body_program_ready": False,
                "control_path_ready": False,
                "reasons": ["ros_master_unavailable"],
                "error": repr(exc),
                "control_path": control_path,
            }

        status_nodes = service_map.get(REAL_LAUNCH_STATUS_SERVICE, [])
        initial_start_nodes = service_map.get(REAL_INITIAL_START_SERVICE, [])
        reasons = []
        if not status_nodes:
            reasons.append("real_launch_status_service_unavailable")
        if not initial_start_nodes:
            reasons.append("real_initial_start_service_unavailable")

        launch_status = None
        initial_pose = None
        mpc_mode = None
        query_errors = []
        if query_services and status_nodes:
            try:
                response = rospy.ServiceProxy(REAL_LAUNCH_STATUS_SERVICE, Trigger)()
                launch_status = {
                    "success": bool(response.success),
                    "state": response.message.strip().lower(),
                    "raw_message": response.message,
                }
                if not response.success:
                    reasons.append("real_launch_status_reported_failure")
            except rospy.ServiceException as exc:
                query_errors.append("launch_status: {}".format(repr(exc)))
                reasons.append("real_launch_status_query_failed")

        if query_services and control_path.get("initial_pose_service_nodes"):
            try:
                initial_pose = self.get_initial_pose(refresh=True).as_dict()
            except (rospy.ROSException, rospy.ServiceException, RuntimeError, ValueError) as exc:
                query_errors.append("initial_pose: {}".format(repr(exc)))
                reasons.append("initial_torso_pose_query_failed")

        if query_services and control_path.get("get_mpc_mode_service_nodes"):
            try:
                mpc_mode = self.get_mpc_mode()
            except (rospy.ROSException, rospy.ServiceException) as exc:
                query_errors.append("mpc_mode: {}".format(repr(exc)))
                reasons.append("mpc_mode_query_failed")

        body_program_ready = bool(status_nodes and initial_start_nodes)
        if launch_status is not None:
            body_program_ready = body_program_ready and launch_status["success"]

        return {
            "body_program_ready": body_program_ready,
            "control_path_ready": bool(control_path.get("ready")),
            "reasons": reasons,
            "launch_status": launch_status,
            "initial_torso_pose": initial_pose,
            "mpc_mode": mpc_mode,
            "query_errors": query_errors,
            "control_path": control_path,
        }

    def get_initial_pose(self, refresh: bool = False) -> TorsoPose:
        """通过只读服务取得模型计算的初始躯干位姿。"""
        if self._initial_pose is not None and not refresh:
            return self._initial_pose

        rospy.wait_for_service(self.initial_pose_service, timeout=self.service_timeout)
        response = rospy.ServiceProxy(
            self.initial_pose_service,
            getLbTorsoInitialPose,
        )(True)
        if not response.result:
            raise RuntimeError(response.message or "initial torso pose query rejected")

        pose = TorsoPose(
            x=response.linear.x,
            y=response.linear.y,
            z=response.linear.z,
            yaw=response.angular.z,
            pitch=response.angular.y,
            roll=response.angular.x,
        )
        pose.validate()
        self._initial_pose = pose
        return pose

    def get_mpc_mode(self) -> dict:
        """只读查询 ReferenceManager 保存的 MPC 模式。"""
        rospy.wait_for_service(self.get_mpc_mode_service, timeout=self.service_timeout)
        response = rospy.ServiceProxy(
            self.get_mpc_mode_service,
            changeTorsoCtrlMode,
        )(0)
        return {
            "result": bool(response.result),
            "mode": int(response.mode),
            "message": response.message,
            "source_note": "v63 source currently executes and publishes BaseArm(3)",
        }

    def wait_until_ready(self, timeout: Optional[float] = None) -> bool:
        """等待完整控制链路上线；只读检查。"""
        wait_timeout = self.control_path_timeout if timeout is None else max(0.0, float(timeout))
        deadline = time.monotonic() + wait_timeout
        while not rospy.is_shutdown():
            status = self.readiness()
            if status.get("ready"):
                return True
            if time.monotonic() >= deadline:
                rospy.logerr("高度控制链路未就绪: %s", status)
                return False
            rospy.logwarn_throttle(1.0, "等待高度控制链路: %s", status.get("reasons"))
            rospy.sleep(0.1)
        return False

    def ensure_stance_ready(self) -> bool:
        """真实执行前按需调用本体初始化服务并等待高度控制链路。"""
        if not self.execute:
            rospy.loginfo("[dry-run] 不调用本体 stance 初始化服务")
            return True

        with self._stance_lock:
            if self.readiness().get("ready"):
                self._stance_initialized = True
                return True
            if self._stance_initialized:
                rospy.logerr("stance 曾初始化成功，但高度控制链路已经失效")
                return False

            try:
                rospy.wait_for_service(
                    REAL_LAUNCH_STATUS_SERVICE,
                    timeout=self.control_path_timeout,
                )
                status = rospy.ServiceProxy(REAL_LAUNCH_STATUS_SERVICE, Trigger)()
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logerr("无法查询本体启动状态，请先在下位机启动 wheel launch: %s", exc)
                return False

            state = status.message.strip().lower()
            if not status.success or state not in ("ready_stance", "launched"):
                rospy.logerr("当前本体状态不能进入 stance: success=%s state=%s", status.success, state)
                return False

            rospy.logwarn("即将进入 stance，本体控制器会接管关节并执行初始化预动作")
            try:
                rospy.wait_for_service(
                    REAL_INITIAL_START_SERVICE,
                    timeout=self.control_path_timeout,
                )
                result = rospy.ServiceProxy(REAL_INITIAL_START_SERVICE, Trigger)()
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logerr("调用 stance 初始化服务失败: %s", exc)
                return False
            if not result.success:
                rospy.logerr("stance 初始化被拒绝: %s", result.message)
                return False
            if not self.wait_until_ready(timeout=self.stance_initialization_timeout):
                return False
            self._stance_initialized = True
            return True

    def plan_offset(self, offset_m: float) -> dict:
        """根据实机初始位姿计算目标，不发布命令。"""
        offset = self._validate_offset(offset_m)
        initial = self.get_initial_pose()
        target = TorsoPose(
            x=initial.x,
            y=initial.y,
            z=initial.z + offset,
            yaw=initial.yaw,
            pitch=initial.pitch,
            roll=initial.roll,
        )
        target.validate()
        return {
            "initial_pose": initial.as_dict(),
            "offset_m": offset,
            "target_pose": target.as_dict(),
            "allowed_offset_m": [self.min_offset, self.max_offset],
        }

    def move_to_offset(self, offset_m: float, wait: bool = True) -> bool:
        """移动到 ``初始 Z + offset``；默认 offset 必须位于 0~0.32 m。"""
        if not self.execute:
            plan = self.plan_offset(offset_m)
            rospy.loginfo("躯干高度计划: %s", plan)
            rospy.loginfo("[dry-run] 不发布 %s", self.command_topic)
            return True
        if not self.ensure_stance_ready():
            return False
        if not self.wait_until_ready():
            return False

        # 初始位姿服务可能只在 stance 控制链路完整上线后可用，因此真实执行时
        # 必须在 ensure_stance_ready() 之后重新读取，不能沿用启动前的默认值。
        self.get_initial_pose(refresh=True)
        plan = self.plan_offset(offset_m)
        rospy.loginfo("躯干高度计划: %s", plan)

        target = TorsoPose(**plan["target_pose"])
        msg = Twist()
        msg.linear.x = target.x
        msg.linear.y = target.y
        msg.linear.z = target.z
        msg.angular.x = target.roll
        msg.angular.y = target.pitch
        msg.angular.z = target.yaw

        with self._reach_condition:
            generation_before = self._reach_generation
        self._publisher.publish(msg)
        rospy.loginfo(
            "已发布躯干目标: initial_z=%.6f offset=%.6f target_z=%.6f",
            plan["initial_pose"]["z"],
            plan["offset_m"],
            target.z,
        )
        if not wait:
            return True
        return self._wait_for_planned_motion(generation_before)

    def move_to_initial(self, wait: bool = True) -> bool:
        """回到本体模型的初始躯干高度。"""
        return self.move_to_offset(0.0, wait=wait)

    def _validate_offset(self, offset_m: float) -> float:
        offset = float(offset_m)
        if not math.isfinite(offset):
            raise ValueError("offset must be finite")
        if offset < self.min_offset or offset > self.max_offset:
            raise ValueError(
                "offset {:.6f} outside safe source-derived range [{:.6f}, {:.6f}]".format(
                    offset,
                    self.min_offset,
                    self.max_offset,
                )
            )
        return offset

    def _reach_time_callback(self, msg: Float32) -> None:
        value = float(msg.data)
        if not math.isfinite(value) or value < 0.0:
            rospy.logwarn_throttle(1.0, "忽略无效躯干 Ruckig 时间: %s", value)
            return
        with self._reach_condition:
            self._latest_reach_time = value
            self._reach_generation += 1
            self._reach_condition.notify_all()

    def _wait_for_planned_motion(self, generation_before: int) -> bool:
        deadline = time.monotonic() + self.reach_feedback_timeout
        with self._reach_condition:
            while not rospy.is_shutdown() and self._reach_generation <= generation_before:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    rospy.logerr("未收到新的 %s 规划时间反馈", self.reach_time_topic)
                    return False
                self._reach_condition.wait(timeout=remaining)
            planned_duration = self._latest_reach_time

        if planned_duration is None:
            return False
        rospy.loginfo(
            "Ruckig 预计运动时间 %.3f s；该反馈不是实测到位状态",
            planned_duration,
        )
        rospy.sleep(planned_duration + self.reach_time_margin)
        return not rospy.is_shutdown()
