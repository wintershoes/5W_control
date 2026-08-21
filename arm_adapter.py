#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo 5-W 机械臂关节控制适配器。

运行本程序前，必须先在下位机 ``lab@机器人下位机IP`` 启动本体控制程序：

    cd ~/kuavo-ros-opensource
    sudo su
    source devel/setup.bash
    roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=h12

本适配器通常在上位机运行，只能使用下位机启动后提供的 ROS 话题和服务；
上位机本地没有 ``humanoid_controllers`` 包，不能代替下位机执行上述 launch。

当前机械臂尚未做首次实机动作测试。正式运行前必须逐项核对：

1. ``/humanoid_controller/real_launch_status`` 和
   ``/humanoid_controller/real_initial_start`` 服务均由下位机本体节点提供；
2. ``/sensors_data_raw`` 有本体数据发布者，且数据持续更新；
3. ``/kuavo_arm_traj`` 的类型为 ``sensor_msgs/JointState``，并且存在真正通向
   ``MobileManipulatorReferenceManager`` 的订阅者，不能只有 ``joy_node`` 发布者；
4. ``/mobile_manipulator_mpc_control`` 服务，以及
   ``/lb_arm_joint_reach_time/left``、``/lb_arm_joint_reach_time/right`` 发布者
   会在进入 stance 后正常出现；
5. 当前代码假设 14 个关节的顺序、单位（度）与 ``ARM_JOINT_NAMES`` 完全一致，
   必须通过 v63 实机接口或供应商提供的安全姿态再次确认；
6. 从旧机器人迁移的 READY/RETRACT 姿态尚未验证零位、正负方向、机械限位、
   自碰撞和与桌面/底盘的间隙，第一次不得直接执行完整三段式动作；
7. 先由技术人员提供一个接近当前姿态且确认安全的 14 关节目标，做单帧小幅测试，
   再逐段验证 READY_POSE_1、READY_POSE_2、READY_POSE_3 和 RETRACT_POSE；
8. 确认没有 G12、网页、SDK 或其他节点同时向机械臂发布目标；测试时急停可触及，
   机械臂工作区内无人且无障碍物；
9. ``real_initial_start`` 会让控制器接管机械臂并执行 stance 初始化预动作，调用前
   必须保证机器人具备安全运动条件，不能把它当成纯状态切换服务。

源码确认的正式链路：

    /kuavo_arm_traj (sensor_msgs/JointState，14 个关节，单位为度)
        -> MobileManipulatorReferenceManager
        -> 左右臂 Ruckig 轨迹规划
        -> 轮臂 MPC / WBC
        -> /joint_cmd
        -> HardwareNodeletCXX

本适配器不会主动切换 `/mobile_manipulator_mpc_control`。当前机器人源码虽然
保留该服务，但实际 reference 更新固定走 BaseArm 路径，且 `/kuavo_arm_traj`
会自动切换左右臂为关节空间控制。真实发布前仍会确认本体传感器、MPC 服务、
关节目标订阅者以及左右臂 Ruckig 时间发布者均在线。

当 ``execute=True`` 时，首次真实发布前还会自动查询本体启动状态，并在
``ready_stance`` 或 ``launched`` 状态调用 ``real_initial_start`` 进入 stance。
该步骤会使控制器接管机械臂并执行本体预设初始化动作；dry-run 不会调用服务。

旧程序姿态尚未确认适配新 5-W 的零位、方向和限位，因此默认 dry-run；真实
发布旧姿态必须显式允许 `allow_unverified_poses`。夹爪、高度和视觉检测不在
本文件中实现。
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence

import rosgraph
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


ARM_TOPIC = "/kuavo_arm_traj"
LEFT_REACH_TIME_TOPIC = "/lb_arm_joint_reach_time/left"
RIGHT_REACH_TIME_TOPIC = "/lb_arm_joint_reach_time/right"
SENSORS_TOPIC = "/sensors_data_raw"
MPC_CONTROL_SERVICE = "/mobile_manipulator_mpc_control"
REAL_LAUNCH_STATUS_SERVICE = "/humanoid_controller/real_launch_status"
REAL_INITIAL_START_SERVICE = "/humanoid_controller/real_initial_start"

ARM_JOINT_NAMES = (
    "l_arm_pitch",
    "l_arm_roll",
    "l_arm_yaw",
    "l_forearm_pitch",
    "l_hand_yaw",
    "l_hand_pitch",
    "l_hand_roll",
    "r_arm_pitch",
    "r_arm_roll",
    "r_arm_yaw",
    "r_forearm_pitch",
    "r_hand_yaw",
    "r_hand_pitch",
    "r_hand_roll",
)


@dataclass(frozen=True)
class ArmPose:
    """一帧 14 关节机械臂目标姿态，单位为度。"""

    name: str
    joints: Sequence[float]
    old_duration: float = 3.0
    wait_after: float = 1.8
    verified: bool = False

    def validate(self) -> None:
        if len(self.joints) != len(ARM_JOINT_NAMES):
            raise ValueError("{} must contain exactly 14 joint values".format(self.name))
        if not all(math.isfinite(float(value)) for value in self.joints):
            raise ValueError("{} contains a non-finite joint value".format(self.name))
        if self.old_duration < 0.0 or self.wait_after < 0.0:
            raise ValueError("{} contains a negative wait duration".format(self.name))


class ArmAdapter:
    """新机器人机械臂关节控制适配器。"""

    READY_POSE_1 = ArmPose(
        "READY_POSE_1",
        [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 20.0, -60.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    READY_POSE_2 = ArmPose(
        "READY_POSE_2",
        [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, -10.0, -60.0, 0.0, -90.0, 35.0, 20.0, 0.0],
    )
    READY_POSE_3 = ArmPose(
        "READY_POSE_3",
        [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, -75.0, -7.0, 50.0, -3.0, 39.0, 70.5, 2.0],
    )
    READY_POSE_3_DISK = ArmPose(
        "READY_POSE_3_DISK",
        [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, -75.0, -7.0, 50.0, -3.0, 40.0, 70.5, 2.0],
    )
    RETRACT_POSE = ArmPose(
        "RETRACT_POSE",
        [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0, -75.0, -7.0, 50.0, -3.0, 40.0, 70.5, 2.0],
    )

    # 旧视觉微调使用的右臂关节索引，方向和限位仍需在新机器人上逐项确认。
    RIGHT_SHOULDER_X_INDEX = 7
    RIGHT_LATERAL_INDEX_A = 8
    RIGHT_LATERAL_INDEX_B = 13
    RIGHT_HEIGHT_INDEX = 12

    def __init__(
        self,
        execute: bool = False,
        arm_topic: str = ARM_TOPIC,
        left_reach_time_topic: str = LEFT_REACH_TIME_TOPIC,
        right_reach_time_topic: str = RIGHT_REACH_TIME_TOPIC,
        sensors_topic: str = SENSORS_TOPIC,
        mpc_control_service: str = MPC_CONTROL_SERVICE,
        allow_unverified_poses: bool = False,
        wait_for_control_path_timeout: float = 5.0,
        stance_initialization_timeout: float = 30.0,
        reach_feedback_timeout: float = 2.0,
        reach_time_margin: float = 0.25,
    ):
        self.execute = execute
        self.arm_topic = arm_topic
        self.left_reach_time_topic = left_reach_time_topic
        self.right_reach_time_topic = right_reach_time_topic
        self.sensors_topic = sensors_topic
        self.mpc_control_service = mpc_control_service
        self.allow_unverified_poses = allow_unverified_poses
        self.wait_for_control_path_timeout = max(0.0, wait_for_control_path_timeout)
        self.stance_initialization_timeout = max(0.0, stance_initialization_timeout)
        self.reach_feedback_timeout = max(0.0, reach_feedback_timeout)
        self.reach_time_margin = max(0.0, reach_time_margin)

        self.current_arm_pose: Optional[List[float]] = None
        self._stance_initialized = False
        self._stance_lock = threading.Lock()
        self._reach_condition = threading.Condition()
        self._reach_times: Dict[str, Optional[float]] = {"left": None, "right": None}

        self._publisher = rospy.Publisher(self.arm_topic, JointState, queue_size=10)
        self._left_reach_subscriber = rospy.Subscriber(
            self.left_reach_time_topic,
            Float32,
            self._left_reach_time_callback,
            queue_size=10,
        )
        self._right_reach_subscriber = rospy.Subscriber(
            self.right_reach_time_topic,
            Float32,
            self._right_reach_time_callback,
            queue_size=10,
        )

    def readiness(self) -> dict:
        """只读查询 ROS 拓扑，返回机械臂控制链路是否完整。"""
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
        arm_subscribers = subscriber_map.get(self.arm_topic, [])
        sensor_publishers = publisher_map.get(self.sensors_topic, [])
        left_reach_publishers = publisher_map.get(self.left_reach_time_topic, [])
        right_reach_publishers = publisher_map.get(self.right_reach_time_topic, [])
        arm_topic_type = topic_types.get(self.arm_topic)
        publisher_connections = self._publisher.get_num_connections()

        reasons = []
        if not arm_subscribers:
            reasons.append("arm_command_has_no_subscriber")
        if arm_topic_type != "sensor_msgs/JointState":
            reasons.append("arm_command_type_mismatch")
        if arm_subscribers and publisher_connections <= 0:
            reasons.append("arm_publisher_not_connected")
        if not sensor_publishers:
            reasons.append("sensors_data_raw_has_no_publisher")
        if not left_reach_publishers:
            reasons.append("left_reach_time_has_no_publisher")
        if not right_reach_publishers:
            reasons.append("right_reach_time_has_no_publisher")
        if self.mpc_control_service not in service_map:
            reasons.append("mpc_reference_manager_service_unavailable")

        return {
            "ready": not reasons,
            "reasons": reasons,
            "execute": self.execute,
            "arm_topic": self.arm_topic,
            "arm_topic_type": arm_topic_type,
            "arm_subscribers": arm_subscribers,
            "publisher_connections": publisher_connections,
            "sensors_topic": self.sensors_topic,
            "sensor_publishers": sensor_publishers,
            "left_reach_time_topic": self.left_reach_time_topic,
            "left_reach_publishers": left_reach_publishers,
            "right_reach_time_topic": self.right_reach_time_topic,
            "right_reach_publishers": right_reach_publishers,
            "mpc_control_service": self.mpc_control_service,
            "mpc_service_nodes": service_map.get(self.mpc_control_service, []),
            "mpc_mode_note": "当前源码固定执行并反馈 BaseArm(3)，不主动切换 ArmOnly",
            "stance_initialized_in_process": self._stance_initialized,
            "allow_unverified_poses": self.allow_unverified_poses,
        }

    def startup_diagnostics(self, query_launch_status: bool = True) -> dict:
        """只读诊断下位机本体程序和机械臂控制链路。

        ``real_launch_status`` 是只读状态服务。该方法不会调用
        ``real_initial_start``、不会进入 stance，也不会发布机械臂目标。
        控制链路在 ``ready_stance/launched`` 阶段可能尚未完整出现，因此分别
        返回 ``body_program_ready`` 与 ``control_path_ready``。
        """
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
        launch_status_error = None
        if query_launch_status and status_nodes:
            try:
                result = rospy.ServiceProxy(
                    REAL_LAUNCH_STATUS_SERVICE,
                    Trigger,
                )()
                launch_status = {
                    "success": bool(result.success),
                    "state": result.message.strip().lower(),
                    "raw_message": result.message,
                }
                if not result.success:
                    reasons.append("real_launch_status_reported_failure")
            except rospy.ServiceException as exc:
                launch_status_error = repr(exc)
                reasons.append("real_launch_status_query_failed")

        body_program_ready = bool(status_nodes and initial_start_nodes)
        if launch_status is not None:
            body_program_ready = body_program_ready and launch_status["success"]

        return {
            "body_program_ready": body_program_ready,
            "control_path_ready": bool(control_path.get("ready")),
            "reasons": reasons,
            "launch_status_service": REAL_LAUNCH_STATUS_SERVICE,
            "launch_status_nodes": status_nodes,
            "initial_start_service": REAL_INITIAL_START_SERVICE,
            "initial_start_nodes": initial_start_nodes,
            "launch_status": launch_status,
            "launch_status_error": launch_status_error,
            "control_path": control_path,
            "note": (
                "ready_stance/launched 表示本体已启动但尚可未进入 stance；"
                "只有 control_path_ready=true 才表示机械臂动作链路完整"
            ),
        }

    def wait_until_ready(self, timeout: Optional[float] = None) -> bool:
        """等待本体机械臂控制链路出现；只查询，不发送模式或运动指令。"""
        wait_timeout = self.wait_for_control_path_timeout if timeout is None else max(0.0, timeout)
        deadline = time.monotonic() + wait_timeout
        while not rospy.is_shutdown():
            status = self.readiness()
            if status.get("ready"):
                return True
            if time.monotonic() >= deadline:
                rospy.logerr("机械臂控制链路未就绪: %s", status)
                return False
            rospy.logwarn_throttle(1.0, "等待机械臂控制链路: %s", status.get("reasons"))
            rospy.sleep(0.1)
        return False

    def ensure_stance_ready(self) -> bool:
        """在真实执行前进入 stance，并等待机械臂控制链路完整就绪。

        已经处于可控制状态时不会重复调用初始化服务。该方法只负责从本体程序
        已加载后的 ready_stance/launched 推进到 stance，不会跨机器启动下位机的
        ``load_kuavo_real_wheel.launch``。
        """
        if not self.execute:
            rospy.loginfo("[dry-run] 不调用本体 stance 初始化服务")
            return True

        with self._stance_lock:
            status = self.readiness()
            if status.get("ready"):
                if not self._stance_initialized:
                    rospy.loginfo("机械臂控制链路已经就绪，不重复执行 stance 初始化")
                self._stance_initialized = True
                return True

            if self._stance_initialized:
                rospy.logerr("stance 曾初始化成功，但机械臂控制链路已失效: %s", status)
                return False

            try:
                rospy.wait_for_service(
                    REAL_LAUNCH_STATUS_SERVICE,
                    timeout=self.wait_for_control_path_timeout,
                )
                launch_status = rospy.ServiceProxy(
                    REAL_LAUNCH_STATUS_SERVICE,
                    Trigger,
                )()
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logerr(
                    "无法查询本体启动状态 %s: %s。请先在下位机启动 "
                    "load_kuavo_real_wheel.launch",
                    REAL_LAUNCH_STATUS_SERVICE,
                    exc,
                )
                return False

            launch_state = launch_status.message.strip().lower()
            rospy.loginfo(
                "本体启动状态: success=%s, state=%s",
                launch_status.success,
                launch_state or "<empty>",
            )
            if not launch_status.success or launch_state not in ("ready_stance", "launched"):
                rospy.logerr(
                    "当前状态不能进入 stance: success=%s, state=%s",
                    launch_status.success,
                    launch_state or "<empty>",
                )
                return False

            rospy.logwarn(
                "即将调用 %s：控制器会接管机械臂并执行初始化预动作",
                REAL_INITIAL_START_SERVICE,
            )
            try:
                rospy.wait_for_service(
                    REAL_INITIAL_START_SERVICE,
                    timeout=self.wait_for_control_path_timeout,
                )
                initialize_result = rospy.ServiceProxy(
                    REAL_INITIAL_START_SERVICE,
                    Trigger,
                )()
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logerr("调用 stance 初始化服务失败: %s", exc)
                return False

            if not initialize_result.success:
                rospy.logerr(
                    "stance 初始化被本体拒绝: %s",
                    initialize_result.message or "未提供原因",
                )
                return False

            rospy.loginfo(
                "stance 初始化请求已接受: %s",
                initialize_result.message or "success",
            )
            if not self.wait_until_ready(timeout=self.stance_initialization_timeout):
                rospy.logerr("进入 stance 后机械臂控制链路未在超时时间内就绪")
                return False

            self._stance_initialized = True
            rospy.loginfo("stance 初始化完成，机械臂控制链路已就绪")
            return True

    def publish_joint_pose(
        self,
        pose: ArmPose,
        wait: bool = True,
        fallback_duration: Optional[float] = None,
    ) -> bool:
        """发布一帧 14 关节角度，并按左右臂 Ruckig 预计时间等待。
        注意这里是pose是自己定义的，还增加了是否经过验证的标记。
        """
        pose.validate()
        fallback = pose.old_duration if fallback_duration is None else float(fallback_duration)
        if fallback < 0.0:
            raise ValueError("fallback_duration must be non-negative")

        rospy.loginfo("机械臂目标: %s", pose.name)
        rospy.loginfo("关节角度(度): %s", [round(float(v), 3) for v in pose.joints])

        if not self.execute:
            rospy.loginfo("[dry-run] 不发布 %s", self.arm_topic)
            self.current_arm_pose = list(pose.joints)
            return True

        if not pose.verified and not self.allow_unverified_poses:
            rospy.logerr(
                "%s 是旧机器人姿态，尚未验证；真实执行需加 --allow-unverified-poses",
                pose.name,
            )
            return False
        if not self.ensure_stance_ready():
            return False

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(ARM_JOINT_NAMES)
        msg.position = [float(value) for value in pose.joints]
        msg.velocity = [0.0] * len(ARM_JOINT_NAMES)
        msg.effort = [0.0] * len(ARM_JOINT_NAMES)

        self._reset_reach_times()
        self._publisher.publish(msg)
        rospy.loginfo("已发布 %s 到 %s", pose.name, self.arm_topic)

        if wait and not self._wait_for_motion_finish(fallback, pose.wait_after):
            return False

        self.current_arm_pose = list(pose.joints)
        return True

    def move_to_ready_position(self, disk: bool = False) -> bool:
        """执行旧程序三段式 READY_POSE_1 -> READY_POSE_2 -> READY_POSE_3。"""
        final_pose = self.READY_POSE_3_DISK if disk else self.READY_POSE_3
        for pose in (self.READY_POSE_1, self.READY_POSE_2, final_pose):
            if not self.publish_joint_pose(pose, wait=True):
                return False
        return True

    def retract(self) -> bool:
        """收回到旧程序 RETRACT_POSE。"""
        return self.publish_joint_pose(self.RETRACT_POSE, wait=True)

    def apply_joint_delta(
        self,
        deltas: dict,
        name: str = "VISION_ADJUST",
        duration: float = 2.0,
    ) -> bool:
        """在最近一次成功下发的姿态上叠加微调量。"""
        if self.current_arm_pose is None:
            raise RuntimeError("必须先成功下发一个基准姿态，才能执行视觉关节微调")
        joints = list(self.current_arm_pose)
        for index, delta in deltas.items():
            if index < 0 or index >= len(joints):
                raise IndexError("joint index out of range: {}".format(index))
            joints[index] += float(delta)
        pose = ArmPose(
            name=name,
            joints=joints,
            old_duration=duration,
            wait_after=0.0,
            verified=True,
        )
        return self.publish_joint_pose(pose, wait=True, fallback_duration=duration)

    def preview_old_pick_sequence(self, disk: bool = False) -> None:
        """打印旧程序机械臂动作顺序，不发送指令。"""
        final_name = "READY_POSE_3_DISK" if disk else "READY_POSE_3"
        rospy.loginfo("旧机械臂顺序: READY_POSE_1 -> READY_POSE_2 -> %s", final_name)
        rospy.loginfo("随后视觉会小幅修改最近一次成功下发的 14 关节数组")
        rospy.loginfo("抓取下降/抬升原来由升降机构完成，本 adapter 暂不处理高度")
        rospy.loginfo("夹爪原来单独调用服务，本 adapter 暂不处理夹爪")

    def _left_reach_time_callback(self, msg: Float32) -> None:
        '''这里的两个reach_time是机械臂规划程序，计算出来的机械臂执行时间。'''
        self._record_reach_time("left", msg.data)

    def _right_reach_time_callback(self, msg: Float32) -> None:
        self._record_reach_time("right", msg.data)

    def _record_reach_time(self, side: str, value: float) -> None:
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            rospy.logwarn("忽略无效的%s臂 Ruckig 时间: %s", side, value)
            return
        with self._reach_condition:
            self._reach_times[side] = value
            self._reach_condition.notify_all()

    def _reset_reach_times(self) -> None:
        with self._reach_condition:
            self._reach_times = {"left": None, "right": None}

    def _wait_for_motion_finish(self, fallback_duration: float, wait_after: float) -> bool:
        deadline = time.monotonic() + self.reach_feedback_timeout
        with self._reach_condition:
            while not rospy.is_shutdown():
                left = self._reach_times["left"]
                right = self._reach_times["right"]
                if left is not None and right is not None:
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._reach_condition.wait(timeout=min(0.1, remaining))
            left = self._reach_times["left"]
            right = self._reach_times["right"]

        if left is None or right is None:
            # 不继续执行下一段姿态，避免未知时长的动作互相覆盖。
            conservative_wait = fallback_duration + wait_after + self.reach_time_margin
            rospy.logerr(
                "未收到完整 Ruckig 时间反馈(left=%s, right=%s)，保守等待 %.2fs 后中止序列",
                left,
                right,
                conservative_wait,
            )
            rospy.sleep(conservative_wait)
            return False

        planned_duration = max(left, right)
        total_wait = planned_duration + wait_after + self.reach_time_margin
        rospy.loginfo(
            "Ruckig 预计时间: left=%.3fs, right=%.3fs；等待 %.3fs",
            left,
            right,
            total_wait,
        )
        rospy.sleep(total_wait)
        return not rospy.is_shutdown()


def parse_joint_csv(value: str) -> List[float]:
    """解析命令行中的 14 个逗号分隔关节角。"""
    joints = [float(item.strip()) for item in value.split(",") if item.strip()]
    if len(joints) != len(ARM_JOINT_NAMES):
        raise ValueError("--joints must contain exactly 14 comma-separated values")
    if not all(math.isfinite(item) for item in joints):
        raise ValueError("--joints contains a non-finite value")
    return joints


def format_joint_csv(joints: Iterable[float]) -> str:
    return ",".join("{:.3f}".format(float(value)) for value in joints)
