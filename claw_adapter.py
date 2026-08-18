#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo 5-W v63 乐聚二指夹爪适配器。

已由机器人配置和官方文档确认：

    ROBOT_VERSION=63
    EndEffectorType=["lejuclaw", "lejuclaw"]
    HandProtocolType=proto_can

预期正式链路：

    /control_robot_leju_claw (kuavo_msgs/controlLejuClaw 服务)
        -> 下位机夹爪硬件驱动
        -> /leju_claw_state (kuavo_msgs/lejuClawState 状态话题)

运行真实动作前，必须先在下位机启动本体控制程序：

    cd ~/kuavo-ros-opensource
    sudo su
    source devel/setup.bash
    roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=h12

本文件不会启动上述 launch，也不会自动进入 stance。当前机器人尚未启动本体程序
做运行时核验，因此第一次实际测试前必须只读确认：

1. ``rosservice info /control_robot_leju_claw`` 能看到真实服务提供节点；
2. ``rostopic info /leju_claw_state`` 能看到真实状态发布节点；
3. 状态消息中左右顺序确为 ``state[0]=left``、``state[1]=right``；
4. 实机方向确为 ``0=完全张开``、``100=完全闭合``。官方接口文档和当前
   SDK 实现采用该方向，但旧版 msg/srv 文件中的英文注释曾写反；
5. 先在夹爪周围无物、急停可触及的条件下，以很小的位置变化确认方向；
6. 再标定普通工件和 U 盘所需的目标位置、电流及是否以 ``Grabbed`` 为成功；
7. 确认夹爪动作是否要求机器人已经进入 stance，以及关闭本体时夹爪如何卸力；
8. 确认没有遥控器、SDK 或其他节点同时控制同一夹爪。

``/leju_claw_command`` 只在部分运动捕捉源码中作为发布话题出现，尚未确认硬件
驱动订阅，因此本适配器不使用它。``/control_robot_hand_position`` 属于强脑灵巧手
接口，也不用于 v63 乐聚夹爪。

安全策略：默认 ``execute=False``，只打印计划；真实执行时必须同时看到控制服务、
状态发布者和新鲜状态消息。单侧控制会从状态反馈读取并保持另一侧当前位置，绝不
凭空填入另一侧目标。服务响应只表示命令被接受，最终结果以状态话题为准。
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import rosgraph
import rospy
from kuavo_msgs.msg import lejuClawState
from kuavo_msgs.srv import controlLejuClaw, controlLejuClawRequest


CLAW_CONTROL_SERVICE = "/control_robot_leju_claw"
CLAW_STATE_TOPIC = "/leju_claw_state"
CLAW_NAMES = ("left_claw", "right_claw")


@dataclass(frozen=True)
class ClawStateSnapshot:
    """最近一帧左右夹爪状态。"""

    states: Tuple[int, int]
    positions: Tuple[float, float]
    velocities: Tuple[float, float]
    efforts: Tuple[float, float]
    generation: int
    received_monotonic: float


class ClawAdapter:
    """通过官方服务控制 v63 左右乐聚二指夹爪。"""

    ERROR = int(lejuClawState.kError)
    UNKNOWN = int(lejuClawState.kUnknown)
    MOVING = int(lejuClawState.kMoving)
    REACHED = int(lejuClawState.kReached)
    GRABBED = int(lejuClawState.kGrabbed)

    def __init__(
        self,
        execute: bool = False,
        control_service: str = CLAW_CONTROL_SERVICE,
        state_topic: str = CLAW_STATE_TOPIC,
        open_position: float = 0.0,
        close_position: float = 100.0,
        default_velocity: float = 50.0,
        default_effort: float = 1.0,
        service_timeout: float = 3.0,
        state_timeout: float = 3.0,
        state_max_age: float = 1.0,
        position_tolerance: float = 2.0,
    ):
        self.execute = bool(execute)
        self.control_service = control_service
        self.state_topic = state_topic
        self.open_position = self._bounded(open_position, "open_position", 0.0, 100.0)
        self.close_position = self._bounded(close_position, "close_position", 0.0, 100.0)
        self.default_velocity = self._bounded(default_velocity, "default_velocity", 0.0, 100.0)
        self.default_effort = self._bounded(default_effort, "default_effort", 0.0, 10.0)
        self.service_timeout = max(0.0, float(service_timeout))
        self.state_timeout = max(0.0, float(state_timeout))
        self.state_max_age = max(0.0, float(state_max_age))
        self.position_tolerance = max(0.0, float(position_tolerance))

        self._condition = threading.Condition()
        self._snapshot: Optional[ClawStateSnapshot] = None
        self._generation = 0
        self._subscriber = rospy.Subscriber(
            self.state_topic,
            lejuClawState,
            self._state_callback,
            queue_size=10,
        )

    def readiness(self) -> dict:
        """只读检查服务、状态发布者、类型和状态新鲜度。"""
        try:
            master = rosgraph.Master(rospy.get_name())
            publishers, _, services = master.getSystemState()
            topic_types = dict(master.getTopicTypes())
        except Exception as exc:
            return {
                "ready": False,
                "reasons": ["ros_master_unavailable"],
                "error": repr(exc),
                "execute": self.execute,
            }

        publisher_map = dict(publishers)
        service_map = dict(services)
        state_publishers = publisher_map.get(self.state_topic, [])
        service_nodes = service_map.get(self.control_service, [])
        state_type = topic_types.get(self.state_topic)
        snapshot = self.latest_state()
        state_age = None
        if snapshot is not None:
            state_age = max(0.0, time.monotonic() - snapshot.received_monotonic)

        reasons = []
        if not service_nodes:
            reasons.append("claw_control_service_unavailable")
        if not state_publishers:
            reasons.append("claw_state_has_no_publisher")
        if state_type != "kuavo_msgs/lejuClawState":
            reasons.append("claw_state_type_mismatch")
        if snapshot is None:
            reasons.append("claw_state_not_received")
        elif state_age is not None and state_age > self.state_max_age:
            reasons.append("claw_state_stale")

        return {
            "ready": not reasons,
            "reasons": reasons,
            "execute": self.execute,
            "control_service": self.control_service,
            "service_nodes": service_nodes,
            "state_topic": self.state_topic,
            "state_type": state_type,
            "state_publishers": state_publishers,
            "state_age": state_age,
            "latest_state": self._snapshot_as_dict(snapshot),
            "open_position": self.open_position,
            "close_position": self.close_position,
            "default_velocity": self.default_velocity,
            "default_effort": self.default_effort,
        }

    def latest_state(self) -> Optional[ClawStateSnapshot]:
        """返回最近状态快照；不会主动查询或控制机器人。"""
        with self._condition:
            return self._snapshot

    def wait_until_ready(self, timeout: Optional[float] = None) -> bool:
        """等待夹爪控制链路完整出现，只读不发送动作。"""
        wait_timeout = self.state_timeout if timeout is None else max(0.0, float(timeout))
        deadline = time.monotonic() + wait_timeout
        while not rospy.is_shutdown():
            status = self.readiness()
            if status.get("ready"):
                return True
            if time.monotonic() >= deadline:
                rospy.logerr("夹爪控制链路未就绪: %s", status)
                return False
            rospy.logwarn_throttle(1.0, "等待夹爪控制链路: %s", status.get("reasons"))
            rospy.sleep(0.1)
        return False

    def open(self, left: bool = True, right: bool = False, wait: bool = True) -> bool:
        """张开选中的夹爪；默认只操作旧程序使用的左夹爪。"""
        return self._command_selected(
            left=left,
            right=right,
            target=self.open_position,
            action="OPEN",
            wait=wait,
        )

    def close(
        self,
        left: bool = True,
        right: bool = False,
        position: Optional[float] = None,
        wait: bool = True,
    ) -> bool:
        """闭合选中的夹爪；可用 position 覆盖尚待标定的抓取位置。"""
        target = self.close_position if position is None else self._bounded(
            position,
            "position",
            0.0,
            100.0,
        )
        return self._command_selected(
            left=left,
            right=right,
            target=target,
            action="CLOSE",
            wait=wait,
        )

    def control(
        self,
        left_position: Optional[float] = None,
        right_position: Optional[float] = None,
        velocity: Optional[float] = None,
        effort: Optional[float] = None,
        wait: bool = True,
        action: str = "CONTROL",
    ) -> bool:
        """控制一侧或两侧；未指定的一侧保持最新反馈位置。"""
        if left_position is None and right_position is None:
            raise ValueError("至少需要指定 left_position 或 right_position")

        requested = [left_position, right_position]
        for index, value in enumerate(requested):
            if value is not None:
                requested[index] = self._bounded(value, "position", 0.0, 100.0)
        target_velocity = self.default_velocity if velocity is None else self._bounded(
            velocity,
            "velocity",
            0.0,
            100.0,
        )
        target_effort = self.default_effort if effort is None else self._bounded(
            effort,
            "effort",
            0.0,
            10.0,
        )

        rospy.loginfo(
            "夹爪计划: action=%s, left=%s, right=%s, velocity=%.1f, effort=%.2fA",
            action,
            "保持当前" if requested[0] is None else "{:.1f}".format(requested[0]),
            "保持当前" if requested[1] is None else "{:.1f}".format(requested[1]),
            target_velocity,
            target_effort,
        )
        if not self.execute:
            rospy.loginfo("[dry-run] 不调用 %s", self.control_service)
            return True

        if not self.wait_until_ready():
            return False
        snapshot = self.latest_state()
        if snapshot is None:
            rospy.logerr("没有夹爪状态，无法安全保持未控制侧")
            return False

        targets = [snapshot.positions[0], snapshot.positions[1]]
        commanded_indices: List[int] = []
        for index, value in enumerate(requested):
            if value is not None:
                targets[index] = float(value)
                commanded_indices.append(index)

        request = controlLejuClawRequest()
        request.data.name = list(CLAW_NAMES)
        request.data.position = targets
        request.data.velocity = [target_velocity, target_velocity]
        request.data.effort = [target_effort, target_effort]
        generation_before = snapshot.generation

        try:
            rospy.wait_for_service(self.control_service, timeout=self.service_timeout)
            response = rospy.ServiceProxy(
                self.control_service,
                controlLejuClaw,
            )(request)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr("夹爪服务调用失败: %s", exc)
            return False

        if not response.success:
            rospy.logerr("夹爪命令被拒绝: %s", response.message or "未提供原因")
            return False
        rospy.loginfo("夹爪服务已接受命令: %s", response.message or "success")

        if not wait:
            return True
        return self._wait_for_finish(commanded_indices, targets, generation_before)

    def shutdown(self) -> None:
        """注销状态订阅者，不发送任何夹爪命令。"""
        self._subscriber.unregister()

    def _command_selected(
        self,
        left: bool,
        right: bool,
        target: float,
        action: str,
        wait: bool,
    ) -> bool:
        if not left and not right:
            raise ValueError("left 和 right 不能同时为 False")
        return self.control(
            left_position=target if left else None,
            right_position=target if right else None,
            wait=wait,
            action=action,
        )

    def _state_callback(self, msg: lejuClawState) -> None:
        if len(msg.state) < 2:
            rospy.logwarn_throttle(1.0, "忽略长度不足的夹爪 state: %s", list(msg.state))
            return
        if len(msg.data.position) < 2:
            rospy.logwarn_throttle(
                1.0,
                "忽略缺少左右位置的夹爪状态: %s",
                list(msg.data.position),
            )
            return

        velocities = self._pair_or_default(msg.data.velocity, 0.0)
        efforts = self._pair_or_default(msg.data.effort, 0.0)
        with self._condition:
            self._generation += 1
            self._snapshot = ClawStateSnapshot(
                states=(int(msg.state[0]), int(msg.state[1])),
                positions=(float(msg.data.position[0]), float(msg.data.position[1])),
                velocities=velocities,
                efforts=efforts,
                generation=self._generation,
                received_monotonic=time.monotonic(),
            )
            self._condition.notify_all()

    def _wait_for_finish(
        self,
        commanded_indices: Sequence[int],
        targets: Sequence[float],
        generation_before: int,
    ) -> bool:
        deadline = time.monotonic() + self.state_timeout
        seen_moving = {index: False for index in commanded_indices}

        with self._condition:
            while not rospy.is_shutdown():
                snapshot = self._snapshot
                if snapshot is not None and snapshot.generation > generation_before:
                    failed = [
                        CLAW_NAMES[index]
                        for index in commanded_indices
                        if snapshot.states[index] == self.ERROR
                    ]
                    if failed:
                        rospy.logerr("夹爪报告错误: %s", failed)
                        return False

                    for index in commanded_indices:
                        if snapshot.states[index] == self.MOVING:
                            seen_moving[index] = True

                    finished = True
                    for index in commanded_indices:
                        state = snapshot.states[index]
                        position_matches = (
                            abs(snapshot.positions[index] - targets[index])
                            <= self.position_tolerance
                        )
                        if state == self.GRABBED:
                            continue
                        if state == self.REACHED and (seen_moving[index] or position_matches):
                            continue
                        finished = False
                        break

                    if finished:
                        rospy.loginfo(
                            "夹爪动作完成: states=%s, positions=%s",
                            snapshot.states,
                            tuple(round(value, 2) for value in snapshot.positions),
                        )
                        return True

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(timeout=min(0.1, remaining))

        rospy.logerr("等待夹爪完成超时，最后状态: %s", self._snapshot_as_dict(self.latest_state()))
        return False

    @staticmethod
    def _pair_or_default(values: Sequence[float], default: float) -> Tuple[float, float]:
        if len(values) >= 2:
            return float(values[0]), float(values[1])
        return float(default), float(default)

    @staticmethod
    def _bounded(value: float, name: str, minimum: float, maximum: float) -> float:
        result = float(value)
        if not math.isfinite(result) or result < minimum or result > maximum:
            raise ValueError("{} 必须在 [{}, {}] 范围内".format(name, minimum, maximum))
        return result

    @staticmethod
    def _snapshot_as_dict(snapshot: Optional[ClawStateSnapshot]) -> Optional[Dict[str, object]]:
        if snapshot is None:
            return None
        return {
            "states": snapshot.states,
            "positions": snapshot.positions,
            "velocities": snapshot.velocities,
            "efforts": snapshot.efforts,
            "generation": snapshot.generation,
        }
