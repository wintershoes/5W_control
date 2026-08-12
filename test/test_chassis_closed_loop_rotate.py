#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试底盘原生闭环旋转，不自行发送角速度。

调用链路：
1. 从 /move_base/amcl_pose 读取当前地图坐标系朝向。
2. 通过 HTTP NavigationApi 发送 SetRotationTheta；其参数 theta 是目标朝向，
   单位为弧度。
3. 底盘 /rotation_dispatch 节点读取 TF、执行闭环旋转，并在
   /rotation_dispatch/rotation_status 发布 WAIT/RUNNING/SUCCESS。
4. 本程序只在收到当前 request_id 对应的 SUCCESS 后判定成功。

参数：
--delta 弧度              相对当前朝向转动的小角度，左正右负。
--target-theta 弧度       直接指定地图坐标系绝对目标朝向。
--max-turn 弧度           单次允许的最大转角，默认 0.2。
--request-id 整数         可选；默认根据当前时间生成，范围为 Java 正 int。
--timeout 秒              等待底盘闭环完成的超时，默认 30 秒。
--host/--port/--token     底盘 HTTP NavigationApi 参数。
--execute                 实际切换 AUTO 并发送命令；不提供时仅打印计划。

示例：
python3 test/test_chassis_closed_loop_rotate.py --delta 0.05
python3 test/test_chassis_closed_loop_rotate.py --execute --delta 0.05
python3 test/test_chassis_closed_loop_rotate.py --execute --delta -0.05

安全说明：
- --delta 与 --target-theta 必须且只能提供一个。
- 程序会用 AMCL 计算预计转角，超过 --max-turn 时拒绝执行。
- SetRotationTheta 使用底盘原生闭环，不接受速度参数，也不需要本机安装
  jaten_msgs；RotationStatus 通过 rospy.AnyMsg 按已确认字段布局解析。
- 执行前仍需人工确认急停、驱动使能、底盘无故障且旋转范围内无障碍物。
"""

import argparse
import os
import struct
import sys
import threading
import time

import rospy

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from chassis_adapter import (  # noqa: E402
    ChassisHttpClient,
    ChassisReadAdapter,
    normalize_angle,
)


ROTATION_STATUS_TOPIC = "/rotation_dispatch/rotation_status"
STATUS_WAIT = 0
STATUS_SUCCESS = 1
STATUS_RUNNING = 2
STATUS_NAMES = {
    STATUS_WAIT: "WAIT",
    STATUS_SUCCESS: "SUCCESS",
    STATUS_RUNNING: "RUNNING",
}


def decode_rotation_status(data):
    """解析 jaten_msgs/RotationStatus 的 ROS1 序列化数据。"""
    if len(data) < 5:
        raise ValueError("RotationStatus 数据不足 5 字节")
    request_id, rotation_status = struct.unpack_from("<IB", data, 0)
    return {
        "current_request_id": request_id,
        "rotation_status": rotation_status,
        "status_name": STATUS_NAMES.get(rotation_status, "UNKNOWN"),
    }


class RotationStatusMonitor:
    """线程安全地保存底盘原生旋转状态。"""

    def __init__(self, topic=ROTATION_STATUS_TOPIC):
        self.topic = topic
        self._condition = threading.Condition()
        self._generation = 0
        self._state = None
        self._subscriber = rospy.Subscriber(
            topic, rospy.AnyMsg, self._callback, queue_size=20
        )

    def _callback(self, msg):
        try:
            state = decode_rotation_status(msg._buff)
        except (ValueError, struct.error) as exc:
            rospy.logerr_throttle(5.0, "解析 %s 失败: %s", self.topic, exc)
            return
        with self._condition:
            self._state = state
            self._generation += 1
            self._condition.notify_all()

    def wait_for_update(self, generation, timeout):
        deadline = time.monotonic() + max(0.0, timeout)
        with self._condition:
            while self._generation <= generation and not rospy.is_shutdown():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
            state = dict(self._state) if self._state is not None else None
            return self._generation, state


def make_rotation_payload(request_id, target_theta):
    """构造底盘 JAR 中 SetRotationThetaCmd 使用的原生命令。"""
    return {
        "method": "SetRotationTheta",
        "id": str(request_id),
        "params": {"theta": float(target_theta)},
    }


def command_rejected(result):
    return bool(result.get("error") or result.get("success") is False)


def wait_for_rotation(monitor, request_id, timeout):
    """等待指定旋转请求成功；WAIT 不代表失败，超时才返回失败。"""
    deadline = time.monotonic() + timeout
    generation = 0
    last_printed = None
    request_observed = False

    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            print("rotation_timeout: request_id={}".format(request_id))
            return False

        generation, state = monitor.wait_for_update(
            generation, min(0.5, remaining)
        )
        if state is None:
            continue

        summary = (state["current_request_id"], state["rotation_status"])
        if summary != last_printed:
            print("rotation_status:", state)
            last_printed = summary

        if state["current_request_id"] != request_id:
            continue

        request_observed = True
        if state["rotation_status"] == STATUS_SUCCESS:
            return True

    if not request_observed:
        print("未观察到 request_id={} 的旋转状态".format(request_id))
    return False


def main():
    parser = argparse.ArgumentParser(description="Jaten 底盘原生闭环旋转测试")
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        "--delta", type=float, help="相对当前朝向的转角，左正右负，单位弧度"
    )
    target_group.add_argument(
        "--target-theta", type=float, help="地图坐标系绝对目标朝向，单位弧度"
    )
    parser.add_argument("--execute", action="store_true", help="实际执行旋转")
    parser.add_argument("--max-turn", type=float, default=0.2, help="最大允许转角")
    parser.add_argument("--request-id", type=int, default=None, help="uint32 请求编号")
    parser.add_argument("--timeout", type=float, default=30.0, help="完成超时（秒）")
    parser.add_argument("--host", default="192.168.26.22")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--token", default=None)
    args = parser.parse_args()

    if not (0.0 < args.max_turn <= 0.5):
        parser.error("--max-turn 必须在 0 到 0.5 弧度之间")
    if args.timeout <= 0.0:
        parser.error("--timeout 必须大于 0")

    request_id = args.request_id
    if request_id is None:
        # ROS 字段是 uint32，但底盘 JAR 的命令构造函数接收 Java int。
        request_id = int(time.time() * 1000) & 0x7FFFFFFF
    if not (0 <= request_id <= 0x7FFFFFFF):
        parser.error("--request-id 必须在 0 到 2147483647 之间")

    rospy.init_node("test_chassis_closed_loop_rotate", anonymous=True)
    reader = ChassisReadAdapter(wait_timeout=2.0)
    client = ChassisHttpClient(args.host, args.port, args.token)
    before_pose = reader.get_current_pose()
    if before_pose is None:
        print("无法读取 AMCL 当前朝向，拒绝构造旋转任务。")
        raise SystemExit(1)

    current_theta = before_pose["theta"]
    if args.delta is not None:
        target_theta = normalize_angle(current_theta + args.delta)
    else:
        target_theta = normalize_angle(args.target_theta)
    expected_turn = normalize_angle(target_theta - current_theta)

    if abs(expected_turn) > args.max_turn:
        print("拒绝执行：预计转角 {:.6f} rad 超过 --max-turn {:.6f} rad".format(
            expected_turn, args.max_turn
        ))
        raise SystemExit(2)

    payload = make_rotation_payload(request_id, target_theta)
    print("=== native closed-loop rotation test ===")
    print("before_pose:", before_pose)
    print("current_theta:", current_theta)
    print("target_theta:", target_theta)
    print("expected_turn:", expected_turn)
    print("request_id:", request_id)
    print("payload:", payload)
    print("execute:", args.execute)

    if not args.execute:
        print("DRY RUN ONLY. Add --execute to send SetRotationTheta.")
        return

    print("Warning: 请确认急停、驱动使能、底盘故障状态和旋转空间。")
    monitor = RotationStatusMonitor()
    _, initial_state = monitor.wait_for_update(0, 2.0)
    print("rotation_status_before:", initial_state)

    mode_result = client.change_mode("AUTO", request_id=str(request_id))
    print("change_mode_auto_result:", mode_result)
    if command_rejected(mode_result):
        print("AUTO 模式切换被拒绝，停止测试。")
        raise SystemExit(1)
    rospy.sleep(0.5)

    result = client.send_command(payload)
    print("set_rotation_theta_result:", result)
    if command_rejected(result):
        print("SetRotationTheta 被底盘拒绝。")
        raise SystemExit(1)

    success = wait_for_rotation(monitor, request_id, args.timeout)
    after_pose = reader.get_current_pose()
    print("after_pose:", after_pose)
    if after_pose is not None:
        final_error = abs(normalize_angle(target_theta - after_pose["theta"]))
        print("final_angle_error_rad:", final_error)
    print("rotation_success:", success)
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
