#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试底盘原生闭环旋转，不自行发送角速度。

调用链路：
1. 从 /move_base/amcl_pose 读取当前地图坐标系朝向。
2. 通过 HTTP NavigationApi 发送 SetRotationTheta 绝对目标朝向。
3. 底盘 /rotation_dispatch 根据 TF 闭环旋转。
4. 统一适配器等待当前 request_id 对应的 RotationStatus SUCCESS。

参数：
--delta 弧度              相对当前朝向转动，左正右负；脚本会换算为绝对 theta。
--delta-angle 度          相对当前朝向转动，左正右负，单位度。
--target-theta 弧度       直接指定地图坐标系绝对目标朝向。
--target-angle 度         直接指定地图坐标系绝对目标朝向，单位度。
--request-id 整数         可选；默认自动生成。
--timeout 秒              等待底盘闭环完成的超时，默认 30 秒。
--host/--port/--token     底盘 HTTP NavigationApi 参数。
--execute                 实际切换 AUTO 并发送命令；不提供时仅打印计划。

示例：
python3 test/test_chassis_closed_loop_rotate.py --delta 0.08
python3 test/test_chassis_closed_loop_rotate.py --execute --delta -0.08
python3 test/test_chassis_closed_loop_rotate.py --execute --target-theta 1.57
python3 test/test_chassis_closed_loop_rotate.py --execute --target-angle 90

安全说明：执行前请人工确认急停、驱动使能、底盘无故障和旋转空间安全。
"""

import argparse
import math
import os
import sys

import rospy

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from chassis_adapter import (  # noqa: E402
    ChassisAdapter,
    ChassisHttpClient,
    ChassisReadAdapter,
    normalize_angle,
)


def main():
    parser = argparse.ArgumentParser(description="Jaten 底盘原生闭环旋转测试")
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        "--delta", type=float, help="相对当前朝向的转角，左正右负，单位弧度"
    )
    target_group.add_argument(
        "--delta-angle", type=float,
        help="相对当前朝向的转角，左正右负，单位度",
    )
    target_group.add_argument(
        "--target-theta", type=float, help="地图坐标系绝对目标朝向，单位弧度"
    )
    target_group.add_argument(
        "--target-angle", type=float,
        help="地图坐标系绝对目标朝向，单位度",
    )
    parser.add_argument("--execute", action="store_true", help="实际执行旋转")
    parser.add_argument("--request-id", type=int, default=None, help="旋转请求编号")
    parser.add_argument("--timeout", type=float, default=30.0, help="完成超时（秒）")
    parser.add_argument("--host", default="192.168.26.22")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--token", default=None)
    args = parser.parse_args()

    if args.timeout <= 0.0:
        parser.error("--timeout 必须大于 0")
    if args.request_id is not None and not (
            0 <= args.request_id <= 0x7FFFFFFF):
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
    elif args.delta_angle is not None:
        target_theta = normalize_angle(
            current_theta + math.radians(args.delta_angle)
        )
    elif args.target_angle is not None:
        target_theta = normalize_angle(math.radians(args.target_angle))
    else:
        target_theta = normalize_angle(args.target_theta)
    expected_turn = normalize_angle(target_theta - current_theta)

    print("=== native closed-loop rotation test ===")
    print("before_pose:", before_pose)
    print("current_theta:", current_theta)
    print("target_theta:", target_theta)
    print("target_angle_deg:", math.degrees(target_theta))
    print("expected_turn:", expected_turn)
    print("request_id:", args.request_id if args.request_id is not None else "auto")
    print("execute:", args.execute)

    if not args.execute:
        print("DRY RUN ONLY. Add --execute to send SetRotationTheta.")
        return

    print("Warning: 请确认急停、驱动使能、底盘故障状态和旋转空间。")
    chassis = ChassisAdapter(
        host=args.host,
        port=args.port,
        token=args.token,
        reader=reader,
        http_client=client,
    )
    success = chassis.rotate_to_theta(
        target_theta=target_theta,
        timeout=args.timeout,
        request_id=args.request_id,
    )
    print("after_pose:", reader.get_current_pose())
    print("rotation_success:", success)
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
