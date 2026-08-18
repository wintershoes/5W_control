#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo 5-W v63 躯干高度接口诊断与微小动作测试。

默认是只读 dry-run，不会进入 stance，也不会发布高度命令：

    python3 test/test_height_control.py
    python3 test/test_height_control.py --action offset --offset 0.005
    python3 test/test_height_control.py --action initial

dry-run 会检查 ROBOT_VERSION=63、实际 taskFile、本体启动状态、MPC observation、
传感器、``/cmd_lb_torso_pose`` 订阅者、初始位姿服务、MPC 模式查询服务和
``/lb_torso_pose_reach_time`` 发布者，并读取实际初始躯干位姿。

只有显式添加 ``--execute`` 才会按需进入 stance 并发布目标。第一次实机测试建议：

    python3 test/test_height_control.py --action offset --offset 0.005
    python3 test/test_height_control.py --action offset --offset 0.005 --execute
    python3 test/test_height_control.py --action initial --execute

第一条先检查，第二条仅上升 5 mm，第三条回到初始高度。测试时不要同时使用 H12
躯干控制，确保机器人周围安全且急停可触及。Ruckig 时间反馈只是预计运动时间，
当前程序不能把它解释成独立传感器确认的实际到位状态。
"""

import argparse
import os
import sys

import rospy

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(CURRENT_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from height_adapter import HeightAdapter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="v63 躯干高度只读诊断与微小动作测试；默认 dry-run",
    )
    parser.add_argument(
        "--action",
        choices=("check", "offset", "initial"),
        default="check",
        help="check 只检查；offset 移到初始高度加偏移；initial 回到初始高度",
    )
    parser.add_argument(
        "--offset",
        type=float,
        default=0.005,
        help="offset 动作相对初始躯干 Z 的正偏移，单位 m，默认 0.005",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="按需进入 stance 并实际发布高度目标；不提供时只读检查和预览",
    )
    parser.add_argument("--expected-robot-version", type=int, default=63)
    parser.add_argument("--min-offset", type=float, default=0.0)
    parser.add_argument("--max-offset", type=float, default=0.32)
    parser.add_argument(
        "--check-timeout",
        type=float,
        default=5.0,
        help="等待高度控制链路上线的时间",
    )
    parser.add_argument(
        "--feedback-timeout",
        type=float,
        default=2.0,
        help="真实发布后等待 Ruckig 规划时间反馈的时间",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rospy.init_node("test_height_control", anonymous=True)

    height = HeightAdapter(
        execute=args.execute,
        expected_robot_version=args.expected_robot_version,
        min_offset=args.min_offset,
        max_offset=args.max_offset,
        control_path_timeout=args.check_timeout,
        reach_feedback_timeout=args.feedback_timeout,
    )

    print("=== torso height startup diagnostics (read-only) ===")
    diagnostics = height.startup_diagnostics(query_services=True)
    print(diagnostics)
    print("body_program_ready:", diagnostics.get("body_program_ready"))
    print("control_path_ready:", diagnostics.get("control_path_ready"))
    print("initial_torso_pose:", diagnostics.get("initial_torso_pose"))
    print("mpc_mode:", diagnostics.get("mpc_mode"))
    print("action:", args.action)
    print("execute:", args.execute)
    print()

    if args.action == "check":
        print("CHECK ONLY. No stance service or torso command was sent.")
        return 0 if diagnostics.get("body_program_ready") and diagnostics.get("control_path_ready") else 1

    offset = 0.0 if args.action == "initial" else args.offset
    if not args.execute:
        try:
            plan = height.plan_offset(offset)
        except Exception as exc:
            print("cannot build height plan:", repr(exc))
            return 1
        print("planned_height:", plan)
        print()
        print("DRY RUN ONLY. No stance service or torso command was sent.")
        print("Add --execute only after diagnostics pass and the workspace is safe.")
        return 0 if diagnostics.get("body_program_ready") and diagnostics.get("control_path_ready") else 1

    print("planned_height: will be recalculated from the refreshed initial pose after stance")
    ok = height.move_to_offset(offset, wait=True)
    print()
    print("result:", ok)
    print("note: success means the command was planned and its expected duration elapsed;")
    print("      it is not an independent measured-height arrival confirmation.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
