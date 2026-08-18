#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo 5-W v63 乐聚二指夹爪诊断与测试程序。

默认是只读诊断型 dry-run，不调用夹爪服务：

    python3 test/test_claw_control.py --action check
    python3 test/test_claw_control.py --action open --side left
    python3 test/test_claw_control.py --action close --side left --position 10

dry-run 会检查 `/control_robot_leju_claw` 服务、`/leju_claw_state` 发布者、
消息类型和最新状态，并打印原本准备执行的动作。只有显式添加 `--execute` 才会
调用服务；真实测试前必须先阅读 `claw_adapter.py` 顶部的现场核对清单。

第一次实机测试不要直接使用 0 或 100 满行程。应读取当前 position，然后用
`--action position` 只改变少量数值，确认 0=张开、100=闭合的实际方向后，再测试
open/close。程序不会启动下位机 launch，也不会自动进入 stance。
"""

import argparse
import os
import sys

import rospy

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(CURRENT_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from claw_adapter import ClawAdapter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="v63 乐聚二指夹爪只读诊断与安全测试；默认 dry-run",
    )
    parser.add_argument(
        "--action",
        choices=("check", "open", "close", "position"),
        default="check",
        help="check 只检查；其余动作在没有 --execute 时也只预览",
    )
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default="left",
        help="控制左夹爪、右夹爪或两侧；旧程序默认使用左侧",
    )
    parser.add_argument(
        "--position",
        type=float,
        help="position 动作的目标值；close 动作提供时覆盖默认闭合位置",
    )
    parser.add_argument("--velocity", type=float, default=50.0, help="速度 0~100")
    parser.add_argument("--effort", type=float, default=1.0, help="电流限制，单位 A")
    parser.add_argument("--open-position", type=float, default=0.0)
    parser.add_argument("--close-position", type=float, default=100.0)
    parser.add_argument(
        "--check-timeout",
        type=float,
        default=3.0,
        help="等待服务和状态话题上线的只读检查时间",
    )
    parser.add_argument(
        "--motion-timeout",
        type=float,
        default=3.0,
        help="真实动作等待 Reached/Grabbed 的时间",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际调用夹爪服务；不提供时只诊断和打印计划",
    )
    return parser


def selected_sides(side: str):
    return side in ("left", "both"), side in ("right", "both")


def main() -> int:
    args = build_parser().parse_args()
    if args.action == "position" and args.position is None:
        raise SystemExit("--action position requires --position")

    rospy.init_node("test_claw_control", anonymous=True)
    claw = ClawAdapter(
        execute=args.execute,
        open_position=args.open_position,
        close_position=args.close_position,
        default_velocity=args.velocity,
        default_effort=args.effort,
        state_timeout=args.motion_timeout,
    )

    print("=== claw startup diagnostics (read-only) ===")
    ready = claw.wait_until_ready(timeout=args.check_timeout)
    diagnostics = claw.readiness()
    print(diagnostics)
    print("control_path_ready:", ready)
    print("action:", args.action)
    print("side:", args.side)
    print("execute:", args.execute)
    print()

    if args.action == "check":
        print("CHECK ONLY. No claw command was sent.")
        return 0 if ready else 1

    left, right = selected_sides(args.side)
    if args.action == "open":
        planned = claw.open(left=left, right=right)
    elif args.action == "close":
        planned = claw.close(
            left=left,
            right=right,
            position=args.position,
        )
    elif args.action == "position":
        planned = claw.control(
            left_position=args.position if left else None,
            right_position=args.position if right else None,
            velocity=args.velocity,
            effort=args.effort,
            action="POSITION",
        )
    else:
        raise ValueError(args.action)

    if not args.execute:
        print()
        print("DRY RUN ONLY. No claw command was sent.")
        print("Add --execute only after checking the current position and safe direction.")
        return 0 if ready and planned else 1

    print()
    print("result:", planned)
    return 0 if planned else 1


if __name__ == "__main__":
    raise SystemExit(main())
