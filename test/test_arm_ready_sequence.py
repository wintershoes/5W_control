#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试 Kuavo 5-W 正式机械臂关节接口。

默认是诊断型 dry-run，不会控制机械臂。它会查询下位机本体启动状态，并检查
机械臂命令订阅者、传感器、MPC 服务和左右臂 Ruckig 反馈是否符合预期：

python3 test/test_arm_ready_sequence.py --sequence ready
python3 test/test_arm_ready_sequence.py --sequence retract
python3 test/test_arm_ready_sequence.py --sequence custom --joints "0,0,..."

正式控制链路为 `/kuavo_arm_traj` -> Ruckig -> MPC/WBC -> `/joint_cmd`。
真实发布时必须显式加 `--execute`。程序会在首次真实发布前自动查询本体状态，
必要时调用 `/humanoid_controller/real_initial_start` 进入 stance；不需要再用遥控器
按第二次 C。如果要发布从旧程序搬来的 READY/RETRACT
姿态，还必须额外加 `--allow-unverified-poses`，因为这些姿态尚未确认适配新 5-W
机械臂的零位、方向和限位。第一次现场测试建议先让技术人员给一个确认安全的
14 关节姿态，用 `--sequence custom --joints ... --execute` 跑，不要直接跑旧姿态。

dry-run 只会调用只读的 `/humanoid_controller/real_launch_status`，不会调用
`real_initial_start`。如果状态为 ready_stance/launched 而完整控制链路尚未出现，
表示本体程序已经启动，但仍需在正式执行时进入 stance。
"""

import argparse
import os
import sys

import rospy

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(CURRENT_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from arm_adapter import ArmAdapter, ArmPose, format_joint_csv, parse_joint_csv


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="机械臂关节控制测试；默认 dry-run",
    )
    parser.add_argument(
        "--sequence",
        choices=("ready", "ready-disk", "retract", "custom", "preview"),
        default="preview",
        help="要测试的机械臂动作序列",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="自动进入 stance 并实际发布 /kuavo_arm_traj；不提供时只打印",
    )
    parser.add_argument(
        "--allow-unverified-poses",
        action="store_true",
        help="允许真实发布从旧程序搬来的未验证姿态",
    )
    parser.add_argument("--arm-topic", default="/kuavo_arm_traj")
    parser.add_argument(
        "--left-reach-time-topic",
        default="/lb_arm_joint_reach_time/left",
    )
    parser.add_argument(
        "--right-reach-time-topic",
        default="/lb_arm_joint_reach_time/right",
    )
    parser.add_argument(
        "--joints",
        default="",
        help="custom 模式使用的 14 个逗号分隔关节角，单位为度",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="custom 模式缺失 Ruckig 时间反馈时的保守等待时间",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rospy.init_node("test_arm_ready_sequence", anonymous=True)

    arm = ArmAdapter(
        execute=args.execute,
        arm_topic=args.arm_topic,
        left_reach_time_topic=args.left_reach_time_topic,
        right_reach_time_topic=args.right_reach_time_topic,
        allow_unverified_poses=args.allow_unverified_poses,
    )

    print("=== arm startup diagnostics (read-only) ===")
    diagnostics = arm.startup_diagnostics(query_launch_status=True)
    print(diagnostics)
    print("body_program_ready:", diagnostics.get("body_program_ready"))
    print("control_path_ready:", diagnostics.get("control_path_ready"))
    print("sequence:", args.sequence)
    print("execute:", args.execute)
    print("stance_init:", "首次真实动作前自动执行" if args.execute else "dry-run 不执行")
    print("allow_unverified_poses:", args.allow_unverified_poses)
    print()

    if args.sequence == "preview":
        arm.preview_old_pick_sequence()
        print("READY_POSE_1:", format_joint_csv(ArmAdapter.READY_POSE_1.joints))
        print("READY_POSE_2:", format_joint_csv(ArmAdapter.READY_POSE_2.joints))
        print("READY_POSE_3:", format_joint_csv(ArmAdapter.READY_POSE_3.joints))
        print("RETRACT_POSE:", format_joint_csv(ArmAdapter.RETRACT_POSE.joints))
        return 0 if diagnostics.get("body_program_ready") else 1

    if not args.execute:
        print()
        print("DRY RUN ONLY. 未调用 stance 服务，也未发布机械臂目标。")
        print("Add --execute only after the diagnostics and workspace are confirmed safe.")
        arm.preview_old_pick_sequence(disk=args.sequence == "ready-disk")
        if args.sequence == "custom" and args.joints:
            joints = parse_joint_csv(args.joints)
            print("custom joints:", format_joint_csv(joints))
        return 0 if diagnostics.get("body_program_ready") else 1

    if args.sequence == "ready":
        ok = arm.move_to_ready_position(disk=False)
    elif args.sequence == "ready-disk":
        ok = arm.move_to_ready_position(disk=True)
    elif args.sequence == "retract":
        ok = arm.retract()
    elif args.sequence == "custom":
        if not args.joints:
            raise SystemExit("--sequence custom requires --joints")
        pose = ArmPose(
            name="CUSTOM_VERIFIED_POSE",
            joints=parse_joint_csv(args.joints),
            old_duration=args.duration,
            wait_after=0.0,
            verified=True,
        )
        ok = arm.publish_joint_pose(pose, wait=True)
    else:
        raise ValueError(args.sequence)

    print()
    print("result:", ok)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
