#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kuavo 5-W 右腕相机与 AprilTag 只读测试。

本测试不会控制机器人，也不需要 ``--execute``。运行前先按 ``vision_adapter.py``
顶部说明在上位机启动腕部相机和右腕 AprilTag 检测器，并在当前终端加载：

    source /opt/ros/noetic/setup.bash
    source ~/kuavo_ros_application/devel/setup.bash

只检查接口和相机内参：

    python3 test/test_vision_apriltag.py --action check

将 ID 0 的 tag36h11 标签放入右腕画面后，读取一帧：

    python3 test/test_vision_apriltag.py --action once --tag-id 0

连续读取10帧，剔除位置跳点并输出平均值和标准差：

    python3 test/test_vision_apriltag.py --action average --tag-id 0 --samples 10

手机显示的标签可以验证识别链路，但只有实物有效边长与配置的 0.042m 相等时，
输出的三维距离才可用于抓取标定。
"""

import argparse
import os
import sys

import rospy

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(CURRENT_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from vision_adapter import VisionAdapter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="右腕相机和 AprilTag 只读诊断；不会控制机器人",
    )
    parser.add_argument(
        "--action",
        choices=("check", "once", "average"),
        default="check",
        help="check 检查接口；once 读取一帧；average 连续采样并过滤",
    )
    parser.add_argument("--tag-id", type=int, default=0, help="目标AprilTag ID")
    parser.add_argument("--samples", type=int, default=10, help="average采样帧数")
    parser.add_argument("--timeout", type=float, default=5.0, help="等待标签总时间")
    parser.add_argument(
        "--max-data-age",
        type=float,
        default=0.5,
        help="允许使用的最新检测帧年龄，单位秒",
    )
    parser.add_argument(
        "--expected-frame",
        default="right_wrist_camera_color_optical_frame",
        help="期望的检测坐标系",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.samples <= 0:
        raise SystemExit("--samples must be positive")
    if args.timeout <= 0.0:
        raise SystemExit("--timeout must be positive")

    rospy.init_node("test_vision_apriltag", anonymous=True)
    vision = VisionAdapter(
        expected_frame=args.expected_frame,
        max_data_age=args.max_data_age,
    )

    print("=== right wrist vision diagnostics (read-only) ===")
    ready = vision.wait_until_ready(timeout=min(args.timeout, 3.0))
    diagnostics = vision.readiness()
    print(diagnostics)
    print("interface_ready:", ready)

    try:
        camera_info = vision.read_camera_info(timeout=min(args.timeout, 3.0))
    except Exception as exc:
        print("camera_info_error:", repr(exc))
        camera_info = None
    print("camera_info:", camera_info)
    print("action:", args.action)
    print("tag_id:", args.tag_id)
    print()

    if args.action == "check":
        print("CHECK ONLY. No robot command was sent.")
        print("An empty detections array is normal when no configured tag is visible.")
        return 0 if ready and camera_info else 1

    if not ready or camera_info is None:
        print("Vision interface is incomplete; refusing to treat tag data as valid.")
        return 1

    if args.action == "once":
        pose = vision.wait_for_tag(args.tag_id, timeout=args.timeout)
        if pose is None:
            print("No fresh detection for tag {} within {}s.".format(args.tag_id, args.timeout))
            return 1
        print("tag_pose:", pose.as_dict())
        print("READ ONLY. No robot command was sent.")
        return 0

    try:
        estimate = vision.estimate_tag_pose(
            tag_id=args.tag_id,
            count=args.samples,
            timeout=args.timeout,
            min_inliers=min(3, args.samples),
        )
    except Exception as exc:
        print("tag_estimate_error:", repr(exc))
        return 1
    print("tag_pose_estimate:", estimate.as_dict())
    print("stddev values are in meters; multiply by 1000 for millimeters.")
    print("READ ONLY. No robot command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
