#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘小距离前进/后退测试。

调用方法：
1. 使用 ChassisReadAdapter 读取 /move_base/amcl_pose 和当前导航源，只做状态确认。
2. 使用 ChassisHttpClient.robot_motion()，通过底盘 jaten-api 的
   POST /command?cmd=<RobotMotion JSON> 持续发送本体系线速度。
3. 发送时长按 distance / speed 计算；结束或发生异常时发送零速度停止。

参数：
--direction forward|back  前进或后退，默认 forward。
--distance 米             预计移动距离，默认 0.02，测试脚本限制不超过 0.05。
--speed 米/秒             线速度绝对值，默认 0.03，测试脚本限制不超过 0.05。
--host                    底盘 API 地址，默认 192.168.26.22。
--port                    底盘 API 端口，默认 8888。
--token                   可选的 HTTP Authorization 值。
--execute                 实际发送运动指令；不提供时仅打印参数和状态。

示例：
python3 test_chassis_micro_move.py --direction forward --distance 0.02 --speed 0.03
python3 test_chassis_micro_move.py --execute --direction back --distance 0.02 --speed 0.03

注意：执行前人工确认自动模式、驱动使能、急停状态、底盘故障和周围空间。
"""

import argparse
import time

import rospy

from chassis_http_adapter import ChassisHttpClient
from chassis_read_adapter import ChassisReadAdapter


def main():
    parser = argparse.ArgumentParser(description="Tiny chassis HTTP motion test")
    parser.add_argument("--execute", action="store_true", help="实际发送 HTTP 运动指令")
    parser.add_argument("--direction", choices=["forward", "back"], default="forward")
    parser.add_argument("--distance", type=float, default=0.02, help="估计移动距离（米）")
    parser.add_argument("--speed", type=float, default=0.03, help="线速度（米/秒）")
    parser.add_argument("--host", default="192.168.26.22", help="底盘 API 主机地址")
    parser.add_argument("--port", type=int, default=8888, help="底盘 API 端口")
    parser.add_argument("--token", default=None, help="可选 Authorization 值")
    args = parser.parse_args()

    rospy.init_node("test_chassis_http_micro_move", anonymous=True)
    chassis = ChassisReadAdapter(wait_timeout=2.0)
    http_client = ChassisHttpClient(args.host, args.port, args.token)

    distance = abs(args.distance)
    speed = abs(args.speed)
    duration = distance / speed if speed > 0 else 0.0
    direction_sign = 1.0 if args.direction == "forward" else -1.0
    vx = direction_sign * speed

    interface_ready = chassis.check_motion_interface_ready()
    before_pose = chassis.get_current_pose()
    nav_source_before = chassis.get_nav_source_used()

    # 这里只读检查位姿；不会检查急停、驱动使能或底盘故障。
    print("=== motion interface readiness ===")
    print(interface_ready)
    print("before_pose:", before_pose)
    print("nav_source_before:", nav_source_before)
    print("http_endpoint:", "http://{}:{}/command?cmd=<RobotMotion JSON>".format(args.host, args.port))
    print("direction:", args.direction)
    print("distance:", distance)
    print("speed:", speed)
    print("duration:", duration)
    print("execute:", args.execute)

    if not args.execute:
        print("\nDRY RUN ONLY. Add --execute to send HTTP motion commands.")
        return

    if not interface_ready.get("ready", False):
        print("\nRefusing to move because pose interface is unavailable:", interface_ready.get("reasons"))
        return
    if distance > 0.05:
        print("\nRefusing to move more than 0.05m in this smoke test.")
        return
    if speed > 0.05:
        print("\nRefusing to use speed > 0.05m/s in this smoke test.")
        return

    print("\nWarning: jaten_msgs safety checks are disabled. Confirm emergency stop, driver enable, and chassis faults manually.")
    print("Sending HTTP RobotMotion velocity pulse...")
    try:
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and not rospy.is_shutdown():
            result = http_client.robot_motion(vx, 0.0, 0.0)
            print("motion result:", result)
            rospy.sleep(0.1)
    finally:
        print("Sending HTTP zero velocity stop...")
        try:
            http_client.stop()
        except RuntimeError as exc:
            print("stop request failed:", exc)

    rospy.sleep(1.0)
    after_pose = chassis.get_current_pose()
    nav_source_after = chassis.get_nav_source_used()
    print("\nafter_pose:", after_pose)
    print("nav_source_after:", nav_source_after)


if __name__ == "__main__":
    main()
