#!/usr/bin/env python3
"""底盘原地旋转的低风险测试。默认只演练，不发送运动指令。"""

import argparse
import time

import rospy
from std_msgs.msg import String

from chassis_http_adapter import ChassisHttpClient
from chassis_read_adapter import ChassisReadAdapter


def read_nav_source():
    try:
        return rospy.wait_for_message(
            "/twist_mux_new/out/nav_source_used", String, timeout=1.0
        ).data
    except (rospy.ROSException, rospy.ROSInterruptException):
        return None


def main():
    parser = argparse.ArgumentParser(description="通过底盘 HTTP RobotMotion 测试原地旋转")
    parser.add_argument("--execute", action="store_true", help="实际执行旋转")
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument("--angle", type=float, default=0.05, help="旋转角度，单位弧度")
    parser.add_argument("--speed", type=float, default=0.03, help="角速度，单位弧度/秒")
    parser.add_argument("--host", default="192.168.26.22")
    parser.add_argument("--port", type=int, default=8888)
    args = parser.parse_args()
    rospy.init_node("test_chassis_rotate", anonymous=True)

    if not (0.0 < args.angle <= 0.2):
        parser.error("--angle 必须在 0 到 0.2 弧度之间")
    if not (0.0 < args.speed <= 0.1):
        parser.error("--speed 必须在 0 到 0.1 弧度/秒之间")

    reader = ChassisReadAdapter()
    client = ChassisHttpClient(host=args.host, port=args.port)
    before = reader.get_current_pose()
    nav_source = read_nav_source()
    duration = args.angle / args.speed
    vw = args.speed if args.direction == "left" else -args.speed

    print("=== chassis rotation test ===")
    print("before_pose:", before)
    print("nav_source:", nav_source)
    print("http_endpoint:", client.base_url + "/command?cmd=<RobotMotion JSON>")
    print("direction:", args.direction)
    print("angle_rad:", args.angle)
    print("angular_speed_rad_s:", args.speed)
    print("duration:", duration)
    print("execute:", args.execute)

    if not args.execute:
        print("DRY RUN ONLY. Add --execute to actually rotate.")
        return

    print("Warning: 请确认急停、驱动使能、自动模式和周围安全区域。")
    started = time.monotonic()
    try:
        while time.monotonic() - started < duration:
            result = client.robot_motion(vx=0.0, vy=0.0, vw=vw)
            print("motion result:", result)
            if isinstance(result, dict) and result.get("success") is False:
                print("RobotMotion rejected; stopping immediately.")
                break
            time.sleep(0.1)
    finally:
        print("Sending HTTP zero velocity stop...")
        client.stop()

    time.sleep(0.5)
    print("after_pose:", reader.get_current_pose())
    print("nav_source_after:", read_nav_source())


if __name__ == "__main__":
    main()
