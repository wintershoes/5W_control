#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tiny chassis movement smoke test.

Default mode is dry-run. Add --execute to publish motion commands.

This script is intentionally conservative:
- reads chassis readiness before moving
- supports very small default distance/speed
- publishes zero velocity before and after the pulse
- prints before/after pose from /move_base/amcl_pose
"""

import argparse
import math
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from chassis_read_adapter import ChassisReadAdapter

try:
    from leju_mobile_base_msgs.srv import BaseMove
except ImportError:
    BaseMove = None


NAV_SOURCE_TOPIC = "/twist_mux_new/in/switch/nav_source"
MPC_MOVE_BASE_TOPIC = "/twist_mux_new/in/nav/mpc_move_base"
TCP_API_TOPIC = "/twist_mux_new/in/op/tcp_api"
BASE_MOVE_SERVICE = "/move_base/base_move"


def make_twist(linear_x=0.0, linear_y=0.0, angular_z=0.0):
    msg = Twist()
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z
    return msg


def pose_distance(a, b):
    if not a or not b:
        return None
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"])


def publish_zero(pub, repeat=5, rate_hz=20):
    rate = rospy.Rate(rate_hz)
    zero = make_twist()
    for _ in range(repeat):
        pub.publish(zero)
        rate.sleep()


def pulse_twist(pub, twist, duration, rate_hz=20):
    rate = rospy.Rate(rate_hz)
    end_time = time.time() + duration
    while time.time() < end_time and not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()


def call_base_move_service(args):
    if BaseMove is None:
        raise RuntimeError("leju_mobile_base_msgs/BaseMove is not available in this environment")

    rospy.wait_for_service(BASE_MOVE_SERVICE, timeout=3.0)
    proxy = rospy.ServiceProxy(BASE_MOVE_SERVICE, BaseMove)

    direction_sign = 1.0 if args.direction == "forward" else -1.0
    x = direction_sign * abs(args.distance)

    return proxy(
        x=x,
        y=0.0,
        theta=0.0,
        avoid_enable=args.avoid_enable,
        avoid_distance=args.avoid_distance,
        linear_velocity=abs(args.speed),
        angular_velocity=0.15,
        position_threshold=args.position_threshold,
        angle_threshold=0.05,
        allow_rotation=False,
    )


def main():
    parser = argparse.ArgumentParser(description="Tiny chassis movement test")
    parser.add_argument("--execute", action="store_true", help="actually publish motion commands")
    parser.add_argument("--direction", choices=["forward", "back"], default="forward")
    parser.add_argument("--distance", type=float, default=0.02, help="target pulse distance in meters")
    parser.add_argument("--speed", type=float, default=0.03, help="linear speed in m/s")
    parser.add_argument("--method", choices=["base_move", "mpc_move_base", "tcp_api"], default="base_move")
    parser.add_argument(
        "--switch-nav-source",
        action="store_true",
        help="publish nav_mpc_move_base to /twist_mux_new/in/switch/nav_source before moving",
    )
    parser.add_argument("--avoid-enable", action="store_true", help="enable base_move obstacle avoidance")
    parser.add_argument("--avoid-distance", type=float, default=0.3)
    parser.add_argument("--position-threshold", type=float, default=0.01)
    args = parser.parse_args()

    rospy.init_node("test_chassis_micro_move", anonymous=True)
    chassis = ChassisReadAdapter(wait_timeout=2.0)

    distance = abs(args.distance)
    speed = abs(args.speed)
    duration = distance / speed if speed > 0 else 0.0
    direction_sign = 1.0 if args.direction == "forward" else -1.0
    linear_x = direction_sign * speed

    topic = None
    pub = None
    if args.method == "mpc_move_base":
        topic = MPC_MOVE_BASE_TOPIC
        pub = rospy.Publisher(topic, Twist, queue_size=1)
    elif args.method == "tcp_api":
        topic = TCP_API_TOPIC
        pub = rospy.Publisher(topic, Twist, queue_size=1)
    nav_pub = rospy.Publisher(NAV_SOURCE_TOPIC, String, queue_size=1)

    rospy.sleep(0.5)

    ready = chassis.check_chassis_ready()
    before_pose = chassis.get_current_pose()
    nav_source_before = chassis.get_nav_source_used()

    print("=== chassis readiness ===")
    print(ready)
    print("before_pose:", before_pose)
    print("nav_source_before:", nav_source_before)
    print("method:", args.method)
    print("topic:", topic)
    print("service:", BASE_MOVE_SERVICE if args.method == "base_move" else None)
    print("direction:", args.direction)
    print("distance:", distance)
    print("speed:", speed)
    print("duration:", duration)
    print("switch_nav_source:", args.switch_nav_source)
    print("avoid_enable:", args.avoid_enable)
    print("avoid_distance:", args.avoid_distance)
    print("position_threshold:", args.position_threshold)
    print("execute:", args.execute)

    if not args.execute:
        print("\nDRY RUN ONLY. Add --execute to actually move.")
        return

    if not ready.get("ready", False):
        print("\nRefusing to move because chassis is not ready:", ready.get("reasons"))
        return

    if distance > 0.05:
        print("\nRefusing to move more than 0.05m in this smoke test.")
        return

    if speed > 0.05:
        print("\nRefusing to use speed > 0.05m/s in this smoke test.")
        return

    if args.method == "base_move":
        print("\nCalling /move_base/base_move...")
        try:
            result = call_base_move_service(args)
        except Exception as exc:
            print("base_move call failed:", exc)
            return
        print("base_move result:", result)
    else:
        print("\nPublishing zero velocity first...")
        publish_zero(pub)

        if args.switch_nav_source:
            print("Switching nav source to nav_mpc_move_base...")
            for _ in range(5):
                nav_pub.publish(String(data="nav_mpc_move_base"))
                rospy.sleep(0.05)

        print("Publishing tiny movement pulse...")
        twist = make_twist(linear_x=linear_x)
        pulse_twist(pub, twist, duration)

        print("Publishing stop...")
        publish_zero(pub, repeat=20)

    rospy.sleep(1.0)
    after_pose = chassis.get_current_pose()
    nav_source_after = chassis.get_nav_source_used()
    moved = pose_distance(before_pose, after_pose)

    print("\nafter_pose:", after_pose)
    print("nav_source_after:", nav_source_after)
    print("estimated_xy_distance:", moved)


if __name__ == "__main__":
    main()
