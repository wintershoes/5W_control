#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过 jaten-api 按路网站点名称测试导航，默认只做 dry-run。"""

import argparse
import json

import rospy

from chassis_http_adapter import ChassisHttpClient
from chassis_read_adapter import ChassisReadAdapter


def main():
    parser = argparse.ArgumentParser(description="Jaten chassis station navigation test")
    parser.add_argument("--node", required=True, help="路网站点名称，例如 NP1；区分大小写")
    parser.add_argument("--execute", action="store_true", help="实际下发站点导航命令")
    parser.add_argument("--request-id", default="1", help="NavigationApi 请求 ID")
    parser.add_argument("--host", default="192.168.26.22", help="底盘 API 主机地址")
    parser.add_argument("--port", type=int, default=8888, help="底盘 API 端口")
    parser.add_argument("--token", default=None, help="可选 Authorization 值")
    args = parser.parse_args()

    rospy.init_node("test_chassis_station_navigation", anonymous=True)
    reader = ChassisReadAdapter(wait_timeout=2.0)
    client = ChassisHttpClient(args.host, args.port, args.token)
    payload = client.make_dispatch_goal_node_name_payload(args.node, args.request_id)

    readiness = reader.check_motion_interface_ready()
    before_pose = reader.get_current_pose()
    nav_source_before = reader.get_nav_source_used()

    print("=== station navigation test ===")
    print("readiness:", readiness)
    print("before_pose:", before_pose)
    print("nav_source_before:", nav_source_before)
    print("http_endpoint:", "http://{}:{}/command?cmd=<DispatchGoalNodeName JSON>".format(
        args.host, args.port
    ))
    print("target_node:", args.node)
    print("payload:", json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    print("execute:", args.execute)

    if not args.execute:
        print("\nDRY RUN ONLY. Add --execute to dispatch this station goal.")
        return

    if not readiness.get("ready", False):
        print("\nRefusing to navigate because pose interface is unavailable:", readiness.get("reasons"))
        return

    print("\nWarning: confirm automatic mode, driver enable, emergency stop and a clear route.")
    result = client.dispatch_goal_node_name(args.node, args.request_id)
    print("dispatch_result:", result)
    print("\nThe HTTP response only confirms command handling, not arrival at the station.")
    rospy.sleep(1.0)
    print("nav_source_after_dispatch:", reader.get_nav_source_used())


if __name__ == "__main__":
    main()
