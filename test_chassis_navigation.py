#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过当前路网中的站点名称测试底盘导航。

调用方法：
1. 使用 ChassisReadAdapter 读取 /move_base/amcl_pose 和当前导航源。
2. 使用 ChassisHttpClient.dispatch_goal_node_name()，通过底盘 jaten-api 的
   POST /command?cmd=<DispatchGoalNodeName JSON> 下发站点名称。
3. 请求格式与底盘网页右键站点导航一致：
   {"method":"DispatchGoalNodeName","id":"1","params":{"name":["NP1"]}}

参数：
--node 站点名             必填，例如 NP1；必须存在于当前加载的路网且区分大小写。
--request-id 请求编号     默认 1，只用于请求关联，不是站点 ID。
--host                    底盘 API 地址，默认 192.168.26.22。
--port                    底盘 API 端口，默认 8888。
--token                   可选的 HTTP Authorization 值。
--execute                 实际下发导航；不提供时仅构造并打印请求。

示例：
python3 test_chassis_navigation.py --node NP1
python3 test_chassis_navigation.py --node NP1 --execute

注意：HTTP 返回只表示命令已被处理，不代表机器人已经到站。执行前确认底盘处于
自动模式、驱动已使能、急停已释放、当前路网正确且路径无障碍。
"""

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
