#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过当前路网中的站点名称测试底盘导航。

调用方法：
1. 使用 ChassisReadAdapter 读取 /move_base/amcl_pose 和当前导航源。
2. 使用 ChassisAdapter.navigate_to_station() 通过底盘 jaten-api 下发站点名称。
3. 订阅 /path_sequence_executor/out/path_track_state，持续打印路网进度，并在
   reached_node_id == end_node_id 且剩余边为 0 时确认导航成功。
4. 请求格式与底盘网页右键站点导航一致：
   {"method":"DispatchGoalNodeName","id":"1","params":{"name":["NP1"]}}
5. 提供 --target-theta 时，到站后使用 SetRotationTheta 执行底盘原生闭环
   旋转；收到对应 RotationStatus SUCCESS 后才判定整个任务成功。

参数：
--node 站点名             必填，例如 NP1；必须存在于当前加载的路网且区分大小写。
--request-id 请求编号     默认 1，只用于请求关联，不是站点 ID。
--host                    底盘 API 地址，默认 192.168.26.22。
--port                    底盘 API 端口，默认 8888。
--token                   可选的 HTTP Authorization 值。
--timeout                 任务启动后的导航超时，默认 120 秒。
--start-timeout           等待路网状态进入执行中的超时，默认 15 秒。
--target-theta            可选，地图坐标系绝对目标朝向，单位弧度。
--target-angle            可选，地图坐标系绝对目标朝向，单位度。
--rotation-timeout        等待闭环旋转完成的超时，默认 30 秒。
--rotation-request-id     可选的旋转请求编号；默认自动生成。
--execute                 实际下发导航；不提供时仅构造并打印请求。

示例：
python3 test_chassis_navigation.py --node NP1
python3 test_chassis_navigation.py --node NP1 --execute
python3 test_chassis_navigation.py --node NP1 --target-theta 1.57 --execute
python3 test_chassis_navigation.py --node NP1 --target-angle 90 --execute

注意：HTTP 返回只表示命令已被处理；程序以 PathTrackState 确认最终到站。
"""

import argparse
import json

import rospy
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)
from chassis_adapter import ChassisAdapter, ChassisHttpClient, ChassisReadAdapter


def main():
    parser = argparse.ArgumentParser(description="Jaten chassis station navigation test")
    parser.add_argument("--node", required=True, help="路网站点名称，例如 NP1；区分大小写")
    parser.add_argument("--execute", action="store_true", help="实际下发站点导航命令")
    parser.add_argument("--request-id", default="1", help="NavigationApi 请求 ID")
    parser.add_argument("--host", default="192.168.26.22", help="底盘 API 主机地址")
    parser.add_argument("--port", type=int, default=8888, help="底盘 API 端口")
    parser.add_argument("--token", default=None, help="可选 Authorization 值")
    parser.add_argument("--timeout", type=float, default=120.0, help="导航执行超时（秒）")
    parser.add_argument("--start-timeout", type=float, default=15.0, help="等待任务启动超时（秒）")
    orientation_group = parser.add_mutually_exclusive_group()
    orientation_group.add_argument(
        "--target-theta", type=float, default=None,
        help="到站后的地图坐标系绝对目标朝向（弧度）",
    )
    orientation_group.add_argument(
        "--target-angle", type=float, default=None,
        help="到站后的地图坐标系绝对目标朝向（度）",
    )
    parser.add_argument(
        "--rotation-timeout", type=float, default=30.0,
        help="等待底盘原生闭环旋转完成的超时（秒）",
    )
    parser.add_argument(
        "--rotation-request-id", type=int, default=None,
        help="可选的闭环旋转请求编号；默认自动生成",
    )
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
    print("target_theta:", args.target_theta)
    print("target_angle_deg:", args.target_angle)
    print("payload:", json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    print("execute:", args.execute)

    if not args.execute:
        print("\nDRY RUN ONLY. Add --execute to dispatch this station goal.")
        return

    if not readiness.get("ready", False):
        print("\nRefusing to navigate because pose interface is unavailable:", readiness.get("reasons"))
        return

    print("\nWarning: confirm driver enable, emergency stop and a clear route.")
    chassis = ChassisAdapter(
        host=args.host,
        port=args.port,
        token=args.token,
        reader=reader,
        http_client=client,
    )
    success = chassis.navigate_to_station(
        args.node,
        timeout=args.timeout,
        initial_check_time=args.start_timeout,
        max_retries=1,
        request_id=args.request_id,
        target_theta=args.target_theta,
        target_angle=args.target_angle,
        rotation_timeout=args.rotation_timeout,
        rotation_request_id=args.rotation_request_id,
    )
    print("navigation_success:", success)
    print("nav_source_after:", reader.get_nav_source_used())
    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
