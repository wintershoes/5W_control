#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘手动/自动运行模式切换测试，默认不发送命令。

调用方法：
使用 ChassisHttpClient.change_mode()，通过已验证的 jaten-api 通用入口
POST /command?cmd=<ChangeMode JSON> 下发模式切换：
{"method":"ChangeMode","id":"1","params":{"mode":"MANUAL"}}

模式用途：
MANUAL  手动模式，RobotMotion 前进、后退和旋转必须使用该模式。
AUTO    自动模式，DispatchGoalNodeName 路网站点导航必须使用该模式。

参数：
--mode MANUAL|AUTO        必填，目标运行模式；参数不区分大小写。
--request-id 请求编号     默认 1，只用于请求关联。
--host                    底盘 API 地址，默认 192.168.26.22。
--port                    底盘 API 端口，默认 8888。
--token                   可选的 HTTP Authorization 值。
--execute                 实际切换模式；不提供时仅打印请求。

示例：
python3 test_chassis_mode.py --mode manual
python3 test_chassis_mode.py --mode manual --execute
python3 test_chassis_mode.py --mode auto --execute

注意：模式切换会改变底盘控制权限。操作前确保机器人静止、没有正在执行的导航任务，
并确认急停位置。HTTP 成功响应只表示命令已处理，随后应从底盘网页或
/jaten_behavior_manager/robot_state 的 manual_mode 字段确认最终模式。
"""

import argparse
import json

from chassis_adapter import ChassisHttpClient


def parse_mode(value):
    mode = value.strip().upper()
    if mode not in ("MANUAL", "AUTO"):
        raise argparse.ArgumentTypeError("模式只能是 MANUAL 或 AUTO")
    return mode


def main():
    parser = argparse.ArgumentParser(description="Jaten chassis ChangeMode test")
    parser.add_argument("--mode", required=True, type=parse_mode,
                        help="目标模式：MANUAL 或 AUTO")
    parser.add_argument("--execute", action="store_true", help="实际发送模式切换命令")
    parser.add_argument("--request-id", default="1", help="NavigationApi 请求 ID")
    parser.add_argument("--host", default="192.168.26.22", help="底盘 API 主机地址")
    parser.add_argument("--port", type=int, default=8888, help="底盘 API 端口")
    parser.add_argument("--token", default=None, help="可选 Authorization 值")
    args = parser.parse_args()

    client = ChassisHttpClient(args.host, args.port, args.token)
    payload = client.make_change_mode_payload(args.mode, args.request_id)

    print("=== chassis mode switch test ===")
    print("http_endpoint:", "http://{}:{}/command?cmd=<ChangeMode JSON>".format(
        args.host, args.port
    ))
    print("target_mode:", args.mode)
    print("payload:", json.dumps(payload, ensure_ascii=False, separators=(",", ":")))
    print("execute:", args.execute)

    if not args.execute:
        print("\nDRY RUN ONLY. Add --execute to change the chassis mode.")
        return

    print("\nWarning: confirm that the chassis is stationary and no navigation task is running.")
    result = client.change_mode(args.mode, args.request_id)
    print("change_mode_result:", result)

    if isinstance(result, dict) and result.get("error"):
        print("\nMode switch was rejected. The chassis mode was not confirmed.")
        raise SystemExit(1)

    expected_manual_mode = args.mode == "MANUAL"
    print("\nCommand accepted. Verify the final state on the chassis:")
    print("  rostopic echo -n 1 /jaten_behavior_manager/robot_state")
    print("  expected manual_mode:", expected_manual_mode)


if __name__ == "__main__":
    main()
