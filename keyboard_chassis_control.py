#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通过上位机终端安全地离散遥控 Jaten 底盘。

本程序不依赖底盘物理键盘和 `/keyop`，直接复用已经验证的 HTTP 接口：

* `ChangeMode(MANUAL)`：开始前自动切换到手动模式。
* `RobotMotion(vx, vy, vw)`：发送短时间速度脉冲。
* `RobotMotion(0, 0, 0)`：每次动作结束、异常退出和正常退出时清零速度。

控制方式：

* 输入 W 后按 Enter：短距离前进。
* 输入 S 后按 Enter：短距离后退。
* 输入 A 后按 Enter：短时间左转。
* 输入 D 后按 Enter：短时间右转。
* 输入 X 或直接按 Enter：立即重复发送零速度。
* 输入 Q 后按 Enter：停止并退出。

安全设计：

1. 每次按键只产生一个固定时长的速度脉冲，默认 0.15 秒。
2. 脉冲结束后在 finally 中重复发送零速度，不依赖用户再按停止键。
3. 只接受单字符命令；长按形成 `wwww` 时整条命令无效并清零速度。
4. 线速度硬限制不超过 0.05m/s，角速度不超过 0.10rad/s，脉冲不超过 0.50s。
5. 默认 dry-run；只有显式提供 `--execute` 才连接和控制底盘。

运行位置：已经能访问 192.168.26.22 的机器人上位机。

示例：

python3 keyboard_chassis_control.py
python3 keyboard_chassis_control.py --execute
python3 keyboard_chassis_control.py --execute --linear-speed 0.03 --pulse-duration 0.15

注意：程序退出后底盘保持 MANUAL 模式，不会自动切回 AUTO。SSH 终端无法获得真正的
按键释放事件，因此使用“单字符 + Enter”作为一次动作的明确边界。
"""

import argparse
import signal
import time
from typing import Dict, Optional, Tuple

from chassis_adapter import ChassisHttpClient


MAX_LINEAR_SPEED = 0.05
MAX_ANGULAR_SPEED = 0.10
MAX_PULSE_DURATION = 0.50


def command_rejected(result: Dict[str, object]) -> bool:
    return bool(result.get("error") or result.get("success") is False)


def send_zero_velocity(client: ChassisHttpClient, repeat: int = 5) -> None:
    """尽最大努力重复清零；某一次失败不阻止后续零速度请求。"""
    last_error: Optional[Exception] = None
    for _ in range(max(1, repeat)):
        try:
            result = client.robot_motion(0.0, 0.0, 0.0)
            if command_rejected(result):
                last_error = RuntimeError("zero velocity was rejected: {}".format(result))
            else:
                last_error = None
        except Exception as exc:  # 清零阶段继续尝试剩余次数
            last_error = exc
        time.sleep(0.03)
    if last_error is not None:
        raise RuntimeError("最后一次零速度请求失败: {}".format(last_error))


def send_motion_pulse(
        client: ChassisHttpClient,
        vx: float,
        vw: float,
        duration: float,
        send_interval: float = 0.05) -> None:
    """发送一次有限时长运动脉冲，任何退出路径都会执行速度清零。"""
    end_time = time.monotonic() + duration
    try:
        while time.monotonic() < end_time:
            result = client.robot_motion(vx=vx, vy=0.0, vw=vw)
            if command_rejected(result):
                raise RuntimeError("RobotMotion 被拒绝: {}".format(result))
            time.sleep(send_interval)
    finally:
        send_zero_velocity(client)


def motion_for_key(
        key: str,
        linear_speed: float,
        angular_speed: float) -> Optional[Tuple[float, float, str]]:
    mapping = {
        "w": (linear_speed, 0.0, "前进"),
        "s": (-linear_speed, 0.0, "后退"),
        "a": (0.0, angular_speed, "左转"),
        "d": (0.0, -angular_speed, "右转"),
    }
    return mapping.get(key)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Jaten 底盘离散式安全键盘遥控")
    parser.add_argument("--execute", action="store_true", help="实际控制底盘")
    parser.add_argument("--host", default="192.168.26.22", help="底盘 HTTP 地址")
    parser.add_argument("--port", type=int, default=8888, help="底盘 HTTP 端口")
    parser.add_argument("--token", default=None, help="可选 Authorization 值")
    parser.add_argument(
        "--linear-speed", type=float, default=0.03,
        help="前后速度，默认 0.03m/s，最大 0.05m/s",
    )
    parser.add_argument(
        "--angular-speed", type=float, default=0.05,
        help="旋转速度，默认 0.05rad/s，最大 0.10rad/s",
    )
    parser.add_argument(
        "--pulse-duration", type=float, default=0.15,
        help="每次按键运动时间，默认 0.15s，最大 0.50s",
    )
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if not 0.0 < args.linear_speed <= MAX_LINEAR_SPEED:
        parser.error("--linear-speed 必须在 (0, {:.2f}] 范围内".format(MAX_LINEAR_SPEED))
    if not 0.0 < args.angular_speed <= MAX_ANGULAR_SPEED:
        parser.error("--angular-speed 必须在 (0, {:.2f}] 范围内".format(MAX_ANGULAR_SPEED))
    if not 0.0 < args.pulse_duration <= MAX_PULSE_DURATION:
        parser.error("--pulse-duration 必须在 (0, {:.2f}] 范围内".format(MAX_PULSE_DURATION))


def print_controls(args: argparse.Namespace) -> None:
    print("=" * 58)
    print("Jaten 底盘离散式键盘遥控")
    print("输入单个 W/S + Enter: 前进/后退")
    print("输入单个 A/D + Enter: 左转/右转")
    print("输入 X 或直接 Enter: 立即清零；Q + Enter: 清零并退出")
    print("每次动作: {:.2f}s，线速度: {:.3f}m/s，角速度: {:.3f}rad/s".format(
        args.pulse_duration,
        args.linear_speed,
        args.angular_speed,
    ))
    print("只接受单字符命令；例如 wwww 会被拒绝并保持零速度。")
    print("=" * 58)


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)
    print_controls(args)

    if not args.execute:
        print("\nDRY RUN：没有连接或控制机器人。添加 --execute 后进入按键控制。")
        return 0

    client = ChassisHttpClient(
        host=args.host,
        port=args.port,
        token=args.token,
        timeout=1.0,
    )
    def request_stop(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print("\n正在确认 MANUAL 模式...")
    try:
        mode_result = client.ensure_manual_mode()
    except Exception as exc:
        print("无法进入安全的手动控制状态:", exc)
        return 1
    print("MANUAL 模式已确认:", mode_result)
    print("请确认急停可用、底盘无故障、周围无人。现在可以按键。")

    try:
        send_zero_velocity(client)
        while True:
            command = input("\n控制命令 [w/s/a/d/x/q] > ").strip().lower()

            if command == "q":
                print("收到退出命令，正在清零速度...")
                break

            if command in ("", "x", "stop"):
                print("停止")
                send_zero_velocity(client)
                continue

            # 必须恰好是一个运动字符。长按形成 wwww 时不会执行任何动作。
            if len(command) != 1:
                print("拒绝多字符命令，保持零速度:", repr(command))
                send_zero_velocity(client)
                continue

            motion = motion_for_key(
                command,
                linear_speed=args.linear_speed,
                angular_speed=args.angular_speed,
            )
            if motion is None:
                print("未知命令，保持零速度:", repr(command))
                send_zero_velocity(client)
                continue

            vx, vw, description = motion
            print("{}：脉冲 {:.2f}s，随后自动停止".format(
                description,
                args.pulse_duration,
            ))
            send_motion_pulse(
                client,
                vx=vx,
                vw=vw,
                duration=args.pulse_duration,
            )
            print("动作结束，已发送零速度")
    except Exception as exc:
        print("\n键盘控制异常:", exc)
        return_code = 1
    else:
        return_code = 0
    finally:
        try:
            send_zero_velocity(client, repeat=8)
            print("\n已发送最终零速度，底盘保持 MANUAL 模式。")
        except Exception as exc:
            print("\n警告：最终零速度请求未全部成功:", exc)
            return_code = 1

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
