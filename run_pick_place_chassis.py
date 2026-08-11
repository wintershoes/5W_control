#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新机器人抓放运行主程序（当前只接入底盘）。

旧程序包含两套可选任务：

* workpiece：普通工件抓放。
* disk：U 盘抓放。

每套任务都使用同一条底盘流程：

抓取站点导航 -> 前进到抓取工作位 -> 抓取动作（留空） -> 后退到安全位
-> 放置站点导航 -> 前进到放置工作位 -> 放置动作（留空） -> 后退到安全位

跨区域导航只使用 `DispatchGoalNodeName` 路网站点，不再支持旧 task_id 或任意
x/y/theta 导航。程序默认是 dry-run，只打印流程；必须提供 `--execute` 才会调用
底盘。机械臂、夹爪、升降台、相机和视觉代码均未接入。

示例：

python3 run_pick_place_chassis.py --mode workpiece \
    --workpiece-pick-station PICK_A --workpiece-place-station PLACE_A

python3 run_pick_place_chassis.py --execute --mode workpiece \
    --workpiece-pick-station PICK_A --workpiece-place-station PLACE_A

python3 run_pick_place_chassis.py --execute --mode both \
    --workpiece-pick-station PICK_A --workpiece-place-station PLACE_A \
    --disk-pick-station PICK_USB --disk-place-station PLACE_USB

注意：当前导航完成判定基于 AMCL 位姿稳定，不能区分正常到站和受阻后停止。
自动导航中的任务取消接口也尚未确认。执行前必须人工确认急停、驱动使能、底盘
无故障、站点名称和路网正确，并保证机器人周围及整条路线安全。
"""

import argparse
from dataclasses import dataclass
from typing import List

import rospy

from chassis_adapter import ChassisAdapter

"""
！！！！！！！先不要跑，还没写好！！！！！！！
"""
@dataclass(frozen=True)
class TaskProfile:
    """一套抓放任务需要的底盘参数。"""

    name: str
    pick_station: str
    place_station: str
    pick_forward: float
    pick_back: float
    place_forward: float
    place_back: float


class PickPlaceChassisProgram:
    """编排两站点抓放流程；非底盘动作暂时只保留函数位置。"""

    def __init__(self, chassis: ChassisAdapter):
        self.chassis = chassis

    # ==================== 非底盘动作占位 ====================

    def perform_workpiece_pick(self) -> bool:
        """普通工件抓取动作占位：机械臂、视觉、升降和夹爪以后写在这里。"""
        rospy.logwarn("[TODO] 普通工件抓取动作尚未接入，本次直接跳过")
        return True

    def perform_workpiece_place(self) -> bool:
        """普通工件放置动作占位。"""
        rospy.logwarn("[TODO] 普通工件放置动作尚未接入，本次直接跳过")
        return True

    def perform_disk_pick(self) -> bool:
        """U 盘抓取动作占位：机械臂、视觉、升降和夹爪以后写在这里。"""
        rospy.logwarn("[TODO] U 盘抓取动作尚未接入，本次直接跳过")
        return True

    def perform_disk_place(self) -> bool:
        """U 盘放置动作占位。"""
        rospy.logwarn("[TODO] U 盘放置动作尚未接入，本次直接跳过")
        return True

    def _perform_pick(self, profile: TaskProfile) -> bool:
        if profile.name == "workpiece":
            return self.perform_workpiece_pick()
        if profile.name == "disk":
            return self.perform_disk_pick()
        raise ValueError("unknown task profile: {}".format(profile.name))

    def _perform_place(self, profile: TaskProfile) -> bool:
        if profile.name == "workpiece":
            return self.perform_workpiece_place()
        if profile.name == "disk":
            return self.perform_disk_place()
        raise ValueError("unknown task profile: {}".format(profile.name))

    # ==================== 底盘主流程 ====================

    def run_profile(self, profile: TaskProfile) -> bool:
        """执行一套抓放任务，任何关键底盘步骤失败都会停止后续流程。"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("开始执行 %s 底盘抓放流程", profile.name)
        rospy.loginfo("抓取站点: %s", profile.pick_station)
        rospy.loginfo("放置站点: %s", profile.place_station)
        rospy.loginfo("=" * 60)

        rospy.loginfo("[1/8] 导航到抓取站点 %s", profile.pick_station)
        if not self.chassis.navigate_to_station(profile.pick_station):
            rospy.logerr("导航到抓取站点失败")
            return False

        rospy.loginfo("[2/8] 前进到抓取工作位 %.3fm", profile.pick_forward)
        if not self.chassis.move_relative(forward=profile.pick_forward):
            rospy.logerr("前进到抓取工作位失败")
            return False

        rospy.loginfo("[3/8] 执行抓取动作")
        if not self._perform_pick(profile):
            rospy.logerr("抓取动作失败")
            return False

        rospy.loginfo("[4/8] 后退到抓取安全位 %.3fm", profile.pick_back)
        if not self.chassis.move_relative(forward=-profile.pick_back):
            rospy.logerr("离开抓取工作位失败")
            return False

        rospy.loginfo("[5/8] 导航到放置站点 %s", profile.place_station)
        if not self.chassis.navigate_to_station(profile.place_station):
            rospy.logerr("导航到放置站点失败")
            return False

        rospy.loginfo("[6/8] 前进到放置工作位 %.3fm", profile.place_forward)
        if not self.chassis.move_relative(forward=profile.place_forward):
            rospy.logerr("前进到放置工作位失败")
            return False

        rospy.loginfo("[7/8] 执行放置动作")
        if not self._perform_place(profile):
            rospy.logerr("放置动作失败")
            return False

        rospy.loginfo("[8/8] 后退到放置安全位 %.3fm", profile.place_back)
        if not self.chassis.move_relative(forward=-profile.place_back):
            rospy.logerr("离开放置工作位失败")
            return False

        rospy.loginfo("%s 底盘抓放流程完成", profile.name)
        return True

    def run(self, profiles: List[TaskProfile]) -> bool:
        """按传入顺序执行任务；`both` 模式为普通工件后执行 U 盘。"""
        for profile in profiles:
            if not self.run_profile(profile):
                return False
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="新机器人底盘抓放主流程；默认 dry-run，不控制机器人",
    )
    parser.add_argument(
        "--mode",
        choices=("workpiece", "disk", "both"),
        default="workpiece",
        help="普通工件、U 盘，或依次执行两套流程",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="实际执行底盘导航和移动；不提供时仅打印流程",
    )
    parser.add_argument("--host", default="192.168.26.22", help="底盘 HTTP 地址")
    parser.add_argument("--port", type=int, default=8888, help="底盘 HTTP 端口")
    parser.add_argument("--token", default=None, help="可选 Authorization 值")

    parser.add_argument("--workpiece-pick-station", default="")
    parser.add_argument("--workpiece-place-station", default="")
    parser.add_argument("--disk-pick-station", default="")
    parser.add_argument("--disk-place-station", default="")

    # 距离沿用旧主程序当前配置，后续可按新机器人现场标定结果调整。
    parser.add_argument("--workpiece-pick-forward", type=float, default=0.30)
    parser.add_argument("--workpiece-pick-back", type=float, default=0.35)
    parser.add_argument("--workpiece-place-forward", type=float, default=0.28)
    parser.add_argument("--workpiece-place-back", type=float, default=0.30)
    parser.add_argument("--disk-pick-forward", type=float, default=0.355)
    parser.add_argument("--disk-pick-back", type=float, default=0.40)
    parser.add_argument("--disk-place-forward", type=float, default=0.43)
    parser.add_argument("--disk-place-back", type=float, default=0.40)
    return parser


def build_profiles(args: argparse.Namespace) -> List[TaskProfile]:
    workpiece = TaskProfile(
        name="workpiece",
        pick_station=args.workpiece_pick_station.strip(),
        place_station=args.workpiece_place_station.strip(),
        pick_forward=args.workpiece_pick_forward,
        pick_back=args.workpiece_pick_back,
        place_forward=args.workpiece_place_forward,
        place_back=args.workpiece_place_back,
    )
    disk = TaskProfile(
        name="disk",
        pick_station=args.disk_pick_station.strip(),
        place_station=args.disk_place_station.strip(),
        pick_forward=args.disk_pick_forward,
        pick_back=args.disk_pick_back,
        place_forward=args.disk_place_forward,
        place_back=args.disk_place_back,
    )
    if args.mode == "workpiece":
        return [workpiece]
    if args.mode == "disk":
        return [disk]
    return [workpiece, disk]


def validate_profiles(
        parser: argparse.ArgumentParser,
        profiles: List[TaskProfile]) -> None:
    """执行和 dry-run 都要求明确站点名，避免把空值带入正式运行。"""
    for profile in profiles:
        if not profile.pick_station:
            parser.error("{} 模式缺少抓取站点名称".format(profile.name))
        if not profile.place_station:
            parser.error("{} 模式缺少放置站点名称".format(profile.name))
        for field_name in (
                "pick_forward", "pick_back", "place_forward", "place_back"):
            if getattr(profile, field_name) < 0.0:
                parser.error("{} 的 {} 不能为负数".format(profile.name, field_name))


def print_dry_run(profiles: List[TaskProfile], host: str, port: int) -> None:
    print("=== DRY RUN：不会连接或控制机器人 ===")
    print("chassis_http: http://{}:{}".format(host, port))
    for index, profile in enumerate(profiles, start=1):
        print("\n任务 {}: {}".format(index, profile.name))
        print("  1. AUTO 导航到抓取站点:", profile.pick_station)
        print("  2. MANUAL 前进 {:.3f}m".format(profile.pick_forward))
        print("  3. 抓取动作: TODO，当前跳过")
        print("  4. MANUAL 后退 {:.3f}m".format(profile.pick_back))
        print("  5. AUTO 导航到放置站点:", profile.place_station)
        print("  6. MANUAL 前进 {:.3f}m".format(profile.place_forward))
        print("  7. 放置动作: TODO，当前跳过")
        print("  8. MANUAL 后退 {:.3f}m".format(profile.place_back))
    print("\n添加 --execute 后才会实际执行以上底盘动作。")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    profiles = build_profiles(args)
    validate_profiles(parser, profiles)

    if not args.execute:
        print_dry_run(profiles, args.host, args.port)
        return 0

    rospy.init_node("kuavo_pick_place_chassis_main", anonymous=False)
    chassis = ChassisAdapter(
        host=args.host,
        port=args.port,
        token=args.token,
    )
    program = PickPlaceChassisProgram(chassis)

    rospy.logwarn("即将实际执行底盘动作；抓取和放置动作当前为空实现")
    try:
        success = program.run(profiles)
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.logwarn("运行被用户中断")
        success = False
    except Exception as exc:
        rospy.logerr("主流程异常: %s", exc)
        success = False

    if success:
        rospy.loginfo("全部已选底盘流程执行完成")
        return 0
    rospy.logerr("底盘主流程未完成")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

