# 5W_control

用于 Kuavo/Jaten 机器人底盘迁移与测试的代码。

## 运行位置

代码可以在 Windows 上编辑和提交；涉及 ROS 的脚本应在已连接 ROS Master 的上位机环境中运行。

```bash
cd ~/5W_control
source /opt/ros/noetic/setup.bash
source ~/kuavo_ros_application/devel/setup.bash
```

## 文件说明

- `chassis_adapter.py`：唯一的底盘接口文件，包含 ROS 位姿读取、HTTP 模式/运动/导航命令、站点导航等待、AMCL 闭环相对移动和旋转。
- `run_pick_place_chassis.py`：底盘抓放运行主程序。支持普通工件、U 盘或依次运行两套流程；抓取和放置动作当前留空。
- `keyboard_chassis_control.py`：上位机终端离散式键盘遥控。每条单字符命令只执行一次短脉冲，随后自动清零速度。
- `test/`：模式切换、前后移动、旋转和站点导航测试程序。

## 安全键盘遥控

先查看 dry-run 参数，不连接底盘：

```bash
python3 keyboard_chassis_control.py
```

确认现场安全后启动实际控制：

```bash
python3 keyboard_chassis_control.py --execute
```

为避免 SSH 终端无法识别按键松开带来的风险，程序采用“单字符 + Enter”控制：

- `w` + Enter：短距离前进。
- `s` + Enter：短距离后退。
- `a` + Enter：短时间左转。
- `d` + Enter：短时间右转。
- `x` + Enter 或直接 Enter：重复发送零速度。
- `q` + Enter：发送零速度并退出。

每次运动默认只有 `0.15s`，随后自动重复发送零速度。类似 `wwww` 的多字符输入不会执行运动。程序启动时会自动切换并确认 MANUAL 模式，退出后保持 MANUAL。

## 主程序用法

先执行 dry-run，只检查参数和动作顺序：

```bash
python3 run_pick_place_chassis.py \
  --mode workpiece \
  --workpiece-pick-station PICK_A \
  --workpiece-place-station PLACE_A
```

确认站点、距离、路网和现场安全后，显式添加 `--execute` 才会控制底盘：

```bash
python3 run_pick_place_chassis.py \
  --execute \
  --mode workpiece \
  --workpiece-pick-station PICK_A \
  --workpiece-place-station PLACE_A
```

`--mode` 支持：

- `workpiece`：只执行普通工件流程。
- `disk`：只执行 U 盘流程。
- `both`：先执行普通工件流程，再执行 U 盘流程；需要提供四个站点名称。

程序只使用路网站点导航，不接受旧 task ID 或任意坐标导航。每套流程顺序为：抓取站点 -> 前进 -> 抓取占位 -> 后退 -> 放置站点 -> 前进 -> 放置占位 -> 后退。

## 安全说明

默认 dry-run 不会控制机器人。实际执行前必须确认急停可用、驱动已使能、底盘无故障、当前地图和路网正确，并保证整个路径无人员和障碍。当前自动导航完成判定基于 AMCL 位姿稳定，尚不能可靠区分正常到站与受阻停止。
