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

- `chassis_adapter.py`：唯一的底盘接口文件，包含 ROS 位姿读取、HTTP 模式/运动/导航命令、PathTrackState 到站反馈、AMCL 闭环相对移动，以及 `SetRotationTheta + RotationStatus` 原生闭环旋转。
- `run_pick_place_chassis.py`：底盘抓放运行主程序。支持普通工件、U 盘或依次运行两套流程；抓取和放置动作当前留空。
- `keyboard_chassis_control.py`：浏览器键盘遥控。按下立即移动、松开立即停止，并由服务端看门狗处理失焦或网络中断。
- `test/`：模式切换、前后移动、旋转和站点导航测试程序。

## 安全键盘遥控

SSH 终端无法可靠获得按键松开事件，因此键盘遥控使用浏览器捕获 `keydown/keyup`。先查看 dry-run 参数，不连接底盘：

```bash
python3 keyboard_chassis_control.py
```

确认现场安全后，在上位机启动控制服务：

```bash
python3 keyboard_chassis_control.py \
  --execute \
  --browser-host 192.168.31.232
```

`--browser-host` 是 Windows 能访问的上位机局域网地址，只影响程序打印的浏览器 URL；
底盘控制仍通过默认的 `192.168.26.22` 进行。省略该参数时程序会自动选择一个不在底盘
内部 `192.168.26.x` 子网中的地址。程序会打印一个包含随机访问令牌的地址。在 Windows
浏览器打开该地址后：

- 按住 `W/S` 或上下方向键：前进/后退；松开立即停止。
- 按住 `A/D` 或左右方向键：左转/右转；松开立即停止。
- `Space` 或网页红色 `STOP`：立即停止。
- 浏览器窗口失焦、页面隐藏或关闭：立即请求停止。
- 控制心跳中断超过默认 `0.30s`：服务端看门狗自动清零速度。

服务启动时会自动切换并确认 MANUAL 模式，退出时重复发送零速度，退出后保持 MANUAL。

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

站点导航测试可在确认到站后指定地图坐标系绝对朝向：

```bash
python3 test/test_chassis_navigation.py \
  --execute --node NP1 --target-angle 90
```

`--target-angle` 使用度，`--target-theta` 使用弧度，二者不能同时提供。未指定
朝向时只执行原有站点导航。指定后，程序必须先确认到站，
再等待底盘原生闭环旋转返回 `RotationStatus SUCCESS`，之后才返回成功。

## 安全说明

默认 dry-run 不会控制机器人。实际执行前必须确认急停可用、驱动已使能、底盘无故障、当前地图和路网正确，并保证整个路径无人员和障碍。自动导航只有在 PathTrackState 报告已到目标节点、当前边和剩余边均为 0 时才返回成功；受阻后不会误判到站，但当前仍以任务超时作为最终失败条件。
