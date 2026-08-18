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
- `arm_adapter.py`：机械臂正式关节接口。`execute=True` 时自动从 `ready_stance/launched` 推进到 `stance`，再向 `/kuavo_arm_traj` 发布 14 关节角度（单位为度），并等待左右臂 Ruckig 预计时间。旧姿态尚未现场验证，真实执行需要额外确认。
- `height_adapter.py`：v63 躯干高度接口。读取实机初始躯干位姿，以 `初始 Z + 偏移` 生成 `/cmd_lb_torso_pose` 目标，并等待 Ruckig 预计时间。默认限制偏移为 `0~0.32m`。
- `vision_adapter.py`：右腕相机与 AprilTag 只读接口。检查图像、内参和检测节点，按标签 ID 获取新鲜数据，并支持连续采样、跳点过滤和位姿平均。
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
- 网页 `SPEED` 滑杆可在 `10%–100%` 间动态调速；每次打开页面都从 `30%` 慢速启动。
- `Space` 或网页红色 `STOP`：立即停止。
- 浏览器窗口失焦、页面隐藏或关闭：立即请求停止。
- 控制心跳中断超过默认 `0.30s`：服务端看门狗自动清零速度。

服务启动时会自动切换并确认 MANUAL 模式，退出时重复发送零速度，退出后保持 MANUAL。
默认 `100%` 对应线速度 `0.10m/s`、角速度 `0.20rad/s`。启动时可用
`--linear-speed` 和 `--angular-speed` 调整网页的硬上限；服务端允许的绝对上限分别为
`0.20m/s` 和 `0.40rad/s`。首次提高上限时应从低档开始现场测试。

## 机械臂测试

机械臂测试前必须在下位机启动本体控制程序：

```bash
cd ~/kuavo-ros-opensource
sudo su
source devel/setup.bash
roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=h12
```

也可以由正常工作的 H12 状态机通过第一次 `E左 + F右 + C` 启动本体程序。
测试程序不能跨机器代替该 launch；但提供 `--execute` 后，会自动查询
`/humanoid_controller/real_launch_status` 并调用 `/humanoid_controller/real_initial_start`
进入 `stance`，不再需要用遥控器按第二次 C。初始化服务成功且完整控制链路就绪后，
程序才会发布机械臂姿态。

先只看旧程序迁移出来的姿态和接口状态，不会控制机械臂：

```bash
python3 test/test_arm_ready_sequence.py --sequence preview
```

如果技术人员给了一个确认安全、接近当前姿态的 14 关节角度，可以先用 custom
模式测试 `/kuavo_arm_traj` 链路：

```bash
python3 test/test_arm_ready_sequence.py \
  --execute \
  --sequence custom \
  --joints "<左臂7个角度,右臂7个角度>"
```

`--joints` 使用度，必须恰好提供 14 个有限数值。不要把示例占位文字原样运行，
也不要默认把 14 个零当作安全姿态。程序会等待
`/lb_arm_joint_reach_time/left` 和 `/lb_arm_joint_reach_time/right` 给出的 Ruckig
预计时间；缺少任一反馈时会保守等待并中止后续动作。

旧程序 READY/RETRACT 姿态还没确认适配新 5-W。即使加了 `--execute`，
程序也会拒绝发布这些旧姿态；现场确认安全后才额外添加
`--allow-unverified-poses`。

## 高度测试

高度控制同样要求下位机已经启动 `load_kuavo_real_wheel.launch`。先运行只读检查：

```bash
python3 test/test_height_control.py
python3 test/test_height_control.py --action offset --offset 0.005
```

dry-run 会检查 `ROBOT_VERSION=63`、实际 `taskFile`、本体状态、MPC observation、
传感器、命令订阅者、初始躯干位姿服务和 Ruckig 时间反馈。全部确认后，第一次只
上升 5mm，再回到初始高度：

```bash
python3 test/test_height_control.py --action offset --offset 0.005 --execute
python3 test/test_height_control.py --action initial --execute
```

`--offset` 是相对启动初始躯干 Z 的偏移，不是旧升降台绝对高度。程序默认只接受
`0~0.32m`，不会允许低于初始高度。`/lb_torso_pose_reach_time` 仅表示 Ruckig
规划的预计时间，目前不能作为独立传感器确认的实测到位信号。

## 右腕视觉测试

先在上位机的两个终端分别启动腕部相机和右腕 AprilTag 检测器：

```bash
cd ~/kuavo_ros_application
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch dynamic_biped load_robot_head.launch \
  use_orbbec:=true enable_wrist_camera:=true all_enable:=false
```

```bash
cd ~/kuavo_ros_application
source /opt/ros/noetic/setup.bash
source devel/setup.bash
ROS_NAMESPACE=/apriltag_cam_r \
roslaunch apriltag_ros continuous_detection.launch \
  camera_name:=/right_wrist_camera/color image_topic:=image_raw
```

运行测试的终端也必须加载 `kuavo_ros_application` 环境：

```bash
cd ~/5W_control
source /opt/ros/noetic/setup.bash
source ~/kuavo_ros_application/devel/setup.bash
python3 test/test_vision_apriltag.py --action check
python3 test/test_vision_apriltag.py --action once --tag-id 0
python3 test/test_vision_apriltag.py --action average --tag-id 0 --samples 10
```

上述程序完全只读，不控制机器人。当前标签0配置为 `tag36h11`、有效边长
`0.042m`。适配器返回标准右腕光学坐标 `x向右、y向下、z向前`，没有沿用旧机器
的相机旋转和反向安装修正。手机显示可用于检测测试，但不能代替实物尺寸标定。

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
