# 5W_control

用于 Kuavo/Jaten 机器人底盘迁移与测试的代码。

## 运行位置

代码可以在 Windows 上编辑和提交；涉及 ROS 的脚本应在已经连接 ROS Master 的上位机环境中运行。

```bash
cd ~/5W_control
source /opt/ros/noetic/setup.bash
source ~/kuavo_ros_application/devel/setup.bash
```

## 当前内容

- `chassis_read_adapter.py`：只读读取底盘位姿、状态、错误和导航来源。
- `test_chassis_read_adapter.py`：只读状态检查。
- `chassis_http_adapter.py`：调用底盘 `POST /RobotMotion` HTTP 接口。
- `test_chassis_micro_move.py`：HTTP 小距离速度测试，默认是 dry-run，不会发送运动指令。

## 安全说明

运行移动测试前必须确认机器人周围安全、急停可用，并先执行 dry-run。只有明确使用 `--execute` 时才会发送 HTTP 移动请求。
