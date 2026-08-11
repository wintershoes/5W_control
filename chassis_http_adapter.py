#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTTP client for the chassis jaten-api command endpoint."""

import json
from typing import Any, Dict, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


class ChassisHttpClient:
    """调用底盘 jaten-api，不依赖 ROS 厂商自定义消息包。"""

    def __init__(self, host: str = "192.168.26.22", port: int = 8888,
                 token: Optional[str] = None, timeout: float = 3.0):
        self.base_url = "http://{}:{}".format(host, port).rstrip("/")
        self.token = token
        self.timeout = timeout

    def send_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        """通过 /command?cmd=... 发送一条 NavigationApi JSON 命令。"""
        headers = {"Content-Type": "application/json"}
        if self.token:
            headers["Authorization"] = self.token

        # 命令 JSON 是 /command 的 cmd 查询参数，不是直接作为请求体发送。
        command_url = self.base_url + "/command?" + urlencode({
            "cmd": json.dumps(payload, separators=(",", ":")),
        })
        request = Request(
            command_url,
            data=b"",
            headers=headers,
            method="POST",
        )
        try:
            with urlopen(request, timeout=self.timeout) as response:
                raw = response.read().decode("utf-8", errors="replace")
                if not raw:
                    return {"http_status": response.status}
                try:
                    result = json.loads(raw)
                except json.JSONDecodeError:
                    result = {"raw": raw}
                if isinstance(result, dict):
                    result.setdefault("http_status", response.status)
                return result
        except HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError("Chassis command HTTP {}: {}".format(exc.code, body)) from exc
        except URLError as exc:
            raise RuntimeError("Chassis command connection failed: {}".format(exc.reason)) from exc

    def robot_motion(self, vx: float, vy: float, vw: float) -> Dict[str, Any]:
        """发送一次 RobotMotion 速度请求。"""
        payload = {
            "id": "0",
            "method": "RobotMotion",
            "params": {
                "vx": float(vx),
                "vy": float(vy),
                "vw": float(vw),
            },
        }
        return self.send_command(payload)

    @staticmethod
    def make_dispatch_goal_node_name_payload(
            node_name: str, request_id: str = "1") -> Dict[str, Any]:
        """构造与网页右键站点导航相同的命令，不发送请求。"""
        node_name = node_name.strip()
        if not node_name:
            raise ValueError("node_name must not be empty")
        return {
            "method": "DispatchGoalNodeName",
            "id": str(request_id),
            "params": {
                "name": [node_name],
            },
        }

    def dispatch_goal_node_name(
            self, node_name: str, request_id: str = "1") -> Dict[str, Any]:
        """按当前路网中的站点名称下发导航目标。"""
        payload = self.make_dispatch_goal_node_name_payload(node_name, request_id)
        return self.send_command(payload)

    def stop(self, repeat: int = 3) -> None:
        """发送零速度，尽量确保底盘停止。"""
        last_error = None
        for _ in range(max(1, repeat)):
            try:
                self.robot_motion(0.0, 0.0, 0.0)
                last_error = None
            except RuntimeError as exc:
                last_error = exc
        if last_error is not None:
            raise last_error
