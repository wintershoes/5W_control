#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""浏览器键盘遥控 Jaten 底盘。

SSH 终端只能收到字符，无法可靠收到按键松开事件。本程序在上位机启动一个轻量网页，
由浏览器捕获 keydown/keyup：按住 W/S/A/D 或方向键时移动，松开时立即发送零速度。

安全机制：

1. 浏览器按键松开、窗口失焦、页面隐藏或关闭时立即请求停止。
2. 浏览器按住按键时每 80ms 发送心跳；服务端超过 watchdog 时间未收到心跳，
   自动重复发送零速度。默认 watchdog 为 0.30s。
3. 每条控制请求带递增序号，服务端忽略乱序到达的旧请求。
4. 服务端限制线速度、角速度，浏览器只能发送 -1/0/1 方向，不能修改速度。
5. 服务启动时确认 MANUAL 模式，异常退出和正常退出时都重复发送零速度。
6. 网页使用随机访问令牌，未携带令牌的局域网请求不能控制底盘。

运行位置：能访问底盘 192.168.26.22 的机器人上位机。

默认 dry-run：

    python3 keyboard_chassis_control.py

实际启动：

    python3 keyboard_chassis_control.py --execute

然后在 Windows 浏览器打开程序打印的 URL，例如：

    http://192.168.31.232:8765/?token=...

控制键：W/S 前后，A/D 左右旋转，方向键同样有效，Space 立即停止。
程序退出后底盘保持 MANUAL 模式，不会自动切回 AUTO。
"""

import argparse
import json
import secrets
import signal
import socket
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional, Tuple
from urllib.parse import parse_qs, urlparse

from chassis_adapter import ChassisHttpClient


CONTROL_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Jaten Chassis Control</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Arial, "Microsoft YaHei", sans-serif;
      background: #f3f5f7;
      color: #18212b;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 24px;
    }
    main {
      width: min(560px, 100%);
      display: grid;
      gap: 20px;
    }
    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      border-bottom: 1px solid #c9d1d9;
      padding-bottom: 14px;
    }
    h1 { margin: 0; font-size: 22px; letter-spacing: 0; }
    #state {
      min-width: 92px;
      text-align: center;
      padding: 7px 10px;
      border-radius: 4px;
      background: #d9e2ea;
      font-size: 13px;
      font-weight: 700;
    }
    #state.ready { background: #bfe8cf; color: #14532d; }
    #state.moving { background: #ffe29a; color: #713f12; }
    #state.error { background: #ffd0d0; color: #7f1d1d; }
    .pad {
      display: grid;
      grid-template-columns: repeat(3, 88px);
      grid-template-rows: repeat(2, 88px);
      justify-content: center;
      gap: 10px;
    }
    .key, #stop {
      border: 1px solid #8c98a4;
      border-radius: 6px;
      background: #ffffff;
      color: #18212b;
      font: inherit;
      font-size: 24px;
      font-weight: 700;
      cursor: pointer;
      touch-action: none;
      user-select: none;
    }
    .key[data-code="KeyW"] { grid-column: 2; grid-row: 1; }
    .key[data-code="KeyA"] { grid-column: 1; grid-row: 2; }
    .key[data-code="KeyS"] { grid-column: 2; grid-row: 2; }
    .key[data-code="KeyD"] { grid-column: 3; grid-row: 2; }
    .key.active {
      background: #1f6f50;
      border-color: #145c42;
      color: #ffffff;
    }
    #stop {
      width: 100%;
      min-height: 68px;
      background: #b42318;
      border-color: #8f1c14;
      color: #ffffff;
      font-size: 18px;
    }
    #stop:active { background: #7f1d1d; }
    .telemetry {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 1px;
      background: #c9d1d9;
      border: 1px solid #c9d1d9;
    }
    .telemetry div {
      background: #ffffff;
      padding: 12px;
      min-width: 0;
    }
    .label { color: #66717d; font-size: 12px; }
    .value { margin-top: 4px; font-family: Consolas, monospace; font-size: 15px; }
    #message { min-height: 20px; color: #56616d; font-size: 13px; }
    @media (max-width: 420px) {
      body { padding: 16px; }
      .pad {
        grid-template-columns: repeat(3, 72px);
        grid-template-rows: repeat(2, 72px);
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>Jaten Chassis</h1>
      <div id="state">CONNECTING</div>
    </header>

    <div class="pad" aria-label="Direction control">
      <button class="key" data-code="KeyW" type="button">W</button>
      <button class="key" data-code="KeyA" type="button">A</button>
      <button class="key" data-code="KeyS" type="button">S</button>
      <button class="key" data-code="KeyD" type="button">D</button>
    </div>

    <button id="stop" type="button">STOP</button>

    <div class="telemetry">
      <div><div class="label">LINEAR</div><div class="value" id="linear">0</div></div>
      <div><div class="label">ANGULAR</div><div class="value" id="angular">0</div></div>
    </div>
    <div id="message"></div>
  </main>

  <script>
    const ACCESS_TOKEN = __ACCESS_TOKEN__;
    const active = new Set();
    const keyAlias = {
      ArrowUp: "KeyW", ArrowDown: "KeyS",
      ArrowLeft: "KeyA", ArrowRight: "KeyD"
    };
    // 以当前时间作为序号基线。刷新页面后的新请求天然大于旧页面遗留请求。
    let sequence = Date.now() * 1000;
    let heartbeat = null;

    const stateEl = document.getElementById("state");
    const messageEl = document.getElementById("message");
    const linearEl = document.getElementById("linear");
    const angularEl = document.getElementById("angular");

    function setState(text, style) {
      stateEl.textContent = text;
      stateEl.className = style || "";
    }

    function direction() {
      const linear = (active.has("KeyW") ? 1 : 0) - (active.has("KeyS") ? 1 : 0);
      const angular = (active.has("KeyA") ? 1 : 0) - (active.has("KeyD") ? 1 : 0);
      return {linear, angular};
    }

    function renderKeys() {
      document.querySelectorAll(".key").forEach(button => {
        button.classList.toggle("active", active.has(button.dataset.code));
      });
      const motion = direction();
      linearEl.textContent = String(motion.linear);
      angularEl.textContent = String(motion.angular);
      setState(motion.linear || motion.angular ? "MOVING" : "READY",
               motion.linear || motion.angular ? "moving" : "ready");
    }

    async function postMotion(forceStop = false) {
      const motion = forceStop ? {linear: 0, angular: 0} : direction();
      const requestSequence = ++sequence;
      try {
        const response = await fetch("/motion", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
            "X-Control-Token": ACCESS_TOKEN
          },
          body: JSON.stringify({...motion, sequence: requestSequence}),
          cache: "no-store",
          keepalive: forceStop
        });
        if (!response.ok) throw new Error(await response.text());
        messageEl.textContent = "";
      } catch (error) {
        active.clear();
        renderKeys();
        setState("ERROR", "error");
        messageEl.textContent = String(error);
      }
    }

    function ensureHeartbeat() {
      const moving = active.size > 0;
      if (moving && heartbeat === null) {
        heartbeat = window.setInterval(() => postMotion(false), 80);
      } else if (!moving && heartbeat !== null) {
        window.clearInterval(heartbeat);
        heartbeat = null;
      }
    }

    function press(code) {
      if (!["KeyW", "KeyA", "KeyS", "KeyD"].includes(code)) return;
      if (!active.has(code)) {
        active.add(code);
        renderKeys();
        postMotion(false);
      }
      ensureHeartbeat();
    }

    function release(code) {
      if (!active.delete(code)) return;
      renderKeys();
      postMotion(false);
      ensureHeartbeat();
    }

    function emergencyStop() {
      active.clear();
      renderKeys();
      ensureHeartbeat();
      postMotion(true);
    }

    window.addEventListener("keydown", event => {
      const code = keyAlias[event.code] || event.code;
      if (code === "Space") {
        event.preventDefault();
        emergencyStop();
        return;
      }
      if (["KeyW", "KeyA", "KeyS", "KeyD"].includes(code)) {
        event.preventDefault();
        press(code);
      }
    });

    window.addEventListener("keyup", event => {
      const code = keyAlias[event.code] || event.code;
      if (["KeyW", "KeyA", "KeyS", "KeyD"].includes(code)) {
        event.preventDefault();
        release(code);
      }
    });

    document.querySelectorAll(".key").forEach(button => {
      button.addEventListener("pointerdown", event => {
        event.preventDefault();
        button.setPointerCapture(event.pointerId);
        press(button.dataset.code);
      });
      const stopPointer = event => {
        event.preventDefault();
        release(button.dataset.code);
      };
      button.addEventListener("pointerup", stopPointer);
      button.addEventListener("pointercancel", stopPointer);
      button.addEventListener("lostpointercapture", () => release(button.dataset.code));
    });

    document.getElementById("stop").addEventListener("click", emergencyStop);
    window.addEventListener("blur", emergencyStop);
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) emergencyStop();
    });
    window.addEventListener("pagehide", () => {
      active.clear();
      const stopSequence = ++sequence;
      navigator.sendBeacon(`/stop?token=${encodeURIComponent(ACCESS_TOKEN)}&sequence=${stopSequence}`, "");
    });

    renderKeys();
    postMotion(true);
  </script>
</body>
</html>
"""


class SafeMotionController:
    """串行发送底盘速度，并在心跳中断后自动清零。"""

    def __init__(
            self,
            client: ChassisHttpClient,
            linear_speed: float,
            angular_speed: float,
            watchdog_timeout: float,
            send_interval: float = 0.05):
        self.client = client
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.watchdog_timeout = watchdog_timeout
        self.send_interval = send_interval

        self._state_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._worker: Optional[threading.Thread] = None

        self._linear_direction = 0
        self._angular_direction = 0
        self._last_heartbeat = 0.0
        self._last_sequence = -1
        self._last_error: Optional[str] = None
        self._watchdog_stops = 0

    @staticmethod
    def _rejected(result: Dict[str, Any]) -> bool:
        return bool(result.get("error") or result.get("success") is False)

    def start(self) -> Dict[str, Any]:
        """确认 MANUAL、清零速度并启动看门狗线程。"""
        mode_result = self.client.ensure_manual_mode()
        self._send_zero(repeat=8)
        self._worker = threading.Thread(
            target=self._run,
            name="chassis-motion-watchdog",
            daemon=True,
        )
        self._worker.start()
        return mode_result

    def update(self, linear: int, angular: int, sequence: int) -> bool:
        """更新浏览器方向；返回 False 表示请求序号过旧、已被忽略。"""
        if linear not in (-1, 0, 1) or angular not in (-1, 0, 1):
            raise ValueError("linear and angular must be -1, 0, or 1")
        with self._state_lock:
            if sequence <= self._last_sequence:
                return False
            self._last_sequence = sequence
            self._linear_direction = linear
            self._angular_direction = angular
            self._last_heartbeat = time.monotonic()
            self._last_error = None
        self._wake_event.set()
        return True

    def stop_now(self, sequence: Optional[int] = None) -> bool:
        """更新为零方向并在发送锁内立即重复发送零速度。"""
        with self._state_lock:
            if sequence is not None and sequence <= self._last_sequence:
                return False
            if sequence is not None:
                self._last_sequence = sequence
            self._linear_direction = 0
            self._angular_direction = 0
            self._last_heartbeat = time.monotonic()
        self._send_zero(repeat=5)
        self._wake_event.set()
        return True

    def status(self) -> Dict[str, Any]:
        with self._state_lock:
            age = time.monotonic() - self._last_heartbeat
            return {
                "linear_direction": self._linear_direction,
                "angular_direction": self._angular_direction,
                "heartbeat_age": age,
                "last_sequence": self._last_sequence,
                "last_error": self._last_error,
                "watchdog_stops": self._watchdog_stops,
            }

    def close(self) -> None:
        """停止线程并执行最终速度清零。"""
        with self._state_lock:
            self._linear_direction = 0
            self._angular_direction = 0
        self._stop_event.set()
        self._wake_event.set()
        if self._worker is not None:
            self._worker.join(timeout=1.0)
        self._send_zero(repeat=10)

    def _send_zero(self, repeat: int) -> None:
        last_error: Optional[Exception] = None
        with self._send_lock:
            for _ in range(max(1, repeat)):
                try:
                    result = self.client.robot_motion(0.0, 0.0, 0.0)
                    if self._rejected(result):
                        last_error = RuntimeError(
                            "zero velocity rejected: {}".format(result)
                        )
                    else:
                        last_error = None
                except Exception as exc:
                    last_error = exc
                time.sleep(0.02)
        if last_error is not None:
            raise RuntimeError("zero velocity failed: {}".format(last_error))

    def _send_motion(self, linear: int, angular: int) -> None:
        vx = linear * self.linear_speed
        vw = angular * self.angular_speed
        with self._send_lock:
            result = self.client.robot_motion(vx=vx, vy=0.0, vw=vw)
        if self._rejected(result):
            raise RuntimeError("RobotMotion rejected: {}".format(result))

    def _run(self) -> None:
        was_moving = False
        while not self._stop_event.is_set():
            with self._state_lock:
                linear = self._linear_direction
                angular = self._angular_direction
                heartbeat_age = time.monotonic() - self._last_heartbeat

                watchdog_expired = (
                    (linear != 0 or angular != 0)
                    and heartbeat_age > self.watchdog_timeout
                )
                if watchdog_expired:
                    self._linear_direction = 0
                    self._angular_direction = 0
                    self._watchdog_stops += 1
                    linear = 0
                    angular = 0

            moving = linear != 0 or angular != 0
            try:
                if moving:
                    self._send_motion(linear, angular)
                elif was_moving or watchdog_expired:
                    self._send_zero(repeat=5)
            except Exception as exc:
                with self._state_lock:
                    self._linear_direction = 0
                    self._angular_direction = 0
                    self._last_error = str(exc)
                try:
                    self._send_zero(repeat=8)
                except Exception as stop_exc:
                    with self._state_lock:
                        self._last_error = "{}; stop failed: {}".format(exc, stop_exc)
            was_moving = moving
            self._wake_event.wait(self.send_interval)
            self._wake_event.clear()


class ControlRequestHandler(BaseHTTPRequestHandler):
    """网页和控制 API；实例属性由服务器对象提供。"""

    server: "ControlHttpServer"

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if not self._authorized(parsed):
            self._send_json(HTTPStatus.FORBIDDEN, {"error": "invalid token"})
            return
        if parsed.path == "/":
            page = CONTROL_PAGE.replace(
                "__ACCESS_TOKEN__",
                json.dumps(self.server.access_token),
            ).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(page)
            return
        if parsed.path == "/status":
            self._send_json(HTTPStatus.OK, self.server.controller.status())
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if not self._authorized(parsed):
            self._send_json(HTTPStatus.FORBIDDEN, {"error": "invalid token"})
            return
        try:
            if parsed.path == "/motion":
                payload = self._read_json()
                accepted = self.server.controller.update(
                    linear=int(payload["linear"]),
                    angular=int(payload["angular"]),
                    sequence=int(payload["sequence"]),
                )
                self._send_json(HTTPStatus.OK, {"accepted": accepted})
                return
            if parsed.path == "/stop":
                query = parse_qs(parsed.query)
                sequence_text = query.get("sequence", [None])[0]
                sequence = int(sequence_text) if sequence_text is not None else None
                accepted = self.server.controller.stop_now(sequence=sequence)
                self._send_json(HTTPStatus.OK, {"accepted": accepted})
                return
        except (KeyError, TypeError, ValueError) as exc:
            self._send_json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        except Exception as exc:
            self._send_json(HTTPStatus.INTERNAL_SERVER_ERROR, {"error": str(exc)})
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def log_message(self, format_text: str, *args: object) -> None:
        return

    def _authorized(self, parsed: Any) -> bool:
        query_token = parse_qs(parsed.query).get("token", [""])[0]
        header_token = self.headers.get("X-Control-Token", "")
        supplied = header_token or query_token
        return secrets.compare_digest(supplied, self.server.access_token)

    def _read_json(self) -> Dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 2048:
            raise ValueError("invalid content length")
        return json.loads(self.rfile.read(content_length).decode("utf-8"))

    def _send_json(self, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


class ControlHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
            self,
            address: Tuple[str, int],
            controller: SafeMotionController,
            access_token: str):
        super().__init__(address, ControlRequestHandler)
        self.controller = controller
        self.access_token = access_token


def discover_local_ip() -> str:
    """尽量获得浏览器可访问的上位机局域网地址。"""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("192.168.26.22", 8888))
        return probe.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "<上位机IP>"
    finally:
        probe.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Jaten 底盘浏览器键盘遥控")
    parser.add_argument("--execute", action="store_true", help="实际启动控制服务")
    parser.add_argument("--chassis-host", default="192.168.26.22")
    parser.add_argument("--chassis-port", type=int, default=8888)
    parser.add_argument("--authorization", default=None, help="可选底盘 Authorization")
    parser.add_argument("--bind", default="0.0.0.0", help="网页监听地址")
    parser.add_argument("--web-port", type=int, default=8765, help="网页监听端口")
    parser.add_argument("--access-token", default=None, help="可选固定网页访问令牌")
    parser.add_argument("--linear-speed", type=float, default=0.03)
    parser.add_argument("--angular-speed", type=float, default=0.05)
    parser.add_argument("--watchdog", type=float, default=0.30)
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if not 0.0 < args.linear_speed <= 0.05:
        parser.error("--linear-speed 必须在 (0, 0.05] 范围内")
    if not 0.0 < args.angular_speed <= 0.10:
        parser.error("--angular-speed 必须在 (0, 0.10] 范围内")
    if not 0.20 <= args.watchdog <= 0.60:
        parser.error("--watchdog 必须在 [0.20, 0.60] 范围内")
    if not 1 <= args.web_port <= 65535:
        parser.error("--web-port 超出有效范围")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)

    print("Jaten 浏览器键盘遥控")
    print("底盘: http://{}:{}".format(args.chassis_host, args.chassis_port))
    print("线速度: {:.3f}m/s，角速度: {:.3f}rad/s，看门狗: {:.2f}s".format(
        args.linear_speed,
        args.angular_speed,
        args.watchdog,
    ))
    if not args.execute:
        print("DRY RUN：未连接底盘，也未启动网页。添加 --execute 后实际运行。")
        return 0

    access_token = args.access_token or secrets.token_urlsafe(18)
    client = ChassisHttpClient(
        host=args.chassis_host,
        port=args.chassis_port,
        token=args.authorization,
        timeout=1.0,
    )
    controller = SafeMotionController(
        client=client,
        linear_speed=args.linear_speed,
        angular_speed=args.angular_speed,
        watchdog_timeout=args.watchdog,
    )

    try:
        mode_result = controller.start()
    except Exception as exc:
        print("无法确认 MANUAL 模式或清零底盘速度:", exc)
        return 1

    try:
        server = ControlHttpServer(
            (args.bind, args.web_port),
            controller=controller,
            access_token=access_token,
        )
    except Exception as exc:
        print("无法启动网页监听端口:", exc)
        try:
            controller.close()
        except Exception as stop_exc:
            print("警告：启动失败后的零速度请求失败:", stop_exc)
        return 1
    local_ip = discover_local_ip()
    print("MANUAL 模式已确认:", mode_result)
    print("浏览器控制地址:")
    print("http://{}:{}/?token={}".format(local_ip, args.web_port, access_token))
    print("Ctrl+C 退出；退出时会重复发送零速度。")

    def interrupt(_signum: int, _frame: object) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    return_code = 0
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        print("\n正在停止网页控制服务...")
    except Exception as exc:
        print("网页控制服务异常:", exc)
        return_code = 1
    finally:
        server.server_close()
        try:
            controller.close()
            print("已发送最终零速度，底盘保持 MANUAL 模式。")
        except Exception as exc:
            print("警告：最终零速度请求失败:", exc)
            return_code = 1
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
