#!/usr/bin/env python3
"""AILinker tool server.

Layer 3 in the MiniTB stack:
- southbound: talks to atrbridge_ws_gateway over WebSocket/JSON
- northbound: exposes safe local HTTP tools for orchestrators
"""

from __future__ import annotations

import argparse
import asyncio
import json
import signal
import threading
import time
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional
from urllib.parse import urlparse

import websockets


def _now_s() -> float:
    return time.monotonic()


def _json_dumps(data: Any) -> bytes:
    return json.dumps(data, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _tool_schema() -> list[dict[str, Any]]:
    return [
        {
            "name": "get_robot_capabilities",
            "description": "Get current robot capability descriptor and runtime link status.",
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "name": "get_robot_state",
            "description": "Get latest cached telemetry and runtime state from AILinker.",
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "name": "send_cmd_vel",
            "description": "Send bounded velocity command to robot for a limited duration.",
            "parameters": {
                "type": "object",
                "properties": {
                    "vx": {"type": "number", "description": "Linear velocity in m/s"},
                    "wz": {"type": "number", "description": "Angular velocity in rad/s"},
                    "duration_ms": {
                        "type": "integer",
                        "description": "Command hold duration before auto-stop",
                    },
                },
                "required": ["vx", "wz", "duration_ms"],
            },
        },
        {
            "name": "stop_robot",
            "description": "Immediate zero-velocity stop command.",
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "name": "estop",
            "description": "Emergency stop request to gateway/robot.",
            "parameters": {"type": "object", "properties": {}},
        },
    ]


@dataclass
class SafetyConfig:
    max_duration_ms: int = 2000
    deadman_timeout_ms: int = 800
    max_vx_override: Optional[float] = None
    max_wz_override: Optional[float] = None


class SharedState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.capabilities: Dict[str, Any] = {}
        self.telemetry: Dict[str, Any] = {"imu": {}, "ir": {}, "tof": {}}
        self.gateway_connected = False
        self.last_poll_rx_s = 0.0
        self.last_caps_rx_s = 0.0
        self.last_error = ""
        self.last_cmd_s = 0.0
        self.active_motion_until_s = 0.0
        self.last_cmd = {"vx": 0.0, "wz": 0.0, "duration_ms": 0}
        self.estop_count = 0

    def set_gateway_connected(self, connected: bool) -> None:
        with self._lock:
            self.gateway_connected = connected
            if not connected:
                self.active_motion_until_s = 0.0

    def set_error(self, message: str) -> None:
        with self._lock:
            self.last_error = message

    def update_capabilities(self, caps: Dict[str, Any]) -> None:
        with self._lock:
            self.capabilities = caps
            self.last_caps_rx_s = _now_s()

    def update_telemetry(self, telemetry: Dict[str, Any]) -> None:
        with self._lock:
            self.telemetry = telemetry
            self.last_poll_rx_s = _now_s()

    def mark_cmd(self, vx: float, wz: float, duration_ms: int) -> None:
        now = _now_s()
        with self._lock:
            self.last_cmd_s = now
            self.active_motion_until_s = now + max(0.0, duration_ms / 1000.0)
            self.last_cmd = {"vx": vx, "wz": wz, "duration_ms": duration_ms}

    def refresh_cmd_heartbeat(self) -> None:
        with self._lock:
            self.last_cmd_s = _now_s()

    def clear_motion(self) -> None:
        with self._lock:
            self.active_motion_until_s = 0.0
            self.last_cmd = {"vx": 0.0, "wz": 0.0, "duration_ms": 0}

    def mark_estop(self) -> None:
        with self._lock:
            self.estop_count += 1
            self.active_motion_until_s = 0.0
            self.last_cmd = {"vx": 0.0, "wz": 0.0, "duration_ms": 0}

    def snapshot(self) -> Dict[str, Any]:
        now = _now_s()
        with self._lock:
            caps = self.capabilities
            return {
                "gateway": {
                    "connected": self.gateway_connected,
                    "last_poll_age_ms": int((now - self.last_poll_rx_s) * 1000.0)
                    if self.last_poll_rx_s > 0.0
                    else -1,
                    "last_caps_age_ms": int((now - self.last_caps_rx_s) * 1000.0)
                    if self.last_caps_rx_s > 0.0
                    else -1,
                    "last_error": self.last_error,
                },
                "runtime": caps.get("runtime", {}),
                "telemetry": self.telemetry,
                "last_cmd": self.last_cmd,
                "active_motion_remaining_ms": int(max(0.0, self.active_motion_until_s - now) * 1000.0),
                "estop_count": self.estop_count,
            }

    def current_caps(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.capabilities)

    def motion_timing(self) -> tuple[float, float]:
        with self._lock:
            return self.last_cmd_s, self.active_motion_until_s


class AILinkerService:
    def __init__(
        self,
        gateway_ws_url: str,
        poll_hz: float,
        caps_refresh_hz: float,
        safety: SafetyConfig,
    ) -> None:
        self.gateway_ws_url = gateway_ws_url
        self.poll_period_s = 1.0 / max(0.2, poll_hz)
        self.caps_period_s = 1.0 / max(0.1, caps_refresh_hz)
        self.safety = safety
        self.state = SharedState()
        self.stop_event = asyncio.Event()
        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._ws_lock = asyncio.Lock()
        self._motion_lock = asyncio.Lock()

    async def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                async with websockets.connect(
                    self.gateway_ws_url,
                    open_timeout=3.0,
                    ping_interval=20.0,
                    ping_timeout=20.0,
                    max_queue=1,
                ) as ws:
                    self._ws = ws
                    self.state.set_gateway_connected(True)
                    self.state.set_error("")
                    await self._loop_connected()
            except Exception as exc:  # pragma: no cover - runtime network path
                self.state.set_error(f"gateway_connect_error: {exc}")
            finally:
                self._ws = None
                self.state.set_gateway_connected(False)
            await asyncio.sleep(0.5)

    async def shutdown(self) -> None:
        self.stop_event.set()
        ws = self._ws
        if ws is not None:
            await ws.close()

    async def _loop_connected(self) -> None:
        next_poll = 0.0
        next_caps = 0.0
        while not self.stop_event.is_set():
            now = _now_s()
            if now >= next_caps:
                await self._refresh_capabilities()
                next_caps = now + self.caps_period_s
            if now >= next_poll:
                await self._refresh_telemetry()
                next_poll = now + self.poll_period_s
            await self._safety_tick()
            await asyncio.sleep(0.01)

    async def _request_gateway(self, payload: Dict[str, Any], timeout_s: float = 1.0) -> Dict[str, Any]:
        ws = self._ws
        if ws is None:
            raise RuntimeError("gateway_not_connected")
        async with self._ws_lock:
            await ws.send(json.dumps(payload, separators=(",", ":"), ensure_ascii=True))
            raw = await asyncio.wait_for(ws.recv(), timeout=timeout_s)
        if not isinstance(raw, str):
            raise RuntimeError("gateway_non_text_response")
        decoded = json.loads(raw)
        if not isinstance(decoded, dict):
            raise RuntimeError("gateway_bad_json")
        return decoded

    async def _refresh_capabilities(self) -> None:
        try:
            msg = await self._request_gateway({"type": "get_capabilities"}, timeout_s=1.0)
            caps = msg.get("capabilities", {})
            if isinstance(caps, dict):
                self.state.update_capabilities(caps)
        except Exception as exc:  # pragma: no cover - runtime network path
            self.state.set_error(f"capabilities_refresh_error: {exc}")

    async def _refresh_telemetry(self) -> None:
        try:
            msg = await self._request_gateway({"type": "poll"}, timeout_s=1.0)
            telem = {
                "imu": msg.get("imu", {}),
                "ir": msg.get("ir", {}),
                "tof": msg.get("tof", {}),
            }
            self.state.update_telemetry(telem)
        except Exception as exc:  # pragma: no cover - runtime network path
            self.state.set_error(f"telemetry_refresh_error: {exc}")

    def _resolve_limits(self) -> tuple[float, float]:
        caps = self.state.current_caps()
        limits = caps.get("limits", {})
        max_vx = self.safety.max_vx_override
        max_wz = self.safety.max_wz_override
        if max_vx is None:
            max_vx = float(limits.get("max_vx_mps", 0.25))
        if max_wz is None:
            max_wz = float(limits.get("max_wz_rps", 1.0))
        return abs(max_vx), abs(max_wz)

    async def _safety_tick(self) -> None:
        last_cmd_s, active_until_s = self.state.motion_timing()
        now = _now_s()
        if active_until_s > 0.0 and now >= active_until_s:
            await self.stop_robot(reason="duration_elapsed")
            return
        deadman_s = max(0.1, self.safety.deadman_timeout_ms / 1000.0)
        if active_until_s > 0.0 and last_cmd_s > 0.0 and (now - last_cmd_s) >= deadman_s:
            await self.stop_robot(reason="deadman_timeout")

    async def get_robot_capabilities(self) -> Dict[str, Any]:
        caps = self.state.current_caps()
        return {"ok": True, "capabilities": caps}

    async def get_robot_state(self) -> Dict[str, Any]:
        return {"ok": True, "state": self.state.snapshot()}

    async def send_cmd_vel(self, vx: float, wz: float, duration_ms: int) -> Dict[str, Any]:
        caps = self.state.current_caps()
        runtime = caps.get("runtime", {})
        modules = caps.get("modules", {})
        if modules and not modules.get("base", True):
            return {"ok": False, "error": "base_module_not_available"}
        if runtime and not runtime.get("tcp_connected", False):
            return {"ok": False, "error": "robot_not_connected"}

        max_vx, max_wz = self._resolve_limits()
        bounded_vx = _clamp(float(vx), -max_vx, max_vx)
        bounded_wz = _clamp(float(wz), -max_wz, max_wz)
        bounded_duration = int(_clamp(float(duration_ms), 0.0, float(self.safety.max_duration_ms)))

        # Execute one motion tool call at a time and keep command alive for the requested duration.
        async with self._motion_lock:
            resp = await self._request_gateway({"type": "cmd_vel", "vx": bounded_vx, "wz": bounded_wz})
            ok = bool(resp.get("ok", False))
            if not ok:
                return {
                    "ok": False,
                    "applied": {"vx": bounded_vx, "wz": bounded_wz, "duration_ms": bounded_duration},
                    "limits": {
                        "max_vx_mps": max_vx,
                        "max_wz_rps": max_wz,
                        "max_duration_ms": self.safety.max_duration_ms,
                    },
                    "raw_ack": resp,
                }

            self.state.mark_cmd(bounded_vx, bounded_wz, bounded_duration)
            if bounded_duration > 0 and (abs(bounded_vx) > 1e-6 or abs(bounded_wz) > 1e-6):
                keepalive_period_s = max(0.08, min(0.25, self.safety.deadman_timeout_ms / 3000.0))
                deadline = _now_s() + bounded_duration / 1000.0
                keepalive_ok = True
                keepalive_count = 0
                while _now_s() < deadline:
                    await asyncio.sleep(min(keepalive_period_s, max(0.0, deadline - _now_s())))
                    if _now_s() >= deadline:
                        break
                    keepalive_resp = await self._request_gateway(
                        {"type": "cmd_vel", "vx": bounded_vx, "wz": bounded_wz}
                    )
                    keepalive_ok = bool(keepalive_resp.get("ok", False))
                    if not keepalive_ok:
                        self.state.set_error("cmd_keepalive_failed")
                        break
                    self.state.refresh_cmd_heartbeat()
                    keepalive_count += 1

                stop_res = await self.stop_robot(reason="duration_elapsed")
                return {
                    "ok": keepalive_ok and bool(stop_res.get("ok", False)),
                    "applied": {"vx": bounded_vx, "wz": bounded_wz, "duration_ms": bounded_duration},
                    "limits": {
                        "max_vx_mps": max_vx,
                        "max_wz_rps": max_wz,
                        "max_duration_ms": self.safety.max_duration_ms,
                    },
                    "raw_ack": resp,
                    "keepalive_count": keepalive_count,
                    "auto_stop": stop_res,
                }

            return {
                "ok": True,
                "applied": {"vx": bounded_vx, "wz": bounded_wz, "duration_ms": bounded_duration},
                "limits": {
                    "max_vx_mps": max_vx,
                    "max_wz_rps": max_wz,
                    "max_duration_ms": self.safety.max_duration_ms,
                },
                "raw_ack": resp,
            }

    async def stop_robot(self, reason: str = "manual") -> Dict[str, Any]:
        resp = await self._request_gateway({"type": "cmd_vel", "vx": 0.0, "wz": 0.0})
        ok = bool(resp.get("ok", False))
        if ok:
            self.state.clear_motion()
        return {"ok": ok, "reason": reason, "raw_ack": resp}

    async def estop(self) -> Dict[str, Any]:
        resp = await self._request_gateway({"type": "estop"})
        ok = bool(resp.get("ok", False))
        if ok:
            self.state.mark_estop()
        return {"ok": ok, "raw_ack": resp}

    async def invoke_tool(self, tool_name: str, args: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        args = args or {}
        if tool_name == "get_robot_capabilities":
            return await self.get_robot_capabilities()
        if tool_name == "get_robot_state":
            return await self.get_robot_state()
        if tool_name == "send_cmd_vel":
            return await self.send_cmd_vel(
                vx=float(args.get("vx", 0.0)),
                wz=float(args.get("wz", 0.0)),
                duration_ms=int(args.get("duration_ms", 0)),
            )
        if tool_name == "stop_robot":
            return await self.stop_robot(reason="tool_call")
        if tool_name == "estop":
            return await self.estop()
        return {"ok": False, "error": f"unknown_tool:{tool_name}"}

    def context_json(self) -> Dict[str, Any]:
        caps = self.state.current_caps()
        max_vx, max_wz = self._resolve_limits()
        return {
            "ok": True,
            "capabilities": caps,
            "state": self.state.snapshot(),
            "constraints": {
                "max_vx_mps": max_vx,
                "max_wz_rps": max_wz,
                "max_duration_ms": self.safety.max_duration_ms,
                "deadman_timeout_ms": self.safety.deadman_timeout_ms,
            },
            "tools": _tool_schema(),
        }


class ToolRequestHandler(BaseHTTPRequestHandler):
    server_version = "AILinkerHTTP/0.1"
    ailinker: "AILinkerHttpBridge"

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/capabilities":
            self._send_json(HTTPStatus.OK, self.ailinker.call_tool("get_robot_capabilities", {}))
            return
        if path == "/state":
            self._send_json(HTTPStatus.OK, self.ailinker.call_tool("get_robot_state", {}))
            return
        if path == "/context":
            self._send_json(HTTPStatus.OK, self.ailinker.get_context())
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        body = self._read_json_body()
        if body is None:
            return

        if path == "/cmd_vel":
            result = self.ailinker.call_tool("send_cmd_vel", body)
            self._send_json(HTTPStatus.OK if result.get("ok", False) else HTTPStatus.BAD_REQUEST, result)
            return
        if path == "/stop":
            result = self.ailinker.call_tool("stop_robot", {})
            self._send_json(HTTPStatus.OK if result.get("ok", False) else HTTPStatus.BAD_REQUEST, result)
            return
        if path == "/estop":
            result = self.ailinker.call_tool("estop", {})
            self._send_json(HTTPStatus.OK if result.get("ok", False) else HTTPStatus.BAD_REQUEST, result)
            return
        if path == "/rpc":
            method = str(body.get("method", ""))
            params = body.get("params", {})
            result = self.ailinker.call_tool(method, params if isinstance(params, dict) else {})
            response = {"jsonrpc": "2.0", "result": result, "id": body.get("id", None)}
            self._send_json(HTTPStatus.OK, response)
            return
        self._send_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})

    def _read_json_body(self) -> Optional[Dict[str, Any]]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            value = json.loads(raw.decode("utf-8"))
        except Exception:
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad_json"})
            return None
        if not isinstance(value, dict):
            self._send_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "json_must_be_object"})
            return None
        return value

    def _send_json(self, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        data = _json_dumps(payload)
        self.send_response(int(status))
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class AILinkerHttpBridge:
    def __init__(self, service: AILinkerService, loop: asyncio.AbstractEventLoop) -> None:
        self.service = service
        self.loop = loop

    def call_tool(self, tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        timeout_s = 2.0
        if tool_name == "send_cmd_vel":
            # send_cmd_vel blocks until requested duration completes (+ keepalive + auto-stop).
            duration_ms = int(args.get("duration_ms", 0))
            timeout_s = max(3.0, min(20.0, duration_ms / 1000.0 + 3.0))
        fut = asyncio.run_coroutine_threadsafe(self.service.invoke_tool(tool_name, args), self.loop)
        try:
            return fut.result(timeout=timeout_s)
        except Exception as exc:
            return {"ok": False, "error": f"tool_call_failed:{exc}"}

    def get_context(self) -> Dict[str, Any]:
        return self.service.context_json()


async def _run_server(args: argparse.Namespace) -> None:
    safety = SafetyConfig(
        max_duration_ms=args.max_duration_ms,
        deadman_timeout_ms=args.deadman_timeout_ms,
        max_vx_override=args.max_vx_override,
        max_wz_override=args.max_wz_override,
    )
    service = AILinkerService(
        gateway_ws_url=args.gateway_ws_url,
        poll_hz=args.poll_hz,
        caps_refresh_hz=args.capabilities_refresh_hz,
        safety=safety,
    )

    loop = asyncio.get_running_loop()
    bridge = AILinkerHttpBridge(service=service, loop=loop)
    ToolRequestHandler.ailinker = bridge
    httpd = ThreadingHTTPServer((args.bind_host, args.bind_port), ToolRequestHandler)
    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()
    print(
        f"ailinker_server: HTTP http://{args.bind_host}:{args.bind_port} "
        f"gateway={args.gateway_ws_url}"
    )

    stop_evt = asyncio.Event()

    def _on_signal() -> None:
        stop_evt.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _on_signal)
        except NotImplementedError:  # pragma: no cover - non-posix
            pass

    gateway_task = asyncio.create_task(service.run())
    await stop_evt.wait()
    await service.shutdown()
    await asyncio.wait_for(gateway_task, timeout=2.0)
    httpd.shutdown()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AILinker tool server for atrbridge_ws_gateway")
    parser.add_argument("--gateway-ws-url", default="ws://127.0.0.1:8080/ws")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--bind-port", type=int, default=7070)
    parser.add_argument("--poll-hz", type=float, default=20.0)
    parser.add_argument("--capabilities-refresh-hz", type=float, default=1.0)
    parser.add_argument("--max-duration-ms", type=int, default=2000)
    parser.add_argument("--deadman-timeout-ms", type=int, default=800)
    parser.add_argument("--max-vx-override", type=float, default=None)
    parser.add_argument("--max-wz-override", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    try:
        asyncio.run(_run_server(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
