#!/usr/bin/env python3
"""ROS-free AILinker daemon.

Southbound:
- Connects to atrbridge_ws_gateway over WS/JSON

Northbound:
- Exposes local HTTP tool server for AI behavior generator
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

from metrics import CsvMetricLogger, now_mono_us


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _to_json_bytes(data: Any) -> bytes:
    return json.dumps(data, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def _tool_schema() -> list[dict[str, Any]]:
    return [
        {
            "name": "get_robot_capabilities",
            "description": "Return capabilities, limits, and runtime link status.",
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "name": "get_robot_state",
            "description": "Return current cached telemetry and runtime state.",
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "name": "send_cmd_vel",
            "description": "Send safe bounded cmd_vel for bounded duration.",
            "parameters": {
                "type": "object",
                "properties": {
                    "vx": {"type": "number"},
                    "wz": {"type": "number"},
                    "duration_ms": {"type": "integer"},
                },
                "required": ["vx", "wz", "duration_ms"],
            },
        },
        {
            "name": "stop_robot",
            "description": "Immediate stop command.",
            "parameters": {"type": "object", "properties": {}},
        },
        {
            "name": "estop",
            "description": "Emergency stop command.",
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
        self.last_poll_rx_mono_us = 0
        self.last_caps_rx_mono_us = 0
        self.last_error = ""
        self.last_cmd_mono_us = 0
        self.active_motion_until_mono_us = 0
        self.last_cmd = {"vx": 0.0, "wz": 0.0, "duration_ms": 0}
        self.estop_count = 0
        self.poll_count = 0
        self.cmd_count = 0
        self.cmd_fail_count = 0

    def set_connected(self, connected: bool) -> None:
        with self._lock:
            self.gateway_connected = connected
            if not connected:
                self.active_motion_until_mono_us = 0

    def set_error(self, message: str) -> None:
        with self._lock:
            self.last_error = message

    def update_capabilities(self, caps: Dict[str, Any]) -> None:
        with self._lock:
            self.capabilities = caps
            self.last_caps_rx_mono_us = now_mono_us()

    def update_telemetry(self, telemetry: Dict[str, Any]) -> None:
        with self._lock:
            self.telemetry = telemetry
            self.last_poll_rx_mono_us = now_mono_us()
            self.poll_count += 1

    def mark_cmd_start(self, vx: float, wz: float, duration_ms: int) -> None:
        now = now_mono_us()
        with self._lock:
            self.last_cmd_mono_us = now
            self.active_motion_until_mono_us = now + max(0, duration_ms) * 1000
            self.last_cmd = {"vx": vx, "wz": wz, "duration_ms": duration_ms}
            self.cmd_count += 1

    def mark_cmd_fail(self) -> None:
        with self._lock:
            self.cmd_fail_count += 1

    def refresh_cmd_heartbeat(self) -> None:
        with self._lock:
            self.last_cmd_mono_us = now_mono_us()

    def clear_motion(self) -> None:
        with self._lock:
            self.active_motion_until_mono_us = 0
            self.last_cmd = {"vx": 0.0, "wz": 0.0, "duration_ms": 0}

    def mark_estop(self) -> None:
        with self._lock:
            self.estop_count += 1
            self.active_motion_until_mono_us = 0
            self.last_cmd = {"vx": 0.0, "wz": 0.0, "duration_ms": 0}

    def motion_timing(self) -> tuple[int, int]:
        with self._lock:
            return self.last_cmd_mono_us, self.active_motion_until_mono_us

    def current_caps(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.capabilities)

    def snapshot(self) -> Dict[str, Any]:
        now = now_mono_us()
        with self._lock:
            return {
                "gateway": {
                    "connected": self.gateway_connected,
                    "last_poll_age_ms": int((now - self.last_poll_rx_mono_us) / 1000)
                    if self.last_poll_rx_mono_us > 0
                    else -1,
                    "last_caps_age_ms": int((now - self.last_caps_rx_mono_us) / 1000)
                    if self.last_caps_rx_mono_us > 0
                    else -1,
                    "last_error": self.last_error,
                },
                "runtime": self.capabilities.get("runtime", {}),
                "telemetry": self.telemetry,
                "last_cmd": self.last_cmd,
                "active_motion_remaining_ms": int(max(0, self.active_motion_until_mono_us - now) / 1000),
                "counters": {
                    "poll_count": self.poll_count,
                    "cmd_count": self.cmd_count,
                    "cmd_fail_count": self.cmd_fail_count,
                    "estop_count": self.estop_count,
                },
            }


class AILinkerService:
    def __init__(
        self,
        gateway_ws_url: str,
        poll_hz: float,
        caps_refresh_hz: float,
        safety: SafetyConfig,
        metrics_csv_path: str,
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
        self.metrics = CsvMetricLogger(
            metrics_csv_path,
            [
                "wall_time_iso",
                "mono_us",
                "component",
                "event",
                "req_type",
                "tool",
                "ok",
                "latency_us",
                "rtt_us",
                "vx",
                "wz",
                "duration_ms",
                "keepalive_count",
                "http_status",
                "error",
                "detail",
            ],
        )

    def _metric(self, **kwargs: object) -> None:
        row = {"component": "ailinker"}
        row.update(kwargs)
        self.metrics.log(row)

    async def run(self) -> None:
        self._metric(event="daemon_start", ok=True, detail=f"gateway={self.gateway_ws_url}")
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
                    self.state.set_connected(True)
                    self.state.set_error("")
                    self._metric(event="gateway_connected", ok=True)
                    await self._loop_connected()
            except Exception as exc:  # pragma: no cover
                self.state.set_error(f"gateway_connect_error: {exc}")
                self._metric(event="gateway_connect_error", ok=False, error=str(exc))
            finally:
                self._ws = None
                self.state.set_connected(False)
            await asyncio.sleep(0.5)

    async def shutdown(self) -> None:
        self.stop_event.set()
        ws = self._ws
        if ws is not None:
            await ws.close()
        self._metric(event="daemon_stop", ok=True)

    async def _loop_connected(self) -> None:
        next_poll = 0.0
        next_caps = 0.0
        while not self.stop_event.is_set():
            now = time.monotonic()
            if now >= next_caps:
                await self._refresh_capabilities()
                next_caps = now + self.caps_period_s
            if now >= next_poll:
                await self._refresh_telemetry()
                next_poll = now + self.poll_period_s
            await self._safety_tick()
            await asyncio.sleep(0.01)

    async def _request_gateway(
        self,
        payload: Dict[str, Any],
        req_type: str,
        timeout_s: float = 1.0,
        log_ok: bool = False,
    ) -> tuple[Dict[str, Any], int]:
        ws = self._ws
        if ws is None:
            raise RuntimeError("gateway_not_connected")

        t0 = now_mono_us()
        async with self._ws_lock:
            await ws.send(json.dumps(payload, separators=(",", ":"), ensure_ascii=True))
            raw = await asyncio.wait_for(ws.recv(), timeout=timeout_s)
        t1 = now_mono_us()
        rtt_us = t1 - t0

        if not isinstance(raw, str):
            raise RuntimeError("gateway_non_text_response")
        decoded = json.loads(raw)
        if not isinstance(decoded, dict):
            raise RuntimeError("gateway_bad_json")

        ok = bool(decoded.get("ok", True))
        if log_ok or not ok:
            self._metric(event="gw_req", req_type=req_type, ok=ok, rtt_us=rtt_us, detail=str(decoded))
        return decoded, rtt_us

    async def _refresh_capabilities(self) -> None:
        try:
            msg, rtt_us = await self._request_gateway({"type": "get_capabilities"}, "get_capabilities")
            caps = msg.get("capabilities", {})
            if isinstance(caps, dict):
                self.state.update_capabilities(caps)
            self._metric(event="capabilities_rx", req_type="get_capabilities", ok=True, rtt_us=rtt_us)
        except Exception as exc:  # pragma: no cover
            self.state.set_error(f"capabilities_refresh_error: {exc}")
            self._metric(event="capabilities_rx", req_type="get_capabilities", ok=False, error=str(exc))

    async def _refresh_telemetry(self) -> None:
        try:
            msg, rtt_us = await self._request_gateway({"type": "poll"}, "poll")
            telem = {
                "imu": msg.get("imu", {}),
                "ir": msg.get("ir", {}),
                "tof": msg.get("tof", {}),
            }
            self.state.update_telemetry(telem)
            detail = f"tof={telem['tof'].get('range_m', 'na')}"
            self._metric(event="telemetry_rx", req_type="poll", ok=True, rtt_us=rtt_us, detail=detail)
        except Exception as exc:  # pragma: no cover
            self.state.set_error(f"telemetry_refresh_error: {exc}")
            self._metric(event="telemetry_rx", req_type="poll", ok=False, error=str(exc))

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
        last_cmd_us, active_until_us = self.state.motion_timing()
        now_us = now_mono_us()
        if active_until_us > 0 and now_us >= active_until_us:
            await self.stop_robot(reason="duration_elapsed")
            return
        deadman_us = max(100_000, self.safety.deadman_timeout_ms * 1000)
        if active_until_us > 0 and last_cmd_us > 0 and (now_us - last_cmd_us) >= deadman_us:
            await self.stop_robot(reason="deadman_timeout")

    async def get_robot_capabilities(self) -> Dict[str, Any]:
        return {"ok": True, "capabilities": self.state.current_caps()}

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

        exec_start_us = now_mono_us()
        async with self._motion_lock:
            try:
                first_ack, first_rtt = await self._request_gateway(
                    {"type": "cmd_vel", "vx": bounded_vx, "wz": bounded_wz},
                    "cmd_vel",
                    timeout_s=1.5,
                    log_ok=True,
                )
            except Exception as exc:
                self.state.mark_cmd_fail()
                self._metric(
                    event="cmd_exec",
                    tool="send_cmd_vel",
                    ok=False,
                    vx=bounded_vx,
                    wz=bounded_wz,
                    duration_ms=bounded_duration,
                    error=str(exc),
                )
                return {"ok": False, "error": f"send_failed:{exc}"}

            if not bool(first_ack.get("ok", False)):
                self.state.mark_cmd_fail()
                self._metric(
                    event="cmd_exec",
                    tool="send_cmd_vel",
                    ok=False,
                    vx=bounded_vx,
                    wz=bounded_wz,
                    duration_ms=bounded_duration,
                    rtt_us=first_rtt,
                    detail=str(first_ack),
                )
                return {"ok": False, "error": "cmd_ack_false", "raw_ack": first_ack}

            self.state.mark_cmd_start(bounded_vx, bounded_wz, bounded_duration)
            keepalive_count = 0
            keepalive_ok = True
            keepalive_period_s = max(0.08, min(0.25, self.safety.deadman_timeout_ms / 3000.0))

            if bounded_duration > 0 and (abs(bounded_vx) > 1e-6 or abs(bounded_wz) > 1e-6):
                deadline_s = time.monotonic() + bounded_duration / 1000.0
                while time.monotonic() < deadline_s:
                    await asyncio.sleep(min(keepalive_period_s, max(0.0, deadline_s - time.monotonic())))
                    if time.monotonic() >= deadline_s:
                        break
                    try:
                        ack, _ = await self._request_gateway(
                            {"type": "cmd_vel", "vx": bounded_vx, "wz": bounded_wz},
                            "cmd_vel_keepalive",
                            timeout_s=1.5,
                        )
                    except Exception as exc:
                        keepalive_ok = False
                        self.state.set_error(f"cmd_keepalive_error:{exc}")
                        break
                    if not bool(ack.get("ok", False)):
                        keepalive_ok = False
                        self.state.set_error("cmd_keepalive_ack_false")
                        break
                    self.state.refresh_cmd_heartbeat()
                    keepalive_count += 1

                stop_res = await self.stop_robot(reason="duration_elapsed")
                elapsed_us = now_mono_us() - exec_start_us
                ok = keepalive_ok and bool(stop_res.get("ok", False))
                if not ok:
                    self.state.mark_cmd_fail()
                self._metric(
                    event="cmd_exec",
                    tool="send_cmd_vel",
                    ok=ok,
                    vx=bounded_vx,
                    wz=bounded_wz,
                    duration_ms=bounded_duration,
                    latency_us=elapsed_us,
                    rtt_us=first_rtt,
                    keepalive_count=keepalive_count,
                    detail=str(stop_res),
                )
                return {
                    "ok": ok,
                    "applied": {"vx": bounded_vx, "wz": bounded_wz, "duration_ms": bounded_duration},
                    "limits": {
                        "max_vx_mps": max_vx,
                        "max_wz_rps": max_wz,
                        "max_duration_ms": self.safety.max_duration_ms,
                    },
                    "raw_ack": first_ack,
                    "keepalive_count": keepalive_count,
                    "auto_stop": stop_res,
                }

            elapsed_us = now_mono_us() - exec_start_us
            self._metric(
                event="cmd_exec",
                tool="send_cmd_vel",
                ok=True,
                vx=bounded_vx,
                wz=bounded_wz,
                duration_ms=bounded_duration,
                latency_us=elapsed_us,
                rtt_us=first_rtt,
                keepalive_count=0,
            )
            return {
                "ok": True,
                "applied": {"vx": bounded_vx, "wz": bounded_wz, "duration_ms": bounded_duration},
                "limits": {
                    "max_vx_mps": max_vx,
                    "max_wz_rps": max_wz,
                    "max_duration_ms": self.safety.max_duration_ms,
                },
                "raw_ack": first_ack,
            }

    async def stop_robot(self, reason: str = "manual") -> Dict[str, Any]:
        try:
            ack, rtt_us = await self._request_gateway(
                {"type": "cmd_vel", "vx": 0.0, "wz": 0.0},
                "stop_robot",
                timeout_s=1.5,
                log_ok=True,
            )
        except Exception as exc:
            self.state.mark_cmd_fail()
            self._metric(event="stop", tool="stop_robot", ok=False, error=str(exc), detail=reason)
            return {"ok": False, "error": f"stop_failed:{exc}"}

        ok = bool(ack.get("ok", False))
        if ok:
            self.state.clear_motion()
        else:
            self.state.mark_cmd_fail()
        self._metric(event="stop", tool="stop_robot", ok=ok, rtt_us=rtt_us, detail=reason)
        return {"ok": ok, "reason": reason, "raw_ack": ack}

    async def estop(self) -> Dict[str, Any]:
        try:
            ack, rtt_us = await self._request_gateway(
                {"type": "estop"},
                "estop",
                timeout_s=1.5,
                log_ok=True,
            )
        except Exception as exc:
            self.state.mark_cmd_fail()
            self._metric(event="estop", tool="estop", ok=False, error=str(exc))
            return {"ok": False, "error": f"estop_failed:{exc}"}

        ok = bool(ack.get("ok", False))
        if ok:
            self.state.mark_estop()
        else:
            self.state.mark_cmd_fail()
        self._metric(event="estop", tool="estop", ok=ok, rtt_us=rtt_us)
        return {"ok": ok, "raw_ack": ack}

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
            "metrics_csv_path": self.metrics.path,
        }


class HttpBridge:
    def __init__(self, service: AILinkerService, loop: asyncio.AbstractEventLoop) -> None:
        self.service = service
        self.loop = loop

    def call_tool(self, tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        timeout_s = 2.5
        if tool_name == "send_cmd_vel":
            duration_ms = int(args.get("duration_ms", 0))
            timeout_s = max(3.0, min(20.0, duration_ms / 1000.0 + 3.0))
        fut = asyncio.run_coroutine_threadsafe(self.service.invoke_tool(tool_name, args), self.loop)
        try:
            return fut.result(timeout=timeout_s)
        except Exception as exc:
            return {"ok": False, "error": f"tool_call_failed:{exc}"}

    def context(self) -> Dict[str, Any]:
        return self.service.context_json()

    def state(self) -> Dict[str, Any]:
        return {"ok": True, "state": self.service.state.snapshot()}

    def metrics(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "metrics_csv_path": self.service.metrics.path,
            "state_counters": self.service.state.snapshot().get("counters", {}),
        }


class ToolHandler(BaseHTTPRequestHandler):
    server_version = "AILinkerStandalone/0.1"
    bridge: HttpBridge

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path == "/capabilities":
            self._send(HTTPStatus.OK, self.bridge.call_tool("get_robot_capabilities", {}))
            return
        if path == "/state":
            self._send(HTTPStatus.OK, self.bridge.state())
            return
        if path == "/context":
            self._send(HTTPStatus.OK, self.bridge.context())
            return
        if path == "/metrics":
            self._send(HTTPStatus.OK, self.bridge.metrics())
            return
        self._send(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        body = self._body_json()
        if body is None:
            return
        if path == "/cmd_vel":
            res = self.bridge.call_tool("send_cmd_vel", body)
            self._send(HTTPStatus.OK if res.get("ok", False) else HTTPStatus.BAD_REQUEST, res)
            return
        if path == "/stop":
            res = self.bridge.call_tool("stop_robot", {})
            self._send(HTTPStatus.OK if res.get("ok", False) else HTTPStatus.BAD_REQUEST, res)
            return
        if path == "/estop":
            res = self.bridge.call_tool("estop", {})
            self._send(HTTPStatus.OK if res.get("ok", False) else HTTPStatus.BAD_REQUEST, res)
            return
        if path == "/rpc":
            method = str(body.get("method", ""))
            params = body.get("params", {})
            res = self.bridge.call_tool(method, params if isinstance(params, dict) else {})
            self._send(HTTPStatus.OK, {"jsonrpc": "2.0", "result": res, "id": body.get("id")})
            return
        self._send(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not_found"})

    def _body_json(self) -> Optional[Dict[str, Any]]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            data = json.loads(raw.decode("utf-8"))
        except Exception:
            self._send(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "bad_json"})
            return None
        if not isinstance(data, dict):
            self._send(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "json_must_be_object"})
            return None
        return data

    def _send(self, status: HTTPStatus, payload: Dict[str, Any]) -> None:
        data = _to_json_bytes(payload)
        self.send_response(int(status))
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


async def _run(args: argparse.Namespace) -> None:
    service = AILinkerService(
        gateway_ws_url=args.gateway_ws_url,
        poll_hz=args.poll_hz,
        caps_refresh_hz=args.capabilities_refresh_hz,
        safety=SafetyConfig(
            max_duration_ms=args.max_duration_ms,
            deadman_timeout_ms=args.deadman_timeout_ms,
            max_vx_override=args.max_vx_override,
            max_wz_override=args.max_wz_override,
        ),
        metrics_csv_path=args.metrics_csv_path,
    )

    loop = asyncio.get_running_loop()
    ToolHandler.bridge = HttpBridge(service, loop)
    httpd = ThreadingHTTPServer((args.bind_host, args.bind_port), ToolHandler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    print(
        f"ailinker_standalone: http://{args.bind_host}:{args.bind_port} "
        f"gateway={args.gateway_ws_url} metrics={service.metrics.path}"
    )

    stop_evt = asyncio.Event()

    def _on_signal() -> None:
        stop_evt.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _on_signal)
        except NotImplementedError:  # pragma: no cover
            pass

    task = asyncio.create_task(service.run())
    await stop_evt.wait()
    await service.shutdown()
    await asyncio.wait_for(task, timeout=2.0)
    httpd.shutdown()


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ROS-free AILinker daemon")
    p.add_argument("--gateway-ws-url", default="ws://127.0.0.1:8080/ws")
    p.add_argument("--bind-host", default="127.0.0.1")
    p.add_argument("--bind-port", type=int, default=7070)
    p.add_argument("--poll-hz", type=float, default=20.0)
    p.add_argument("--capabilities-refresh-hz", type=float, default=1.0)
    p.add_argument("--max-duration-ms", type=int, default=2000)
    p.add_argument("--deadman-timeout-ms", type=int, default=800)
    p.add_argument("--max-vx-override", type=float, default=None)
    p.add_argument("--max-wz-override", type=float, default=None)
    p.add_argument("--metrics-csv-path", default="/tmp/minitb_ai_pipeline/ailinker_metrics.csv")
    return p.parse_args()


def main() -> None:
    args = _args()
    try:
        asyncio.run(_run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

