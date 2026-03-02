#!/usr/bin/env python3
"""Standalone AI behavior generator (Gemini -> AILinker tools)."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import requests
from requests.exceptions import HTTPError, RequestException

from metrics import CsvMetricLogger, now_mono_us


def _json_compact(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True)


def _to_gemini_schema(node: Any) -> Any:
    if isinstance(node, dict):
        out = {}
        for key, val in node.items():
            if key == "type" and isinstance(val, str):
                mapping = {
                    "object": "OBJECT",
                    "array": "ARRAY",
                    "string": "STRING",
                    "number": "NUMBER",
                    "integer": "INTEGER",
                    "boolean": "BOOLEAN",
                }
                out[key] = mapping.get(val.lower(), val)
            else:
                out[key] = _to_gemini_schema(val)
        return out
    if isinstance(node, list):
        return [_to_gemini_schema(v) for v in node]
    return node


def _build_function_decls(tools: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [
        {
            "name": tool["name"],
            "description": tool.get("description", ""),
            "parameters": _to_gemini_schema(tool.get("parameters", {"type": "object"})),
        }
        for tool in tools
    ]


def _extract_calls_and_text(response_json: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], str]:
    candidates = response_json.get("candidates", [])
    if not candidates:
        return [], ""
    content = candidates[0].get("content", {})
    parts = content.get("parts", [])
    calls: List[Dict[str, Any]] = []
    text_chunks: List[str] = []
    for part in parts:
        if "functionCall" in part:
            fc = part["functionCall"]
            name = fc.get("name")
            args = fc.get("args", {})
            if name:
                calls.append({"name": name, "args": args if isinstance(args, dict) else {}})
        if "text" in part and isinstance(part["text"], str):
            text_chunks.append(part["text"])
    return calls, "\n".join(text_chunks).strip()


def _retry_after_seconds(resp: Optional[requests.Response]) -> float:
    if resp is None:
        return 0.0
    raw = resp.headers.get("Retry-After", "").strip()
    if not raw:
        return 0.0
    try:
        return max(0.0, float(raw))
    except ValueError:
        return 0.0


def _system_prompt(constraints: Dict[str, Any]) -> str:
    return (
        "You are the MiniTB AI behavior planner. "
        "Always use tools to act on robot state; never invent sensor values. "
        "Respect all safety constraints exactly. "
        "Prefer single-turn execution: output the complete sequence of tool calls in one response when possible. "
        "For fixed motion patterns (square, move X seconds, turn X degrees), emit all send_cmd_vel calls and "
        "finish with stop_robot in the same response. "
        "Use multi-turn only if the task explicitly needs fresh sensor feedback between actions. "
        f"Constraints: {json.dumps(constraints, separators=(',', ':'))}."
    )


class AILinkerClient:
    def __init__(self, base_url: str, timeout_s: float = 3.0) -> None:
        self.base = base_url.rstrip("/")
        self.timeout_s = timeout_s
        self.session = requests.Session()

    def get_context(self) -> Dict[str, Any]:
        r = self.session.get(f"{self.base}/context", timeout=self.timeout_s)
        r.raise_for_status()
        return r.json()

    def get_state(self) -> Dict[str, Any]:
        r = self.session.get(f"{self.base}/state", timeout=self.timeout_s)
        r.raise_for_status()
        return r.json()

    def call_tool(self, tool_name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        if tool_name == "get_robot_capabilities":
            r = self.session.get(f"{self.base}/capabilities", timeout=self.timeout_s)
            r.raise_for_status()
            return r.json()
        if tool_name == "get_robot_state":
            return self.get_state()
        if tool_name == "send_cmd_vel":
            r = self.session.post(f"{self.base}/cmd_vel", json=args, timeout=max(4.0, self.timeout_s + 3.0))
            return r.json()
        if tool_name == "stop_robot":
            r = self.session.post(f"{self.base}/stop", json={}, timeout=self.timeout_s)
            return r.json()
        if tool_name == "estop":
            r = self.session.post(f"{self.base}/estop", json={}, timeout=self.timeout_s)
            return r.json()
        return {"ok": False, "error": f"unknown_tool:{tool_name}"}


@dataclass
class GeminiConfig:
    api_key: str
    model: str = "gemini-2.5-flash-lite"
    timeout_s: float = 20.0
    temperature: float = 0.1


class GeminiClient:
    def __init__(self, cfg: GeminiConfig) -> None:
        self.cfg = cfg
        self.session = requests.Session()

    def generate(
        self,
        system_prompt: str,
        contents: List[Dict[str, Any]],
        function_decls: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        url = (
            f"https://generativelanguage.googleapis.com/v1beta/models/"
            f"{self.cfg.model}:generateContent?key={self.cfg.api_key}"
        )
        payload = {
            "system_instruction": {"parts": [{"text": system_prompt}]},
            "contents": contents,
            "tools": [{"functionDeclarations": function_decls}],
            "generationConfig": {"temperature": self.cfg.temperature},
        }
        r = self.session.post(url, json=payload, timeout=self.cfg.timeout_s)
        r.raise_for_status()
        return r.json()


class Orchestrator:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ailinker = AILinkerClient(args.ailinker_url, timeout_s=args.ailinker_timeout_s)
        self.gemini = GeminiClient(
            GeminiConfig(
                api_key=args.gemini_api_key,
                model=args.gemini_model,
                timeout_s=args.gemini_timeout_s,
                temperature=args.temperature,
            )
        )
        self.metrics = CsvMetricLogger(
            args.metrics_csv_path,
            [
                "wall_time_iso",
                "mono_us",
                "component",
                "event",
                "task_id",
                "step",
                "attempt",
                "tool",
                "ok",
                "latency_us",
                "http_status",
                "error",
                "detail",
            ],
        )

    def _metric(self, **kwargs: object) -> None:
        row = {"component": "behavior_gen", "task_id": self.args.task_id}
        row.update(kwargs)
        self.metrics.log(row)

    def _call_gemini_with_retry(
        self,
        system_prompt: str,
        history: List[Dict[str, Any]],
        fdecls: List[Dict[str, Any]],
        step: int,
    ) -> Optional[Dict[str, Any]]:
        attempt = 0
        backoff_s = max(0.1, self.args.llm_backoff_initial_ms / 1000.0)
        while attempt <= self.args.llm_max_retries:
            t0 = now_mono_us()
            try:
                resp = self.gemini.generate(system_prompt, history, fdecls)
                latency_us = now_mono_us() - t0
                self._metric(event="llm_call", step=step, attempt=attempt, ok=True, latency_us=latency_us)
                return resp
            except HTTPError as exc:
                latency_us = now_mono_us() - t0
                status = exc.response.status_code if exc.response is not None else 0
                self._metric(
                    event="llm_call",
                    step=step,
                    attempt=attempt,
                    ok=False,
                    latency_us=latency_us,
                    http_status=status,
                    error=str(exc),
                )
                retryable = status in (429, 500, 502, 503, 504)
                if not retryable or attempt >= self.args.llm_max_retries:
                    print(f"[orchestrator] Gemini HTTP error status={status}: {exc}")
                    return None
                retry_after = _retry_after_seconds(exc.response)
                sleep_s = max(backoff_s, retry_after)
                print(
                    f"[orchestrator] Gemini rate/server limit (status={status}), "
                    f"retrying in {sleep_s:.1f}s (attempt {attempt + 1}/{self.args.llm_max_retries})"
                )
                time.sleep(sleep_s)
                backoff_s = min(self.args.llm_backoff_max_ms / 1000.0, backoff_s * 1.8)
                attempt += 1
            except RequestException as exc:
                latency_us = now_mono_us() - t0
                self._metric(
                    event="llm_call",
                    step=step,
                    attempt=attempt,
                    ok=False,
                    latency_us=latency_us,
                    error=str(exc),
                )
                if attempt >= self.args.llm_max_retries:
                    print(f"[orchestrator] Gemini request failed: {exc}")
                    return None
                print(
                    f"[orchestrator] Gemini request error, retrying in {backoff_s:.1f}s "
                    f"(attempt {attempt + 1}/{self.args.llm_max_retries})"
                )
                time.sleep(backoff_s)
                backoff_s = min(self.args.llm_backoff_max_ms / 1000.0, backoff_s * 1.8)
                attempt += 1
        return None

    def run(self) -> int:
        context = self.ailinker.get_context()
        if not context.get("ok", False):
            print("[orchestrator] AILinker /context failed")
            return 2

        tools = context.get("tools", [])
        constraints = context.get("constraints", {})
        function_decls = _build_function_decls(tools)
        system_prompt = _system_prompt(constraints)

        state = self.ailinker.get_state()
        history: List[Dict[str, Any]] = [
            {
                "role": "user",
                "parts": [
                    {
                        "text": (
                            f"Task: {self.args.task}\n"
                            f"Initial robot state JSON: {_json_compact(state)}\n"
                            "Plan and execute using tool calls."
                        )
                    }
                ],
            }
        ]

        self._metric(event="task_start", step=0, ok=True, detail=self.args.task)

        try:
            for step in range(self.args.max_steps):
                resp = self._call_gemini_with_retry(system_prompt, history, function_decls, step)
                if resp is None:
                    return 1

                calls, text = _extract_calls_and_text(resp)
                if text:
                    print(f"[model] {text}")
                    self._metric(event="model_text", step=step, ok=True, detail=text[:1000])

                if not calls:
                    if text:
                        break
                    print("[orchestrator] no tool call and no text; stopping")
                    self._metric(event="no_tool_call", step=step, ok=False)
                    break

                for call in calls:
                    name = call["name"]
                    tool_args = call.get("args", {})
                    t0 = now_mono_us()
                    result = self.ailinker.call_tool(name, tool_args)
                    latency_us = now_mono_us() - t0
                    ok = bool(result.get("ok", False))
                    print(f"[tool] {name} args={tool_args} -> {result}")
                    self._metric(
                        event="tool_call",
                        step=step,
                        tool=name,
                        ok=ok,
                        latency_us=latency_us,
                        detail=_json_compact({"args": tool_args, "result": result})[:1500],
                    )

                    history.append({"role": "model", "parts": [{"functionCall": {"name": name, "args": tool_args}}]})
                    history.append(
                        {
                            "role": "user",
                            "parts": [
                                {
                                    "functionResponse": {
                                        "name": name,
                                        "response": {"result": result},
                                    }
                                }
                            ],
                        }
                    )

                if any(c["name"] in ("stop_robot", "estop") for c in calls):
                    break

                if step < self.args.max_steps - 1:
                    state = self.ailinker.get_state()
                    history.append(
                        {
                            "role": "user",
                            "parts": [{"text": f"Latest state JSON: {_json_compact(state)}"}],
                        }
                    )
                    time.sleep(max(0.0, self.args.loop_sleep_ms / 1000.0))
        except KeyboardInterrupt:
            print("[orchestrator] interrupted by user")
            self._metric(event="task_interrupt", ok=False)
            return 130
        finally:
            if self.args.stop_on_exit:
                stop_res = self.ailinker.call_tool("stop_robot", {})
                print(f"[tool] stop_robot -> {stop_res}")
                self._metric(event="stop_on_exit", ok=bool(stop_res.get("ok", False)), detail=_json_compact(stop_res))

        self._metric(event="task_end", ok=True)
        return 0


def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Standalone AI behavior generator (Gemini)")
    p.add_argument("--task", required=True, help="Natural-language task")
    p.add_argument("--task-id", default=f"task_{int(time.time())}")
    p.add_argument("--ailinker-url", default="http://127.0.0.1:7070")
    p.add_argument("--ailinker-timeout-s", type=float, default=3.0)
    p.add_argument("--gemini-model", default="gemini-2.5-flash-lite")
    p.add_argument("--gemini-api-key", default=os.environ.get("GEMINI_API_KEY", ""))
    p.add_argument("--gemini-timeout-s", type=float, default=20.0)
    p.add_argument("--temperature", type=float, default=0.1)
    p.add_argument("--max-steps", type=int, default=2)
    p.add_argument("--loop-sleep-ms", type=int, default=900)
    p.add_argument("--llm-max-retries", type=int, default=2)
    p.add_argument("--llm-backoff-initial-ms", type=int, default=1500)
    p.add_argument("--llm-backoff-max-ms", type=int, default=20000)
    p.add_argument("--metrics-csv-path", default="/tmp/minitb_ai_pipeline/behavior_metrics.csv")
    p.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)
    return p.parse_args()


def main() -> None:
    args = _args()
    if not args.gemini_api_key:
        print("Missing Gemini API key. Set GEMINI_API_KEY or pass --gemini-api-key.", file=sys.stderr)
        sys.exit(2)
    code = Orchestrator(args).run()
    sys.exit(code)


if __name__ == "__main__":
    main()

