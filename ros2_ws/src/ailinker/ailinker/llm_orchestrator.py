#!/usr/bin/env python3
"""Layer 4 AI Behavior Generator (Gemini orchestrator)."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Dict, List

import requests
from requests.exceptions import HTTPError


def _json_compact(value: Any) -> str:
    return json.dumps(value, separators=(",", ":"), ensure_ascii=True)


def _to_gemini_schema(node: Any) -> Any:
    if isinstance(node, dict):
        converted = {}
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
                converted[key] = mapping.get(val.lower(), val)
            else:
                converted[key] = _to_gemini_schema(val)
        return converted
    if isinstance(node, list):
        return [_to_gemini_schema(v) for v in node]
    return node


def _build_function_decls(tools: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    decls = []
    for tool in tools:
        decls.append(
            {
                "name": tool["name"],
                "description": tool.get("description", ""),
                "parameters": _to_gemini_schema(tool.get("parameters", {"type": "object"})),
            }
        )
    return decls


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
            r = self.session.post(f"{self.base}/cmd_vel", json=args, timeout=self.timeout_s)
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
    model: str
    timeout_s: float = 20.0
    temperature: float = 0.2


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


def _extract_calls_and_text(response_json: Dict[str, Any]) -> tuple[List[Dict[str, Any]], str]:
    candidates = response_json.get("candidates", [])
    if not candidates:
        return [], ""
    content = candidates[0].get("content", {})
    parts = content.get("parts", [])
    fn_calls: List[Dict[str, Any]] = []
    text_parts: List[str] = []
    for part in parts:
        if "functionCall" in part:
            fc = part["functionCall"]
            name = fc.get("name")
            args = fc.get("args", {})
            if name:
                fn_calls.append({"name": name, "args": args if isinstance(args, dict) else {}})
        if "text" in part and isinstance(part["text"], str):
            text_parts.append(part["text"])
    return fn_calls, "\n".join(text_parts).strip()


def _build_system_prompt(constraints: Dict[str, Any]) -> str:
    return (
        "You are the MiniTB AI behavior planner. "
        "Always use tools to act on robot state; do not invent sensor values. "
        "Follow safety constraints exactly. "
        "Prefer single-turn execution: emit the full sequence of tool calls in one response whenever possible. "
        "For fixed motion patterns (square, circle, move X seconds), output all required send_cmd_vel calls "
        "and finish with stop_robot in the same response. "
        "Use multi-turn only when the task explicitly depends on new sensor feedback after each action. "
        "Keep plans short, incremental, and reversible. "
        f"Constraints: {json.dumps(constraints, separators=(',', ':'))}. "
        "If task is complete, explain briefly and call stop_robot."
    )


def _retry_after_seconds(resp: requests.Response | None) -> float:
    if resp is None:
        return 0.0
    raw = resp.headers.get("Retry-After", "").strip()
    if not raw:
        return 0.0
    try:
        return max(0.0, float(raw))
    except ValueError:
        return 0.0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Gemini-based orchestrator for AILinker")
    parser.add_argument("--ailinker-url", default="http://127.0.0.1:7070")
    parser.add_argument("--gemini-model", default="gemini-2.5-flash-lite")
    parser.add_argument("--gemini-api-key", default=os.environ.get("GEMINI_API_KEY", ""))
    parser.add_argument("--task", required=True, help="Behavior task text")
    parser.add_argument("--max-steps", type=int, default=20)
    parser.add_argument("--loop-sleep-ms", type=int, default=200)
    parser.add_argument("--llm-max-retries", type=int, default=6)
    parser.add_argument("--llm-backoff-initial-ms", type=int, default=1500)
    parser.add_argument("--llm-backoff-max-ms", type=int, default=20000)
    parser.add_argument("--stop-on-exit", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not args.gemini_api_key:
        print("Missing Gemini API key. Set --gemini-api-key or GEMINI_API_KEY.", file=sys.stderr)
        sys.exit(2)

    ailinker = AILinkerClient(args.ailinker_url)
    context = ailinker.get_context()
    if not context.get("ok", False):
        print("AILinker /context failed", file=sys.stderr)
        sys.exit(2)

    tools = context.get("tools", [])
    constraints = context.get("constraints", {})
    function_decls = _build_function_decls(tools)
    gemini = GeminiClient(GeminiConfig(api_key=args.gemini_api_key, model=args.gemini_model))
    system_prompt = _build_system_prompt(constraints)

    state = ailinker.get_state()
    history: List[Dict[str, Any]] = [
        {
            "role": "user",
            "parts": [
                {
                    "text": (
                        f"Task: {args.task}\n"
                        f"Initial robot state JSON: {_json_compact(state)}\n"
                        "Plan and execute using tool calls."
                    )
                }
            ],
        }
    ]

    try:
        for step in range(args.max_steps):
            attempt = 0
            backoff_s = max(0.1, args.llm_backoff_initial_ms / 1000.0)
            resp: Dict[str, Any] | None = None
            while attempt <= args.llm_max_retries:
                try:
                    resp = gemini.generate(system_prompt, history, function_decls)
                    break
                except HTTPError as exc:
                    status = exc.response.status_code if exc.response is not None else 0
                    is_retryable = status in (429, 500, 502, 503, 504)
                    if not is_retryable or attempt >= args.llm_max_retries:
                        print(f"[orchestrator] Gemini HTTP error status={status}: {exc}")
                        return
                    retry_after_s = _retry_after_seconds(exc.response)
                    sleep_s = max(backoff_s, retry_after_s)
                    print(
                        f"[orchestrator] Gemini rate/server limit (status={status}), "
                        f"retrying in {sleep_s:.1f}s (attempt {attempt + 1}/{args.llm_max_retries})"
                    )
                    time.sleep(sleep_s)
                    backoff_s = min(args.llm_backoff_max_ms / 1000.0, backoff_s * 1.8)
                    attempt += 1
                except requests.RequestException as exc:
                    if attempt >= args.llm_max_retries:
                        print(f"[orchestrator] Gemini request failed: {exc}")
                        return
                    print(
                        f"[orchestrator] Gemini request error, retrying in {backoff_s:.1f}s "
                        f"(attempt {attempt + 1}/{args.llm_max_retries})"
                    )
                    time.sleep(backoff_s)
                    backoff_s = min(args.llm_backoff_max_ms / 1000.0, backoff_s * 1.8)
                    attempt += 1

            if resp is None:
                print("[orchestrator] Gemini returned no response; stopping")
                return
            calls, text = _extract_calls_and_text(resp)
            if text:
                print(f"[model] {text}")

            if not calls:
                if text:
                    break
                print("[orchestrator] no tool call and no text; stopping")
                break

            for call in calls:
                name = call["name"]
                tool_args = call.get("args", {})
                result = ailinker.call_tool(name, tool_args)
                print(f"[tool] {name} args={tool_args} -> {result}")
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

            # If the model already issued a terminal safety action, do not spend
            # extra API calls on another planning turn.
            if any(c["name"] in ("stop_robot", "estop") for c in calls):
                break

            if step < args.max_steps - 1:
                state = ailinker.get_state()
                history.append(
                    {
                        "role": "user",
                        "parts": [{"text": f"Latest state JSON: {_json_compact(state)}"}],
                    }
                )
                time.sleep(max(0.0, args.loop_sleep_ms / 1000.0))
    except KeyboardInterrupt:
        print("[orchestrator] interrupted by user")
    finally:
        if args.stop_on_exit:
            try:
                stop_res = ailinker.call_tool("stop_robot", {})
                print(f"[tool] stop_robot -> {stop_res}")
            except Exception as exc:
                print(f"[orchestrator] stop_robot failed: {exc}")


if __name__ == "__main__":
    main()
